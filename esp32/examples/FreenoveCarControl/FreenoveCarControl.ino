// FreenoveCarControl.ino
//
// Drive a Freenove 4WD Car Kit for ESP32 (ESP32-WROVER-B + PCA9685 I2C
// motor driver) using ANY BLE controller/joystick via BLEControllerBridge
// -- including generic no-name VR remotes with no published protocol
// (e.g. Hi-SHOCK VR-Shark, DollaTek VR gamepad) that Bluepad32's supported-
// device list doesn't cover.
//
// DEPENDENCIES for THIS EXAMPLE (not the core BLEControllerBridge library,
// which stays zero-dependency):
//   - BLEControllerBridge (this repo)
//   - PCA9685 Arduino library v3.0.3, bundled by Freenove in their kit's
//     Libraries/PCA9685_v3.0.3.zip -- install via Arduino IDE
//     Sketch > Include Library > Add .ZIP Library, or extract to your
//     Arduino libraries folder. (github.com/Freenove/Freenove_4WD_Car_Kit_for_ESP32)
//
// Board: ESP32-WROVER-B -- select "ESP32 Wrover Module" board
// (FQBN esp32:esp32:esp32wrover) in Arduino IDE/arduino-cli.
//
// SAFETY: this sketch stops all motors immediately on BLE disconnect, and
// on a watchdog timeout if no controller data arrives for >500ms (covers
// the case where the link silently drops without firing a disconnect
// event, or the app hangs/crashes mid-session). A remote-controlled
// vehicle that keeps driving after losing its controller is a real safety
// hazard -- do not remove this behavior.
//
// WORKFLOW for a new/unidentified controller (your VR-Shark or DollaTek
// remote almost certainly fall in this category -- no published protocol):
//  1. Flash BLEControllerBridge's DiscoverAnyDevice.ino example first.
//  2. Move the joystick / pull triggers / press buttons, note the exact
//     byte offsets that change in the [RAW] Serial output.
//  3. Fill in the AXIS_* / BUTTON_* configuration constants below with
//     your controller's real offsets, then flash THIS sketch.

#include <BLEControllerBridge.h>
#include <PCA9685.h>

// ====================================================================
// STEP 1: Fill in these offsets after running DiscoverAnyDevice.ino
// against YOUR controller. Defaults below are PLACEHOLDERS -- a common
// layout for cheap BLE gamepads (1 button byte + 2 axis bytes), but
// verify against your actual device's raw output before trusting it.
// ====================================================================

// Throttle/forward-back axis (typically left stick Y or a trigger pair
// combined into one signed value by your own onRawData() inspection).
#define AXIS_THROTTLE_INDEX   0
#define AXIS_THROTTLE_OFFSET  1
#define AXIS_THROTTLE_WIDTH   bcb::AxisWidth::INT8
#define AXIS_THROTTLE_DEADZONE 4

// Steering axis (typically left/right stick X).
#define AXIS_STEERING_INDEX   1
#define AXIS_STEERING_OFFSET  2
#define AXIS_STEERING_WIDTH   bcb::AxisWidth::INT8
#define AXIS_STEERING_DEADZONE 4

// Assumed raw axis range from the controller (INT8 => -128..127). If your
// controller reports a different range (e.g. 0..255 unsigned, or 16-bit),
// adjust AXIS_RAW_MIN/MAX to match so the mapping to motor speed scales
// correctly.
#define AXIS_RAW_MIN -128
#define AXIS_RAW_MAX 127

// ====================================================================
// STEP 2: Target your specific controller once identified (recommended
// -- prevents accidentally pairing to some other nearby BLE device).
// ====================================================================
// #define TARGET_MAC "aa:bb:cc:dd:ee:ff"

// ====================================================================
// Safety / tuning constants
// ====================================================================
#define MOTOR_SPEED_MAX    4095   // Freenove PCA9685 motor driver's max pulse width
#define WATCHDOG_TIMEOUT_MS 500   // stop motors if no controller data for this long

// ====================================================================
// Freenove PCA9685 motor driver setup (from Freenove_4WD_Car_For_ESP32.cpp,
// Sketches/01.1_Car_Move_and_Turn in the official kit repo)
// ====================================================================
#define PCA9685_SDA 13
#define PCA9685_SCL 14
#ifndef PCA9685_ADDRESS
#define PCA9685_ADDRESS 0x5F
#endif
#define SERVO_FREQUENCY 50
#define PIN_MOTOR_M1_IN1 15  // left-front +
#define PIN_MOTOR_M1_IN2 14  // left-front -
#define PIN_MOTOR_M2_IN1 9   // left-rear +
#define PIN_MOTOR_M2_IN2 8   // left-rear -
#define PIN_MOTOR_M3_IN1 12  // right-front +
#define PIN_MOTOR_M3_IN2 13  // right-rear -
#define PIN_MOTOR_M4_IN1 10  // right-rear +
#define PIN_MOTOR_M4_IN2 11  // right-rear -

PCA9685 pca9685;

void PCA9685_Setup() {
  Wire.begin(PCA9685_SDA, PCA9685_SCL);
  Wire.beginTransmission(PCA9685_ADDRESS);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.endTransmission();
  pca9685.setupSingleDevice(Wire, PCA9685_ADDRESS);
  pca9685.setToFrequency(SERVO_FREQUENCY);
}

// Same signature/semantics as Freenove's own Motor_Move(): each argument
// is -4095..4095, positive = forward, negative = reverse.
void Motor_Move(int m1_speed, int m2_speed, int m3_speed, int m4_speed) {
  m1_speed = constrain(m1_speed, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX);
  m2_speed = constrain(m2_speed, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX);
  m3_speed = constrain(m3_speed, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX);
  m4_speed = constrain(m4_speed, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX);

  auto applyMotor = [](int in1Pin, int in2Pin, int speed) {
    if (speed >= 0) {
      pca9685.setChannelPulseWidth(in1Pin, speed);
      pca9685.setChannelPulseWidth(in2Pin, 0);
    } else {
      pca9685.setChannelPulseWidth(in1Pin, 0);
      pca9685.setChannelPulseWidth(in2Pin, -speed);
    }
  };

  applyMotor(PIN_MOTOR_M1_IN1, PIN_MOTOR_M1_IN2, m1_speed);
  applyMotor(PIN_MOTOR_M2_IN1, PIN_MOTOR_M2_IN2, m2_speed);
  applyMotor(PIN_MOTOR_M3_IN1, PIN_MOTOR_M3_IN2, m3_speed);
  applyMotor(PIN_MOTOR_M4_IN1, PIN_MOTOR_M4_IN2, m4_speed);
}

void Motor_Stop() {
  Motor_Move(0, 0, 0, 0);
}

// ====================================================================
// Controller -> differential-drive mixing
// ====================================================================

BLEControllerBridge controller;

volatile int32_t throttleRaw = 0;  // last seen raw axis value
volatile int32_t steeringRaw = 0;
volatile unsigned long lastControllerDataMs = 0;
volatile bool controllerConnected = false;

// Normalizes a raw axis reading to -1.0..1.0 given the expected raw range.
float normalizeAxis(int32_t raw) {
  float span = (AXIS_RAW_MAX - AXIS_RAW_MIN) / 2.0f;
  float mid = (AXIS_RAW_MAX + AXIS_RAW_MIN) / 2.0f;
  float v = (raw - mid) / span;
  if (v > 1.0f) v = 1.0f;
  if (v < -1.0f) v = -1.0f;
  return v;
}

void applyDrive() {
  float throttle = normalizeAxis(throttleRaw);  // -1 (full reverse) .. 1 (full forward)
  float steering = normalizeAxis(steeringRaw);  // -1 (full left) .. 1 (full right)

  // Simple arcade-drive mixing: left/right wheel speeds from throttle +
  // steering. Tune the steering multiplier to taste (lower = gentler turns).
  float left = throttle + steering;
  float right = throttle - steering;

  // Normalize if either side exceeds +-1 to preserve the throttle/steering
  // ratio rather than clipping asymmetrically.
  float maxMag = max(abs(left), abs(right));
  if (maxMag > 1.0f) {
    left /= maxMag;
    right /= maxMag;
  }

  int leftSpeed = (int)(left * MOTOR_SPEED_MAX);
  int rightSpeed = (int)(right * MOTOR_SPEED_MAX);

  // m1/m2 = left side, m3/m4 = right side (per Freenove's own convention).
  Motor_Move(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
}

void setup() {
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && millis() - start < 3000) {}

  Serial.println("FreenoveCarControl: BLE joystick -> PCA9685 motor driver");

  PCA9685_Setup();
  Motor_Stop();  // ensure motors are off before any controller is connected

  controller.begin("ESP32-Freenove-Car");

#ifdef TARGET_MAC
  controller.setTargetAddress(TARGET_MAC);
#endif

  controller.configureAxis({AXIS_THROTTLE_INDEX, AXIS_THROTTLE_OFFSET, AXIS_THROTTLE_WIDTH, AXIS_THROTTLE_DEADZONE});
  controller.configureAxis({AXIS_STEERING_INDEX, AXIS_STEERING_OFFSET, AXIS_STEERING_WIDTH, AXIS_STEERING_DEADZONE});

  controller.onConnect([](const std::string& name, const std::string& addr) {
    Serial.printf("[CONNECT] %s (%s)\n", name.c_str(), addr.c_str());
    controllerConnected = true;
    lastControllerDataMs = millis();
  });

  controller.onDisconnect([]() {
    Serial.println("[DISCONNECT] Controller lost -- STOPPING MOTORS");
    controllerConnected = false;
    Motor_Stop();
  });

  controller.onAxis([](const BCBAxisEvent& evt) {
    lastControllerDataMs = millis();
    if (evt.axisIndex == AXIS_THROTTLE_INDEX) {
      throttleRaw = evt.value;
    } else if (evt.axisIndex == AXIS_STEERING_INDEX) {
      steeringRaw = evt.value;
    }
    applyDrive();
  });

  controller.onButton([](const BCBButtonEvent& evt) {
    lastControllerDataMs = millis();
    // Any button press acts as an emergency stop -- customize per your
    // controller's actual button layout once identified via onRawData().
    if (evt.changedMask != 0) {
      Serial.println("[BUTTON] Emergency stop triggered");
      throttleRaw = 0;
      steeringRaw = 0;
      Motor_Stop();
    }
  });

  // Always keep raw data visible during bring-up/tuning.
  controller.onRawData([](const BCBRawDataEvent& evt) {
    Serial.printf("[RAW] len=%u: ", (unsigned)evt.length);
    for (size_t i = 0; i < evt.length; i++) Serial.printf("%02X ", evt.data[i]);
    Serial.println();
  });

  controller.scanAndConnect();
}

void loop() {
  controller.loop();

  // Watchdog: if we lose the data stream without a clean disconnect event
  // (e.g. radio interference, app crash on the controller side), stop the
  // car anyway rather than keep driving on stale input.
  if (controllerConnected && (millis() - lastControllerDataMs > WATCHDOG_TIMEOUT_MS)) {
    Motor_Stop();
  }

  delay(10);
}
