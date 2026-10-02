// Example: a controller with both buttons AND analog axes (joystick/
// trigger). Demonstrates configureAxis()/onAxis() in active use.
//
// WORKFLOW:
//  1. First bring up your device with DiscoverAnyDevice.ino and onRawData()
//     to find the exact byte offsets where your axes live -- move the
//     stick/pull the trigger and watch which bytes change.
//  2. Fill in the configureAxis() calls below with the REAL offsets/widths
//     for YOUR controller (the values here are placeholders).
//  3. Reflash this sketch; onAxis() now fires only for configured fields.

#include <BLEControllerBridge.h>

BLEControllerBridge controller;

void setup() {
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && millis() - start < 3000) {}

  Serial.println("BLEControllerBridge: Analog Axes example");

  controller.begin("ESP32-Analog-Controller");

  // Lock onto a known device once identified (see DiscoverAnyDevice.ino):
  // controller.setTargetAddress("aa:bb:cc:dd:ee:ff");

  controller.onConnect([](const std::string& name, const std::string& address) {
    Serial.printf("[CONNECT] %s (%s)\n", name.c_str(), address.c_str());
  });

  controller.onDisconnect([]() {
    Serial.println("[DISCONNECT] Rescanning...");
  });

  // --- Replace these with YOUR controller's real byte layout ------------
  // Example: byte 1 = left-stick X (signed 8-bit), byte 2 = left-stick Y,
  // byte 3 = analog trigger (unsigned 8-bit, 0-255).
  controller.configureAxis({/*index=*/0, /*byteOffset=*/1, bcb::AxisWidth::INT8, /*deadzone=*/2});
  controller.configureAxis({/*index=*/1, /*byteOffset=*/2, bcb::AxisWidth::INT8, /*deadzone=*/2});
  controller.configureAxis({/*index=*/2, /*byteOffset=*/3, bcb::AxisWidth::UINT8, /*deadzone=*/3});

  controller.onAxis([](const BCBAxisEvent& evt) {
    Serial.printf("[AXIS] index=%u value=%ld\n", evt.axisIndex, (long)evt.value);
    // -> drive a servo position, motor speed, etc. proportionally here
  });

  controller.onButton([](const BCBButtonEvent& evt) {
    Serial.printf("[BUTTON] mask=0x%08X changed=0x%08X\n", evt.buttonMask, evt.changedMask);
  });

  // Always keep raw data wired up during bring-up to sanity-check your
  // axis configuration against the ground truth bytes.
  controller.onRawData([](const BCBRawDataEvent& evt) {
    Serial.printf("[RAW] len=%u: ", (unsigned)evt.length);
    for (size_t i = 0; i < evt.length; i++) Serial.printf("%02X ", evt.data[i]);
    Serial.println();
  });

  controller.scanAndConnect();
}

void loop() {
  controller.loop();
  delay(10);
}
