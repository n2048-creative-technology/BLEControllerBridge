// Example: discover an unknown BLE controller, log everything, then lock
// onto it once identified.
//
// Workflow:
//  1. Flash this AS-IS (no target filter set) and open Serial at 115200.
//  2. Put your controller into pairing mode.
//  3. Watch the [SCAN] lines for its name/address, and [RAW] lines once
//     connected for its actual data format.
//  4. Once you know its address, uncomment the setTargetAddress() line
//     below with the real MAC, reflash, and it'll reconnect to that
//     specific device every time (bonding persists across reboots).

#include <BLEControllerBridge.h>

BLEControllerBridge controller;

void setup() {
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && millis() - start < 3000) {}

  Serial.println("BLEControllerBridge: Discover Any Device example");

  controller.begin("ESP32-Controller-Bridge");

  // --- Uncomment ONE of these once you've identified your device: -------
  // controller.setTargetAddress("aa:bb:cc:dd:ee:ff");
  // controller.setTargetName("MyGamepad");

  controller.onScanResult([](const std::string& name, const std::string& address, int rssi) {
    Serial.printf("[SCAN] name=\"%s\" addr=%s rssi=%d\n", name.c_str(), address.c_str(), rssi);
  });

  controller.onConnect([](const std::string& name, const std::string& address) {
    Serial.printf("[CONNECT] Connected to %s (%s)\n", name.c_str(), address.c_str());
  });

  controller.onDisconnect([]() {
    Serial.println("[DISCONNECT] Lost connection, rescanning...");
  });

  controller.onRawData([](const BCBRawDataEvent& evt) {
    Serial.printf("[RAW] svc=%s chr=%s len=%u data=",
                   evt.serviceUUID.c_str(), evt.characteristicUUID.c_str(), (unsigned)evt.length);
    for (size_t i = 0; i < evt.length; i++) {
      Serial.printf("%02X ", evt.data[i]);
    }
    Serial.println();
  });

  controller.onButton([](const BCBButtonEvent& evt) {
    Serial.printf("[BUTTON] mask=0x%08X changed=0x%08X\n", evt.buttonMask, evt.changedMask);
  });

  // --- Analog axes (joystick/trigger) example -----------------------
  // Uncomment and adjust offsets/widths once you've identified them from
  // [RAW] output above (see README "Handling analog axes"):
  // controller.configureAxis({0, /*byteOffset=*/1, bcb::AxisWidth::INT8, /*deadzone=*/2});
  // controller.configureAxis({1, /*byteOffset=*/2, bcb::AxisWidth::INT8, /*deadzone=*/2});
  // controller.onAxis([](const BCBAxisEvent& evt) {
  //   Serial.printf("[AXIS] index=%u value=%ld\n", evt.axisIndex, (long)evt.value);
  // });

  controller.scanAndConnect();  // scans indefinitely until a match
}

void loop() {
  controller.loop();
  delay(10);
}
