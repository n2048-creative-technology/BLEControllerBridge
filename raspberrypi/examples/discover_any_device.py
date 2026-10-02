#!/usr/bin/env python3
"""
Example: discover an unknown BLE controller on a Raspberry Pi, log
everything, then lock onto it once identified. Mirrors the ESP32
DiscoverAnyDevice.ino example.

Run with: sudo python3 discover_any_device.py
(root is required for BlueZ adapter discovery/pairing via D-Bus)
"""

import sys
sys.path.insert(0, "..")

from ble_controller_bridge import BLEControllerBridge


def main():
    bridge = BLEControllerBridge()

    # --- Uncomment ONE of these once you've identified your device: -----
    # bridge.set_target_address("AA:BB:CC:DD:EE:FF")
    # bridge.set_target_name("MyGamepad")

    bridge.on_scan_result(lambda name, addr, rssi:
        print(f"[SCAN] name=\"{name}\" addr={addr} rssi={rssi}"))

    bridge.on_connect(lambda name, addr:
        print(f"[CONNECT] Connected to {name} ({addr})"))

    bridge.on_disconnect(lambda: print("[DISCONNECT] Lost connection"))

    bridge.on_raw_data(lambda evt:
        print(f"[RAW] svc={evt.service_uuid} chr={evt.characteristic_uuid} "
              f"data={evt.data.hex(' ')}"))

    bridge.on_button(lambda evt:
        print(f"[BUTTON] mask=0x{evt.button_mask:08X} changed=0x{evt.changed_mask:08X}"))

    print("Scanning (put your controller into pairing mode now)...")
    found = bridge.scan_and_connect(timeout_s=30)
    if not found:
        print("No matching device found within timeout.")
        return

    print("Listening for notifications. Press Ctrl+C to stop.")
    bridge.run()


if __name__ == "__main__":
    main()
