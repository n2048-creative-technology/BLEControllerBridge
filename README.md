# BLEControllerBridge

Zero-dependency BLE controller input bridge. Discover, pair, bond
(persisted across reboots), and stream input from **any** BLE remote or
gamepad — commercial HID controllers *and* fully custom/no-name BLE
peripherals with non-standard GATT services — into a simple event-driven
API, for use in robotics, animatronics, and automation projects.

Two platform targets, one shared design:

| Platform | Language | Dependencies |
|---|---|---|
| `esp32/` | C++ (Arduino) | **None** beyond the `esp32:esp32` Arduino core's bundled BLE library |
| `raspberrypi/` | Python 3 | **None** beyond `python3-dbus` + `python3-gi`, preinstalled on Raspberry Pi OS |

Both expose the same conceptual API (`on_connect`/`onConnect`,
`on_button`/`onButton`, `on_raw_data`/`onRawData`, etc.) so controller
logic can be ported between an ESP32 firmware and a Raspberry Pi service
with minimal rewriting.

---

## Why this exists

Commercial "supported controller" lists (Bluepad32, game-controller SDKs,
etc.) only cover devices with dedicated vendor parsers. Cheap/no-name BLE
remotes — rings, shutter buttons, generic sensors — usually aren't on any
list, but they're still ordinary BLE peripherals: they advertise, they can
be connected to, and (if they're a HID device or expose any GATT
notify/indicate characteristic) they send data. This library makes that
universal path the default, with a convenience layer on top for the common
"it's basically a button bitmask" case.

**Design principle:** always get *some* data from *any* BLE device
(`onRawData`), with an optional heuristic (`onButton`) layered on top for
the common case — never a hard allowlist you have to extend per device.

---

## Architecture

```
┌─────────────────────────────────────────────┐
│           BLEControllerBridge (facade)        │  <- what most code uses
│  onConnect / onDisconnect / onButton /        │
│  onRawData / scanAndConnect / loop            │
└───────────────┬───────────────┬──────────────┘
                │               │
     ┌──────────▼──────┐  ┌─────▼────────────────┐  ┌────────────────────┐
     │   BleTransport    │  │  GenericGattMonitor   │  │ HidButtonHeuristic  │
     │ scan/connect/bond │  │ subscribe-to-everything│  │ bitmask diffing    │
     │  (reusable alone) │  │   (reusable alone)     │  │  (reusable alone)   │
     └──────────────────┘  └───────────────────────┘  └────────────────────┘
                                                          ┌────────────────────┐
                                                          │ AnalogAxisExtractor │
                                                          │ configured byte-    │
                                                          │ offset axis decode  │
                                                          │  (reusable alone)   │
                                                          └────────────────────┘
```

Each internal module (`esp32/src/internal/*`) is independently reusable —
e.g. if a future project just needs "find + connect + persist bonding"
with no controller semantics at all, use `BleTransport` directly and skip
the facade. See "Advanced: bypassing the facade" below.

---

## ESP32 (Arduino) usage

### Install

No package-manager install needed for the dependency (it's built into the
ESP32 Arduino core). Install this library itself by cloning/symlinking it
into your Arduino libraries folder:

```bash
git clone https://github.com/n2048-creative-technology/BLEControllerBridge.git
ln -s "$(pwd)/BLEControllerBridge/esp32" ~/Arduino/libraries/BLEControllerBridge
```

Or with `arduino-cli`:
```bash
arduino-cli lib install --git-url https://github.com/n2048-creative-technology/BLEControllerBridge.git
```
(adjust subfolder handling per your arduino-cli version, or use the symlink
method above — the library root is the `esp32/` subdirectory of this repo).

**Board requirement:** any `esp32:esp32` board target (ESP32, ESP32-C3,
ESP32-S3, etc) — this is the vanilla Espressif Arduino core, NOT the
Bluepad32 custom core. Verified against ESP32 Arduino core v3.3.12 on a
Seeed XIAO ESP32-C3.

### Quick start

```cpp
#include <BLEControllerBridge.h>

BLEControllerBridge controller;

void setup() {
  Serial.begin(115200);
  controller.begin("MyRobotController");

  // Leave unfiltered for first-time discovery (see "Finding your
  // controller's identity" below), or lock onto a known device:
  // controller.setTargetAddress("aa:bb:cc:dd:ee:ff");
  // controller.setTargetName("MyGamepad");

  controller.onConnect([](const std::string& name, const std::string& addr) {
    Serial.printf("Connected: %s (%s)\n", name.c_str(), addr.c_str());
  });

  controller.onButton([](const BCBButtonEvent& evt) {
    if (evt.changedMask & 0x01) {           // bit 0 toggled
      bool pressed = evt.buttonMask & 0x01;
      Serial.println(pressed ? "Button 1 DOWN" : "Button 1 UP");
      // -> drive a servo, relay, animatronic trigger, etc. here
    }
  });

  controller.onRawData([](const BCBRawDataEvent& evt) {
    // Always wire this up while bringing up a NEW controller type --
    // it's your ground truth for what the device actually sends.
    Serial.printf("RAW [%s/%s] len=%u: ", evt.serviceUUID.c_str(),
                   evt.characteristicUUID.c_str(), (unsigned)evt.length);
    for (size_t i = 0; i < evt.length; i++) Serial.printf("%02X ", evt.data[i]);
    Serial.println();
  });

  controller.scanAndConnect();  // non-blocking; call loop() to drive it
}

void loop() {
  controller.loop();   // MUST be called every iteration
  delay(10);
}
```

### Full example

See `esp32/examples/DiscoverAnyDevice/DiscoverAnyDevice.ino` for a
runnable sketch that scans, logs every advertisement, connects, and dumps
all raw data — the recommended starting point for any new controller.

### Finding your controller's identity (first-time setup for a new device)

1. Flash the `DiscoverAnyDevice` example as-is (no target filter set).
2. Open Serial Monitor at 115200 baud.
3. Put your controller into pairing/discoverable mode.
4. Watch `[SCAN]` lines for its name/MAC address; it connects to the first
   device found by default.
5. Watch `[RAW]` lines once connected — these are the ground-truth bytes
   your device actually sends on button press/movement. Note the length
   and which bytes change.
6. Set `controller.setTargetAddress("...")` with the real MAC and reflash
   — the device will now be targeted specifically and will auto-reconnect
   (bonded) on every future boot without repeating this process.

### Pairing persistence across reboots

Bonding/link-key storage is handled automatically by the ESP32 BLE stack's
NVS (flash) persistence — this library does nothing special to enable it,
and correspondingly, **nothing accidentally disables it**: `forgetBond()`
is the only call that clears it, and the library never calls it internally.
A bonded device reconnects automatically whenever `scanAndConnect()`/
`loop()` are running and the device is in range and powered on, surviving
power cycles and firmware reflashes (flash erase excepted).

### Writing a custom parser for your controller

Once `onRawData()` has shown you the exact byte layout your controller
uses, write your own parsing instead of relying on the generic
`onButton()` heuristic for anything beyond simple single-byte-bitmask
remotes:

```cpp
controller.onRawData([](const BCBRawDataEvent& evt) {
  // Example: your device always sends a 3-byte report where byte 0 is
  // a button bitmask and bytes 1-2 are a signed 16-bit X axis.
  if (evt.length == 3) {
    uint8_t buttons = evt.data[0];
    int16_t axisX = (int16_t)(evt.data[1] | (evt.data[2] << 8));
    // ... your robotics/animatronics logic here
  }
});
```

### Handling analog axes (joysticks, triggers)

`onButton`/`HidButtonHeuristic` is bitmask-only and has no concept of
analog values — feeding it axis bytes produces meaningless noise. For
joysticks and analog triggers, use `configureAxis()` + `onAxis()`
(`AnalogAxisExtractor`) instead: register exactly where each axis lives in
the raw report (byte offset, width/signedness, deadzone), identified from
`onRawData()` inspection, and the library does the byte decode + change
detection for you on every subsequent packet.

```cpp
controller.onRawData([](const BCBRawDataEvent& evt) {
  // Bring-up step: dump raw bytes while moving the stick/pulling the
  // trigger, note which byte(s) change and their range.
  Serial.printf("len=%u: ", (unsigned)evt.length);
  for (size_t i = 0; i < evt.length; i++) Serial.printf("%02X ", evt.data[i]);
  Serial.println();
});

// Once identified -- e.g. byte 1 is a signed 8-bit left-stick X axis,
// byte 3 is an unsigned 8-bit analog trigger:
controller.configureAxis({/*index=*/0, /*byteOffset=*/1, bcb::AxisWidth::INT8, /*deadzone=*/2});
controller.configureAxis({/*index=*/2, /*byteOffset=*/3, bcb::AxisWidth::UINT8, /*deadzone=*/3});

controller.onAxis([](const BCBAxisEvent& evt) {
  Serial.printf("axis %u = %ld\n", evt.axisIndex, (long)evt.value);
  // -> map evt.value to a servo angle, motor PWM, etc.
});
```

`AxisWidth` options: `UINT8`, `INT8`, `UINT16_LE`, `INT16_LE`,
`UINT16_BE`, `INT16_BE` — covers the field sizes/signedness/endianness
used by essentially all simple BLE gamepad/joystick reports. `deadzone`
suppresses the jitter noise common on cheap analog sticks (ignores
changes smaller than the threshold); set it to 0 for perfectly digital
fields (e.g. a trigger you want millimeter precision on) or a few units
for a physically wobbly stick.

This is an **explicit field-offset decoder, not a HID Report Descriptor
parser** — it won't auto-detect axis locations from the device's own
descriptor the way Bluepad32/a full HID stack would. You tell it where the
bytes are, once, after a short bring-up inspection step. See "Advanced:
bypassing the facade" to use `AnalogAxisExtractor` standalone, decoupled
from `BLEControllerBridge`, if needed.

See `esp32/examples/AnalogAxes/AnalogAxes.ino` for a full runnable
example.

### Multiple controllers

This library's facade (`BLEControllerBridge`) manages one active
connection at a time, matching the common single-controller
robotics/animatronics use case. For multiple simultaneous controllers,
instantiate multiple `bcb::BleTransport` + `bcb::GenericGattMonitor` pairs
directly (see "Advanced: bypassing the facade") — the ESP32's BLE radio
supports multiple concurrent GATT client connections (practical limit is
usually 3-9 depending on core/variant and available RAM).

### Advanced: bypassing the facade

Each internal module is independently usable. Example: a project that
only needs raw GATT notification logging across many characteristics with
its own connection management, skipping button heuristics entirely:

```cpp
#include <internal/GenericGattMonitor.h>

bcb::GenericGattMonitor monitor;
monitor.setNotificationCallback([](const bcb::GattNotification& n) {
  // raw bytes + UUIDs, no bitmask heuristic involved
});
monitor.setEnumeratedCallback([](const bcb::GattCharacteristicInfo& info) {
  Serial.printf("Found characteristic %s (notify=%d)\n",
                info.characteristicUUID.c_str(), info.canNotify);
});
monitor.subscribeToAll(myOwnBLEClient);  // any already-connected BLEClient*
```

---

## Raspberry Pi usage

### Install

No `pip install` needed — the Python module relies only on `python3-dbus`
and `python3-gi`, which ship pre-installed on Raspberry Pi OS (Bookworm+).
If missing (minimal/Lite images):

```bash
sudo apt install python3-dbus python3-gi bluez
```

Then just clone this repo and import the module directly (no packaging
step):

```bash
git clone https://github.com/n2048-creative-technology/BLEControllerBridge.git
cd BLEControllerBridge/raspberrypi
```

### Quick start

```python
from ble_controller_bridge import BLEControllerBridge

bridge = BLEControllerBridge()
bridge.set_target_address("AA:BB:CC:DD:EE:FF")  # or set_target_name(...)

bridge.on_connect(lambda name, addr: print(f"Connected: {name} ({addr})"))

bridge.on_button(lambda evt: print(
    f"button mask=0x{evt.button_mask:08X} changed=0x{evt.changed_mask:08X}"))

bridge.on_raw_data(lambda evt: print(
    f"RAW [{evt.service_uuid}/{evt.characteristic_uuid}] {evt.data.hex(' ')}"))

bridge.scan_and_connect()
bridge.run()   # blocking -- Ctrl+C to stop
```

### Handling analog axes (joysticks, triggers) on the Pi

Mirrors the ESP32 side exactly: register byte offsets identified via
`on_raw_data()` inspection, then consume `on_axis`.

```python
from ble_controller_bridge import BLEControllerBridge, AxisFieldConfig, AxisWidth

bridge = BLEControllerBridge()
bridge.configure_axis(AxisFieldConfig(index=0, byte_offset=1, width=AxisWidth.INT8, deadzone=2))
bridge.configure_axis(AxisFieldConfig(index=2, byte_offset=3, width=AxisWidth.UINT8, deadzone=3))

bridge.on_axis(lambda evt: print(f"axis {evt.axis_index} = {evt.value}"))
```

**Must run as root** (or with appropriate D-Bus/BlueZ policy-kit rules) —
BlueZ's adapter control, discovery, and pairing D-Bus methods require
privileged access by default: `sudo python3 your_script.py`.

### Full example

See `raspberrypi/examples/discover_any_device.py` — mirrors the ESP32
`DiscoverAnyDevice` example: scan, log, connect, dump raw bytes.

### Pairing persistence across reboots

BlueZ persists paired/bonded device link keys under `/var/lib/bluetooth/`
automatically once `Pair()` succeeds (called internally during
`scan_and_connect()`). No extra code needed; `forget_bond()` is the
explicit opt-out (calls BlueZ's `RemoveDevice`).

### Alternative: `bluetoothctl` for one-off debugging

Before writing any code, it's often fastest to confirm a device's
discoverability and basic GATT structure with the stock CLI tool (also
zero-dependency, ships with `bluez`):

```bash
sudo bluetoothctl
[bluetoothctl]> scan on
# ... watch for your device's name/address ...
[bluetoothctl]> pair AA:BB:CC:DD:EE:FF
[bluetoothctl]> connect AA:BB:CC:DD:EE:FF
[bluetoothctl]> menu gatt
[bluetoothctl]> list-attributes
# Note characteristic UUIDs/flags, then exit and use the Python library
# for actual notification handling.
```

---

## Shared concepts (both platforms)

### Event model

| ESP32 (C++) | Raspberry Pi (Python) | Fires when |
|---|---|---|
| `onConnect` | `on_connect` | GATT connection established (+ services resolved) |
| `onDisconnect` | `on_disconnect` | Connection dropped (any reason, incl. out-of-range) |
| `onScanResult` | `on_scan_result` | Every advertisement seen during a scan (match or not) |
| `onButton` | `on_button` | Heuristic button-bitmask diff (see below) |
| `onAxis` | `on_axis` | Configured analog axis field changed beyond deadzone (see below) |
| `onRawData` | `on_raw_data` | Every raw notification from every subscribed characteristic |

### The `onButton` heuristic — what it is and isn't

Both platforms implement the exact same heuristic: treat the first
(up to) 4 bytes of any notification **8 bytes or shorter** as a
little-endian button bitmask, and fire a changed-bits diff against the
previous report. This works well for simple remotes/rings/shutter buttons
that report "which buttons are currently held" as a flat bitmask — which
covers a large fraction of cheap commercial BLE remotes.

It is **not** a full USB HID Report Descriptor parser and will not
correctly extract analog joystick axes, D-pad hat-switch encoding, or
multi-byte composite reports from more complex gamepads. For those, use
`onRawData`/`on_raw_data` to inspect the actual bytes and write a
device-specific parser (see the ESP32 "Writing a custom parser" section
above — the same approach applies verbatim on the Pi side in Python).

### Filters

Both platforms support the same three filter modes, evaluated in this
priority order:
1. **Custom filter** (`setDeviceFilter`/`set_device_filter`) — full
   name/address/RSSI logic, highest priority if set.
2. **Address filter** (`setTargetAddress`/`set_target_address`) — exact
   MAC match, case-insensitive.
3. **Name filter** (`setTargetName`/`set_target_name`) — substring match
   against the advertised name.
4. **No filter** — connects to the first connectable device found (use
   only for initial discovery, never in production, since it'll grab
   whatever BLE peripheral is nearby).

---

## Verified hardware / software versions

- ESP32 side: Seeed XIAO ESP32-C3, `esp32:esp32` Arduino core v3.3.12,
  compiled and flashed successfully via `arduino-cli`; confirmed live over
  serial scanning real nearby BLE devices.
- Raspberry Pi side: syntax-verified against Python 3's `dbus`/`gi`
  bindings API surface as shipped on Raspberry Pi OS Bookworm; BlueZ
  D-Bus API per the upstream `bluez` D-Bus API docs
  (`doc/device-api.txt`, `doc/gatt-api.txt`, `doc/adapter-api.txt`).

## License

MIT — see `LICENSE`.
