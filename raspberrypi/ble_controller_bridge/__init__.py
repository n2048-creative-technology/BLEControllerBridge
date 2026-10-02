"""
BLEControllerBridge for Raspberry Pi -- zero pip-install-dependency BLE
controller bridge using BlueZ's D-Bus API directly.

DEPENDENCIES: only `python3-dbus` and `python3-gi`, which ship PRE-INSTALLED
on Raspberry Pi OS (Bookworm and later) as part of the base system -- no
`pip install` required. This mirrors the ESP32 library's "zero external
dependency" design: both rely solely on what the platform already bundles
(ESP32 Arduino core's built-in BLE lib / Raspberry Pi OS's built-in BlueZ
D-Bus stack), nothing fetched from Library Manager / PyPI.

If `python3-dbus`/`python3-gi` are somehow missing (minimal/Lite images),
install with:
    sudo apt install python3-dbus python3-gi bluez

Mirrors the ESP32 library's conceptual API 1:1 so a project can share
button-handling logic between an ESP32 firmware and a Raspberry Pi service:
    bridge = BLEControllerBridge()
    bridge.on_connect(...)
    bridge.on_disconnect(...)
    bridge.on_raw_data(...)
    bridge.on_button(...)
    bridge.set_target_address("AA:BB:CC:DD:EE:FF")   # or set_target_name(...)
    bridge.scan_and_connect()
    bridge.run()   # blocking main loop (GLib mainloop under the hood)

See ../../README.md for the full guide and HidButtonHeuristic details.
"""

from __future__ import annotations

import subprocess
import time
from dataclasses import dataclass
from typing import Callable, Optional

import dbus
import dbus.mainloop.glib
from gi.repository import GLib

BLUEZ_SERVICE = "org.bluez"
ADAPTER_IFACE = "org.bluez.Adapter1"
DEVICE_IFACE = "org.bluez.Device1"
GATT_CHAR_IFACE = "org.bluez.GattCharacteristic1"
GATT_SERVICE_IFACE = "org.bluez.GattService1"
DBUS_OM_IFACE = "org.freedesktop.DBus.ObjectManager"
DBUS_PROP_IFACE = "org.freedesktop.DBus.Properties"


@dataclass
class ButtonEvent:
    button_mask: int
    changed_mask: int


@dataclass
class RawDataEvent:
    data: bytes
    service_uuid: str
    characteristic_uuid: str


@dataclass
class AxisEvent:
    axis_index: int
    value: int


class AxisWidth:
    """Mirrors bcb::AxisWidth on the ESP32 side."""
    UINT8 = "uint8"
    INT8 = "int8"
    UINT16_LE = "uint16_le"
    INT16_LE = "int16_le"
    UINT16_BE = "uint16_be"
    INT16_BE = "int16_be"


@dataclass
class AxisFieldConfig:
    index: int
    byte_offset: int
    width: str  # one of AxisWidth.*
    deadzone: int = 0


class AnalogAxisExtractor:
    """Mirrors bcb::AnalogAxisExtractor on the ESP32 side: explicit,
    configured byte-offset decoding -- NOT a HID report descriptor parser.
    Register axis fields after identifying their layout via on_raw_data()
    inspection (move the stick/trigger, see which bytes change)."""

    _WIDTH_SIZES = {
        AxisWidth.UINT8: 1,
        AxisWidth.INT8: 1,
        AxisWidth.UINT16_LE: 2,
        AxisWidth.INT16_LE: 2,
        AxisWidth.UINT16_BE: 2,
        AxisWidth.INT16_BE: 2,
    }

    def __init__(self):
        self._axes: dict[int, AxisFieldConfig] = {}
        self._last_values: dict[int, int] = {}

    def configure_axis(self, config: AxisFieldConfig):
        self._axes[config.index] = config
        self._last_values.pop(config.index, None)

    def clear_axes(self):
        self._axes.clear()
        self._last_values.clear()

    def _decode(self, data: bytes, config: AxisFieldConfig) -> Optional[int]:
        size = self._WIDTH_SIZES[config.width]
        if config.byte_offset + size > len(data):
            return None
        chunk = data[config.byte_offset:config.byte_offset + size]
        if config.width == AxisWidth.UINT8:
            return chunk[0]
        if config.width == AxisWidth.INT8:
            return int.from_bytes(chunk, "little", signed=True)
        if config.width == AxisWidth.UINT16_LE:
            return int.from_bytes(chunk, "little", signed=False)
        if config.width == AxisWidth.INT16_LE:
            return int.from_bytes(chunk, "little", signed=True)
        if config.width == AxisWidth.UINT16_BE:
            return int.from_bytes(chunk, "big", signed=False)
        if config.width == AxisWidth.INT16_BE:
            return int.from_bytes(chunk, "big", signed=True)
        return None

    def feed(self, data: bytes):
        events = []
        for index, config in self._axes.items():
            value = self._decode(data, config)
            if value is None:
                continue
            last = self._last_values.get(index)
            if last is None or abs(value - last) > config.deadzone:
                self._last_values[index] = value
                events.append(AxisEvent(axis_index=index, value=value))
        return events


class HidButtonHeuristic:
    """Same heuristic as the ESP32 library's HidButtonHeuristic: treats the
    first up-to-4 bytes of any short (<=8 byte) notification as a button
    bitmask and diffs it against the previous report. Convenience only --
    verify against raw_data events for your specific controller."""

    def __init__(self):
        self._last_mask: Optional[int] = None

    def feed(self, data: bytes) -> Optional[ButtonEvent]:
        if not data or len(data) > 8:
            return None
        n = min(len(data), 4)
        mask = int.from_bytes(data[:n], byteorder="little")
        if self._last_mask is None:
            self._last_mask = mask
            return None
        changed = mask ^ self._last_mask
        self._last_mask = mask
        if changed:
            return ButtonEvent(button_mask=mask, changed_mask=changed)
        return None


class BLEControllerBridge:
    def __init__(self, adapter: str = "hci0"):
        dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
        self._bus = dbus.SystemBus()
        self._adapter_path = f"/org/bluez/{adapter}"
        self._mainloop = GLib.MainLoop()

        self._target_address: Optional[str] = None
        self._target_name: Optional[str] = None
        self._device_filter: Optional[Callable[[str, str, int], bool]] = None

        self._on_connect: Optional[Callable[[str, str], None]] = None
        self._on_disconnect: Optional[Callable[[], None]] = None
        self._on_scan_result: Optional[Callable[[str, str, int], None]] = None
        self._on_button: Optional[Callable[[ButtonEvent], None]] = None
        self._on_axis: Optional[Callable[[AxisEvent], None]] = None
        self._on_raw_data: Optional[Callable[[RawDataEvent], None]] = None

        self._heuristic = HidButtonHeuristic()
        self._axis_extractor = AnalogAxisExtractor()
        self._connected_device_path: Optional[str] = None
        self._signal_matches = []

        self._ensure_adapter_powered()

    # --- Setup -------------------------------------------------------------

    def _ensure_adapter_powered(self):
        adapter_obj = self._bus.get_object(BLUEZ_SERVICE, self._adapter_path)
        props = dbus.Interface(adapter_obj, DBUS_PROP_IFACE)
        if not bool(props.Get(ADAPTER_IFACE, "Powered")):
            props.Set(ADAPTER_IFACE, "Powered", True)

    def set_target_address(self, mac_address: str):
        self._target_address = mac_address.upper()
        self._target_name = None
        self._device_filter = None

    def set_target_name(self, name_substring: str):
        self._target_name = name_substring
        self._target_address = None
        self._device_filter = None

    def set_device_filter(self, fn: Callable[[str, str, int], bool]):
        """fn(name, address, rssi) -> bool: return True to accept."""
        self._device_filter = fn

    def clear_target_filter(self):
        self._target_address = None
        self._target_name = None
        self._device_filter = None

    # --- Event subscriptions -------------------------------------------------

    def on_connect(self, fn: Callable[[str, str], None]):
        self._on_connect = fn

    def on_disconnect(self, fn: Callable[[], None]):
        self._on_disconnect = fn

    def on_scan_result(self, fn: Callable[[str, str, int], None]):
        self._on_scan_result = fn

    def on_button(self, fn: Callable[[ButtonEvent], None]):
        self._on_button = fn

    def on_axis(self, fn: Callable[[AxisEvent], None]):
        self._on_axis = fn

    def configure_axis(self, config: AxisFieldConfig):
        self._axis_extractor.configure_axis(config)

    def clear_axes(self):
        self._axis_extractor.clear_axes()

    def on_raw_data(self, fn: Callable[[RawDataEvent], None]):
        self._on_raw_data = fn

    # --- Discovery / connection ----------------------------------------------

    def _evaluate_filter(self, name: str, address: str, rssi: int) -> bool:
        if self._device_filter:
            return self._device_filter(name, address, rssi)
        if self._target_address:
            return address.upper() == self._target_address
        if self._target_name:
            return self._target_name in (name or "")
        return True

    def _object_manager(self):
        return dbus.Interface(
            self._bus.get_object(BLUEZ_SERVICE, "/"), DBUS_OM_IFACE
        )

    def scan_and_connect(self, timeout_s: float = 0):
        """Starts BlueZ discovery and polls for a matching device. Blocking
        up to timeout_s (0 = no timeout, poll until found or stopped)."""
        adapter_obj = self._bus.get_object(BLUEZ_SERVICE, self._adapter_path)
        adapter = dbus.Interface(adapter_obj, ADAPTER_IFACE)
        try:
            adapter.StartDiscovery()
        except dbus.exceptions.DBusException as e:
            if "InProgress" not in str(e):
                raise

        start = time.time()
        found_path = None
        seen = set()
        while True:
            objects = self._object_manager().GetManagedObjects()
            for path, interfaces in objects.items():
                dev = interfaces.get(DEVICE_IFACE)
                if not dev:
                    continue
                address = str(dev.get("Address", ""))
                name = str(dev.get("Name", "") or dev.get("Alias", ""))
                rssi = int(dev.get("RSSI", 0))
                if path not in seen:
                    seen.add(path)
                    if self._on_scan_result:
                        self._on_scan_result(name, address, rssi)
                if self._evaluate_filter(name, address, rssi):
                    found_path = path
                    break
            if found_path:
                break
            if timeout_s and (time.time() - start) > timeout_s:
                break
            time.sleep(0.5)

        adapter.StopDiscovery()

        if found_path:
            self._connect_to(found_path)
        return found_path is not None

    def _connect_to(self, device_path: str):
        dev_obj = self._bus.get_object(BLUEZ_SERVICE, device_path)
        device = dbus.Interface(dev_obj, DEVICE_IFACE)
        props = dbus.Interface(dev_obj, DBUS_PROP_IFACE)

        device.Connect()
        # Trigger/request pairing + bonding (BlueZ persists bond info under
        # /var/lib/bluetooth automatically once paired -- this is what
        # gives reboot-persistent pairing on the Pi side too).
        try:
            device.Pair()
        except dbus.exceptions.DBusException as e:
            if "AlreadyExists" not in str(e) and "Already Paired" not in str(e):
                pass  # pairing may not be required/supported by all devices

        name = str(props.Get(DEVICE_IFACE, "Name") or "")
        address = str(props.Get(DEVICE_IFACE, "Address") or "")
        self._connected_device_path = device_path

        # Wait briefly for GATT service resolution.
        for _ in range(20):
            if bool(props.Get(DEVICE_IFACE, "ServicesResolved")):
                break
            time.sleep(0.5)

        self._subscribe_to_all_characteristics(device_path)

        # Watch for disconnection.
        self._bus.add_signal_receiver(
            self._on_properties_changed,
            dbus_interface=DBUS_PROP_IFACE,
            signal_name="PropertiesChanged",
            path=device_path,
            path_keyword="path",
        )

        if self._on_connect:
            self._on_connect(name, address)

    def _on_properties_changed(self, interface, changed, invalidated, path=None):
        if interface != DEVICE_IFACE:
            return
        if "Connected" in changed and not bool(changed["Connected"]):
            self._connected_device_path = None
            if self._on_disconnect:
                self._on_disconnect()

    def _subscribe_to_all_characteristics(self, device_path: str):
        objects = self._object_manager().GetManagedObjects()
        for path, interfaces in objects.items():
            if not path.startswith(device_path):
                continue
            chr_props = interfaces.get(GATT_CHAR_IFACE)
            if not chr_props:
                continue

            flags = [str(f) for f in chr_props.get("Flags", [])]
            if "notify" not in flags and "indicate" not in flags:
                continue

            chr_obj = self._bus.get_object(BLUEZ_SERVICE, path)
            chr_iface = dbus.Interface(chr_obj, GATT_CHAR_IFACE)
            char_uuid = str(chr_props.get("UUID", ""))

            # Resolve the owning service UUID for context in events.
            service_path = path.rsplit("/", 1)[0]
            service_props = interfaces_for_path(objects, service_path, GATT_SERVICE_IFACE)
            service_uuid = str(service_props.get("UUID", "")) if service_props else ""

            def make_handler(svc_uuid, c_uuid):
                def handler(interface, changed, invalidated, path=None):
                    if interface != GATT_CHAR_IFACE or "Value" not in changed:
                        return
                    raw = bytes(bytearray(changed["Value"]))
                    if self._on_raw_data:
                        self._on_raw_data(RawDataEvent(raw, svc_uuid, c_uuid))
                    evt = self._heuristic.feed(raw)
                    if evt and self._on_button:
                        self._on_button(evt)
                    for axis_evt in self._axis_extractor.feed(raw):
                        if self._on_axis:
                            self._on_axis(axis_evt)
                return handler

            self._bus.add_signal_receiver(
                make_handler(service_uuid, char_uuid),
                dbus_interface=DBUS_PROP_IFACE,
                signal_name="PropertiesChanged",
                path=path,
                path_keyword="path",
            )
            try:
                chr_iface.StartNotify()
            except dbus.exceptions.DBusException:
                pass  # some characteristics may already be notifying

    def disconnect(self):
        if self._connected_device_path:
            dev_obj = self._bus.get_object(BLUEZ_SERVICE, self._connected_device_path)
            dbus.Interface(dev_obj, DEVICE_IFACE).Disconnect()

    def forget_bond(self):
        """Removes the bonded device entirely from BlueZ (equivalent to
        `bluetoothctl remove <MAC>`), clearing persisted pairing info."""
        if self._connected_device_path:
            adapter_obj = self._bus.get_object(BLUEZ_SERVICE, self._adapter_path)
            adapter = dbus.Interface(adapter_obj, ADAPTER_IFACE)
            adapter.RemoveDevice(self._connected_device_path)

    def run(self):
        """Blocking GLib main loop -- required to receive D-Bus signals
        (notifications) after scan_and_connect(). Call from your main
        thread; Ctrl+C to stop."""
        try:
            self._mainloop.run()
        except KeyboardInterrupt:
            self._mainloop.quit()

    def stop(self):
        self._mainloop.quit()


def interfaces_for_path(objects, path, iface):
    entry = objects.get(path)
    if not entry:
        return None
    return entry.get(iface)
