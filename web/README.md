# WebBLE map viewer

The JDY-33 Web Bluetooth viewer has moved here from the one-off
`tools/` debug workspace. Its HTML/JS content is unchanged.
Run `python -m http.server 8000 --directory web` and open
http://localhost:8000/ble_debug_console.html in Chrome or Edge.
This page displays live 5x5/16x16 `MAPCFG/MAPROW/MAPEND` and the
original maze telemetry. It does not program the MCU Flash.

Historical scripts and evidence are preserved on
[the archived debug-tools branch](https://github.com/HoangAnh301194/Micromouse_STM32F411CEU6_BLE_Debug_Web/tree/archive/debug-tools-precleanup-20261024).
Native regression: `node tests/web/test_ble_map_web.js`.
