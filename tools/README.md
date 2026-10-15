# Host Tools

## Motor Driver Debug Pipeline

Pipeline hợp nhất: native driver/watchdog, 9 gate offline, build các motor bench, UART neutral, trace TIM11 và phân tích log CV/plateau/slew/STOP. Dấu encoder đã được chủ dự án xác nhận; không thêm gate mức logic 5 V/3,3 V. Không upload/START/mở khóa P0.

```powershell
python tools/motor_driver_debug_pipeline.py --run --build --port COM6
```

Hướng dẫn, dữ liệu và giới hạn kết luận: `docs/motor_driver/DEBUG_PIPELINE.md`.

## Startup Encoder Bench

Đo PWM khởi động từng bánh, không tải; không sửa Motion/PID hoặc tự áp dụng deadband.
Giới hạn 20% PWM, xung 250 ms, nghỉ tối thiểu 750 ms, watchdog phần cứng.
Chỉ chạy khi người vận hành đã xác nhận bánh nâng khỏi mặt đất, VM không quá 8,4 V.

```powershell
python tools/motor_startup_pipeline.py --self-test
python tools/motor_startup_pipeline.py --run --upload --port COM6 --vm-volts 8.4 --confirm-wheels-elevated
```

Chi tiết: `docs/motor_driver/STARTUP_ENCODER_BENCH.md`.

## BLE Debug Console

<p align="center">
  <img src="../assets/webBLEDebug.png" width="95%" alt="Web BLE Debug Console Interface"/>
</p>

The `ble_debug_console.html` is a standalone host-side web application designed to:

- Connect to the robot via Web Bluetooth API (JDY-33 BLE module)
- Monitor real-time UART/BLE log streams
- Render live 2D maze walls, flood-fill distance maps, calculated path, and robot pose

### Getting Started

1. Open `ble_debug_console.html` using a Chromium-based browser (Google Chrome or Microsoft Edge).
2. Power on the robot and the JDY-33 BLE module.
3. Click the **Connect BLE** button in the console header.
4. Select the Bluetooth device broadcasting the `0xFFE0` service.

### Notes

- The console is currently synced with the firmware's **5x5 maze configuration**.
- The robot sends `CELL:` packets for incremental step updates and `MAZE:` packets for complete maze dumps.
- If you change the maze dimensions in the firmware, update the corresponding constants in `ble_debug_console.html`.
