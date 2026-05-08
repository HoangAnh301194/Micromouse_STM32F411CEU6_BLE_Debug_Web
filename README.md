# Nezumi Chan Micromouse Firmware

Firmware điều khiển robot micromouse chạy trên `STM32F411CE` (Black Pill), viết theo hướng bare-metal với `CMSIS`, không dùng HAL. Dự án này tập trung vào một bộ robot maze nhỏ để thử nghiệm thuật toán dò mê cung, chạy lại theo đường tối ưu, hiệu chuẩn cảm biến IR, bám tường, quay mượt và quan sát trạng thái qua BLE.

Hiện tại firmware đang được cấu hình cho một **mê cung thực hành 5x5** với ô đích tại `(4,4)`. Một số tên hàm vẫn giữ từ logic micromouse cổ điển như `Center`, nhưng trong cấu hình hiện tại hãy hiểu đó là **goal cell** của bài test.

## Điểm nổi bật

- Điều khiển chuyển động có encoder + MPU6050 + PID heading/speed
- Hệ IR 6 mắt đọc tường với hiệu chuẩn nhiều bước và hysteresis
- Maze solver gồm flood-fill, persistent map và A* run
- Lưu map, IR calibration và MPU calibration trực tiếp vào flash nội
- OLED menu để chạy trực tiếp trên robot
- BLE debug console để xem log và trực quan hóa mê cung
- System test menu tách riêng để kiểm tra từng primitive

## Công nghệ và môi trường

- MCU: `STM32F411CE`
- Build system: `PlatformIO`
- Framework: `cmsis`
- Upload: `ST-Link`
- UI on-robot: `SSD1306 OLED`
- Inertial sensor: `MPU6050`
- Debug transport: `USART2 + JDY-33 BLE`

## Cấu trúc repo

```text
.
|-- README.md
|-- platformio.ini
|-- docs/
|   |-- architecture.md
|   `-- archive/
|-- include/
|-- lib/
|-- src/
`-- tools/
    |-- README.md
    `-- ble_debug_console.html
```

### Ý nghĩa từng vùng

- `src/`: toàn bộ firmware C của robot
- `include/`: header công khai cho từng module
- `docs/architecture.md`: bản đồ module, luồng runtime và điểm mở rộng
- `docs/archive/`: ghi chú kỹ thuật của các phiên chỉnh sửa trước
- `tools/ble_debug_console.html`: web console để xem BLE log và render maze
- `lib/`: vùng dành cho thư viện nội bộ nếu sau này tách module dùng chung

## Bản đồ module firmware

### 1. Application layer

- `src/main.c`: điểm vào hệ thống, menu chính, state machine của explore/A*
- `src/systemTest.c`: test menu và các bài test primitive

### 2. Motion and control

- `src/motion_controller.c`: straight, turn, smooth turn, speed profile, PID loop
- `src/pid.c`: cấu trúc PID cơ bản
- `src/back_align.c`: primitive lùi tường để tái chuẩn pose sau khi quay

### 3. Sensing and calibration

- `src/ir_sensor.c`: đọc 6 cảm biến IR bằng DMA
- `src/ir_simple_calib.c`: quy trình calibration và ngưỡng phát hiện tường
- `src/mpu6050.c`: driver gyro/yaw
- `src/sensor_fusion.c`: hợp nhất encoder + gyro + IR để sinh trạng thái dùng cho control

### 4. Navigation and persistence

- `src/maze_solver.c`: flood-fill, smart exploration, A*, export maze cho BLE
- `src/flash_storage.c`: lưu map và calibration vào flash nội

### 5. Platform drivers

- `src/hardware.c`: init GPIO, ADC, timer, motor, safety hooks
- `src/system_timer.c`, `src/timer.c`: timebase và timer helpers
- `src/encoder.c`: quadrature encoder
- `src/i2c.c`: I2C blocking + DMA
- `src/uart.c`: debug UART
- `src/bt_debug.c`: UART2 BLE transport
- `src/tb6612fng.c`: driver motor bridge

### 6. UI

- `src/ssd1306.c`, `src/fonts.c`: OLED rendering

## Luồng chạy chính

1. `main()` khởi tạo hardware, UART/BLE, OLED, encoder, IR, motor, MPU.
2. Nạp dữ liệu persistent từ flash: maze map, IR calibration, MPU calibration.
3. Hiển thị menu trên OLED.
4. Người dùng dùng 1 nút:
   - nhấn ngắn để đổi lựa chọn
   - giữ để chạy mode hiện tại
5. Trong explore/A*:
   - đọc tường từ IR
   - cập nhật maze
   - quyết định hành động tiếp theo
   - thực thi motion primitive
   - phát log/cell update qua BLE

## Các mode trong menu chính

| Mode | Mục đích |
|---|---|
| `1.Calibration` | Chạy quy trình hiệu chuẩn IR và MPU |
| `2.Run Slow` | Explore mode tốc độ chậm, dễ quan sát |
| `3.Run Faster` | Explore mode nhanh hơn |
| `4.A* Run` | Chạy lại theo đường tối ưu từ map đã lưu |
| `5.Speedrun` | Placeholder, chưa hoàn thiện |
| `6.Reset Map` | Xóa persistent map trong flash |
| `7.System Test` | Vào menu test primitive |

## Các mode trong System Test

| Test | Ý nghĩa |
|---|---|
| `1.IR` | Kiểm tra đọc IR và ngưỡng cảm biến |
| `2.Align` | Test front wall alignment |
| `3.Pivot` | Test smooth/pivot turn |
| `4.TrStr` | Transition straight test |
| `5.TrTn` | Transition turn test |
| `6.SPHT` | Sensor presence hysteresis test |
| `7.T1WT` | One-wheel turn / smooth turn trigger test |

## Phần cứng và ánh xạ chính

Các macro đầy đủ nằm trong `include/pinout.h`. Những chân quan trọng:

| Khối | Kết nối chính |
|---|---|
| Motor PWM | `PA15`, `PB3` qua `TIM2` |
| Motor DIR | `PB12`, `PB13`, `PB14`, `PB15` |
| Encoder | `TIM3` (`PB4/PB5`), `TIM4` (`PB6/PB7`) |
| IR RX | `PB1`, `PB0`, `PA6`, `PA5`, `PA4`, `PA1` |
| IR TX | `PA7`, `PB10`, `PB2`, `PA8`, `PA11`, `PA12` |
| I2C | `PB8/PB9` |
| OLED | I2C address `0x3C` |
| MPU6050 | I2C address `0x68` |
| BLE | `USART2` trên `PA2/PA3` |
| User button | `PA0` |
| Status LED | `PC13` |

Thông số robot đang hardcode:

- Cell size: `180.0 mm`
- Wheel diameter: `35.0 mm`
- Wheel base: `84.0 mm`
- Encoder PPR: `1430`

## Build và nạp firmware

### Yêu cầu

- `PlatformIO Core`
- `ST-Link`
- board `STM32F411CE` đang nối đúng driver

### Lệnh cơ bản

```bash
pio run
pio run -t upload
pio device monitor -b 115200
```

### Cấu hình build hiện tại

- environment: `genericSTM32F411CE`
- framework: `cmsis`
- upload protocol: `stlink`

## Quy trình sử dụng đề xuất

### Lần đầu bring-up

1. Flash firmware.
2. Vào `Calibration`.
3. Chạy đủ các bước IR/MPU calibration.
4. Vào `Run Slow` để kiểm tra hướng đi và log.
5. Quan sát log qua BLE console.
6. Khi map đã đủ tin cậy, chạy `A* Run`.

### Nếu robot đổi hành vi sau khi chỉnh cơ khí

1. Reset map nếu dữ liệu cũ không còn đáng tin.
2. Chạy lại calibration.
3. Test từng primitive trong `System Test` trước khi quay lại explore.

## Calibration và dữ liệu persistent

### Lưu ở flash nội

- `Sector 6 @ 0x08040000`: IR calibration
- `Sector 6 + 256`: MPU calibration
- `Sector 7 @ 0x08060000`: persistent maze map

### Ý nghĩa

- IR calibration giúp ngưỡng phát hiện tường phù hợp với robot thực tế
- Persistent maze giúp robot không phải học lại từ đầu sau mỗi lần reset
- MPU calibration giảm drift yaw trong motion control

## BLE debug console

Mở file [tools/ble_debug_console.html](tools/ble_debug_console.html) bằng Chrome hoặc Edge trên desktop.

Console hỗ trợ:

- kết nối Web Bluetooth tới module JDY-33
- xem log dạng text
- render maze, flood value, path và robot pose
- nhận `CELL:` update từng bước và `MAZE:` full dump

Xem thêm trong [tools/README.md](tools/README.md).

## Tài liệu bổ sung

- [Kiến trúc firmware](docs/architecture.md)
- [Ghi chú thay đổi 2026-03-20](docs/archive/2026-03-20-session-changes.md)
- [Ghi chú calibration cũ](docs/archive/ir-calibration-walkthrough.md)

## Giới hạn hiện tại

- `main.c` và `systemTest.c` còn rất lớn, đóng vai trò orchestration trung tâm
- `Speedrun` vẫn là placeholder
- Cấu hình maze hiện tại là `5x5`, chưa phải profile thi đấu 16x16
- Nếu nâng maze size lớn hơn, nên xem lại BLE transport và visualizer

## Hướng tái sử dụng

Nếu muốn dùng repo này làm nền cho robot khác, nên bắt đầu theo thứ tự:

1. cập nhật `include/pinout.h`
2. xác nhận hướng encoder/motor trong `motion_controller.c` và `hardware.c`
3. hiệu chỉnh lại IR thresholds bằng menu calibration
4. kiểm tra từng primitive trong `System Test`
5. sau đó mới đổi solver hoặc mở rộng maze size

