# AI_CODEBASE_MAP.md

Tài liệu bản đồ toàn diện cấu trúc và logic mã nguồn dự án **Nezumi Chan Micromouse Firmware (STM32F411CEU6)** dành cho AI (Claude) kế thừa và xử lý.

> **QUY TẮC BẢO TOÀN**: Tài liệu này phản ánh chính xác 100% **HIỆN TRẠNG MÃ NGUỒN THỰC TẾ** tại thời điểm khảo sát, không thêm thắt suy đoán, không sửa đổi bất kỳ file nào trong dự án.

---

## 1. Cấu Trúc Thư Mục

```text
d:\MICROMOUSE_26\code_mau/
├── .gitignore
├── platformio.ini              # Cấu hình PlatformIO (genericSTM32F411CE, cmsis, stlink)
├── README.md                   # Tài liệu tổng quan dự án phần cứng & phần mềm
├── AI_CODEBASE_MAP.md          # File bản đồ mã nguồn hiện tại (file này)
├── assets/                     # Hình ảnh PCB, 3D render, schematic, console screenshot
│   ├── bottomPCB.png
│   ├── bottomPreview.png
│   ├── reallife.jpg
│   ├── schematic.png
│   ├── topPCB.png
│   ├── topPreview.png
│   └── webBLEDebug.png
├── docs/
│   └── architecture.md         # Tài liệu kiến trúc firmware 6 phân tầng
├── include/                    # Header files cho các module firmware
│   ├── back_align.h            # Macro-maneuver căn lùi tường và phục hồi tọa độ chuẩn
│   ├── bt_debug.h              # Giao tiếp Bluetooth JDY-33 qua USART2 (PA2/PA3)
│   ├── encoder.h               # Đọc xung Encoder từ TIM3 và TIM4 qua bộ đếm phần cứng
│   ├── flash_storage.h         # Đọc/ghi Flash nội bộ (Sector 6: IR/MPU, Sector 7: Maze)
│   ├── fonts.h                 # Định nghĩa font chữ 7x10, 11x18, 16x26 cho OLED
│   ├── hardware.h              # Lớp trừu tượng phần cứng (GPIO, Clocks, JTAG Remap)
│   ├── i2c.h                   # I2C bare-metal thanh ghi (Blocking + DMA cho OLED/MPU)
│   ├── ir_sensor.h             # 6 kênh IR LED/Photodiode, điều chế xung theo cặp, bù ambient
│   ├── ir_simple_calib.h       # Hiệu chuẩn cảm biến khoảng cách IR, hysteresis threshold
│   ├── maze_solver.h           # Thuật toán Flood-Fill, A*, Smart Explore, cấu trúc map 5x5
│   ├── motion_controller.h     # Điều khiển quỹ đạo, PID góc/hướng/vận tốc, smooth turn, T1WT
│   ├── mpu6050.h               # Driver IMU 6 trục MPU6050, lọc LPF/Deadzone/Yaw tích phân
│   ├── pid.h                   # Cấu trúc và thuật toán PID cơ bản (anti-windup)
│   ├── pinout.h                # Định nghĩa toàn bộ chân GPIO, Timer, Kênh ADC, Hằng số vật lý
│   ├── sensor_fusion.h         # Kết hợp dữ liệu Gyro + Encoder + IR (Dynamic Complementary Filter)
│   ├── ssd1306.h               # Driver màn hình OLED SSD1306 128x64 qua I2C + DMA
│   ├── systemTest.h            # Giao diện menu kiểm tra từng module phần cứng
│   ├── system_timer.h          # Timebase microsecond/millisecond sử dụng TIM5 (32-bit)
│   ├── tb6612fng.h             # Driver cầu H TB6612FNG điều khiển động cơ qua PWM
│   ├── timer.h                 # Thao tác mức thanh ghi với các bộ định thời STM32F4
│   └── uart.h                  # Driver USART1 + DMA2 Stream 7 (TX chuyển tiếp qua BLE)
├── src/                        # Mã nguồn thực thi (.c)
│   ├── back_align.c
│   ├── bt_debug.c
│   ├── encoder.c
│   ├── flash_storage.c
│   ├── fonts.c
│   ├── hardware.c
│   ├── i2c.c
│   ├── ir_sensor.c
│   ├── ir_simple_calib.c
│   ├── main.c                  # ĐẶC BIỆT: Hiện tại chứa code FFT Test (Code Micromouse bị comment)
│   ├── maze_solver.c
│   ├── motion_controller.c
│   ├── mpu6050.c
│   ├── pid.c
│   ├── sensor_fusion.c
│   ├── ssd1306.c
│   ├── systemTest.c
│   ├── system_timer.c
│   ├── tb6612fng.c
│   ├── timer.c
│   └── uart.c
└── tools/                      # Công cụ chạy trên máy tính
    ├── README.md               # Hướng dẫn kết nối Web Bluetooth
    └── ble_debug_console.html  # Web App trực quan hóa mê cung 2D và log qua Bluetooth Web API
```

---

## 2. Vai Trò Từng File

| Tên File | Vai Trò | File Gọi/Include | Phụ Thuộc | Module Liên Quan |
|---|---|---|---|---|
| `platformio.ini` | File cấu hình build hệ thống PlatformIO | PlatformIO Core | CMSIS, ststm32 | Build System |
| `pinout.h` | Header trung tâm định nghĩa toàn bộ chân vi điều khiển & thông số cơ khí | Hầu hết các file `.h` và `.c` | `stm32f4xx.h` | Hardware Abstraction |
| `hardware.h` / `hardware.c` | Cấu hình xung nhịp (RCC), tắt JTAG pin (PA15, PB3, PB4), HardFault handler, cầu nối peripheral | `main.c`, `systemTest.c`, `ir_sensor.c`, `motion_controller.c` | `pinout.h`, `tb6612fng.h`, `encoder.h`, `ir_sensor.h`, `system_timer.h` | Platform Drivers |
| `system_timer.h` / `system_timer.c` | Bộ đếm thời gian 32-bit (1us/tick) qua TIM5, cung cấp `micros()`, `millis()`, blocking delay | Hầu hết các file | `stm32f4xx.h` | Core Timing |
| `timer.h` / `timer.c` | Thư viện cấp thấp điều khiển TIM1 - TIM11 ở mức thanh ghi | `tb6612fng.c`, `hardware.c` | `stm32f4xx.h` | Timers |
| `uart.h` / `uart.c` | Driver USART1 dùng DMA2 Stream 7; chuyển tiếp các hàm gửi chuỗi sang BLE | `main.c`, `systemTest.c`, `ir_simple_calib.c`, `motion_controller.c`, `maze_solver.c` | `stm32f4xx.h`, `bt_debug.h` | Communications |
| `bt_debug.h` / `bt_debug.c` | Driver USART2 điều khiển module BLE JDY-33 với vòng đệm tròn ngắt TXE | `uart.c`, `main.c` (commented) | `stm32f4xx.h` | Communications |
| `tb6612fng.h` / `tb6612fng.c` | Điều khiển mạch công suất TB6612FNG (Motor A: TIM2_CH1, Motor B: TIM2_CH2) | `hardware.c` | `stm32f4xx.h` | Actuators |
| `encoder.h` / `encoder.c` | Đọc Encoder quadrature chế độ Encoder Mode 3 qua TIM3 (Right) và TIM4 (Left) | `hardware.c` | `stm32f4xx.h`, `pinout.h`, `system_timer.h` | Sensing |
| `i2c.h` / `i2c.c` | Driver I2C1 (PB8/PB9) hỗ trợ chế độ Blocking và DMA TX (Stream 6) / RX (Stream 0) | `ssd1306.h`, `mpu6050.h` | `stm32f4xx.h`, `system_timer.h` | Peripherals |
| `fonts.h` / `fonts.c` | Bảng mã font chữ điểm ảnh monochrome 7x10, 11x18, 16x26 | `ssd1306.h` | Không | UI Fonts |
| `ssd1306.h` / `ssd1306.c` | Driver màn hình OLED SSD1306 128x64 qua I2C1; cập nhật bằng DMA non-blocking | `main.c` (commented), `systemTest.c` | `i2c.h`, `fonts.h` | UI Display |
| `mpu6050.h` / `mpu6050.c` | Đọc MPU6050 gyro Z qua I2C, lọc LPF/Deadzone và tích phân tính góc Yaw | `sensor_fusion.h`, `main.c`, `systemTest.c` | `i2c.h`, `system_timer.h` | Inertial Sensing |
| `ir_sensor.h` / `ir_sensor.c` | Điều khiển phát xung 6 LED hồng ngoại và lấy mẫu ADC1 qua DMA2 Stream 0 | `hardware.c`, `sensor_fusion.c`, `ir_simple_calib.c`, `main.c` | `pinout.h`, `hardware.c` | Wall Sensing |
| `ir_simple_calib.h` / `ir_simple_calib.c` | Xử lý dữ liệu hiệu chuẩn IR đa bước, cân bằng gain, tính toán ngưỡng Hysteresis | `main.c`, `sensor_fusion.h`, `motion_controller.h`, `systemTest.c` | `ir_sensor.h`, `motion_controller.h`, `system_timer.h`, `uart.h` | Calibration |
| `pid.h` / `pid.c` | Khối tính toán thuật toán PID phổ quát kèm bộ chống bão hòa tích phân (anti-windup) | `sensor_fusion.h` | Không | Control Logic |
| `back_align.h` / `back_align.c` | Thao tác lùi xe chạm tường đuôi để đặt lại điểm mốc (canonical pose) | `main.c` (commented) | `motion_controller.h`, `hardware.h`, `system_timer.h`, `uart.h` | Motion Primitive |
| `sensor_fusion.h` / `sensor_fusion.c` | Bộ lọc kết hợp động lực (Complementary Filter) dung hợp Gyro + Encoder + tính sai số bám tường | `motion_controller.c`, `systemTest.c`, `main.c` | `ir_simple_calib.h`, `mpu6050.h`, `hardware.h`, `ir_sensor.h`, `system_timer.h` | State Estimation |
| `motion_controller.h` / `motion_controller.c` | Bộ điều khiển chuyển động chính: đi thẳng, quay góc, T1WT, bám tường, căn lề tường trước | `main.c`, `systemTest.c`, `back_align.c`, `ir_simple_calib.c` | `pinout.h`, `sensor_fusion.h`, `mpu6050.h`, `hardware.h`, `system_timer.h`, `uart.h` | Motion Engine |
| `maze_solver.h` / `maze_solver.c` | Thuật toán mê cung (5x5): Flood-Fill, A*, Smart Exploration theo lượt ghé, gói tin BLE | `main.c`, `flash_storage.h` | `uart.h` | Navigation & Algorithms |
| `flash_storage.h` / `flash_storage.c` | Đọc và ghi dữ liệu cố định vào Flash nội bộ STM32F4 (Sector 6 và Sector 7) | `main.c` | `ir_simple_calib.h`, `maze_solver.h`, `uart.h` | Storage / Persistence |
| `systemTest.h` / `systemTest.c` | Bộ test độc lập toàn bộ các chức năng phần cứng và chuyển động cơ bản | `main.c` | `hardware.h`, `ssd1306.h`, `sensor_fusion.h`, `motion_controller.h`, `ir_simple_calib.h`, `system_timer.h`, `uart.h` | Diagnostics |
| `main.c` | Điểm vào chính (`main()`). **Lưu ý đặc biệt**: Dòng 1-2865 (Menu, Maze Explore, A*) đang bị comment `//`; Dòng 2868-3237 là chương trình kiểm tra FFT IR đang chạy | MCU Boot | `stm32f4xx.h`, `ir_sensor.h`, `uart.h`, `system_timer.h`, `arm_math.h` | Application Entry |
| `ble_debug_console.html` | Trang web HTML5 giao tiếp Web Bluetooth API để hiển thị mê cung 2D, flood fill, pose robot | Chạy trên trình duyệt client | Web Bluetooth API | External Tool |

---

## 3. Cấu Trúc Chi Tiết Của Từng File

### `include/pinout.h`
- **Includes**: `<stdint.h>`, `"stm32f4xx.h"`
- **Động cơ**:
  - Motor A (Phải): PWM = `PA15` (TIM2_CH1, AF1), IN1 = `PB15`, IN2 = `PB14`
  - Motor B (Trái): PWM = `PB3` (TIM2_CH2, AF1), IN1 = `PB13`, IN2 = `PB12`
  - PWM Freq: 20kHz, Prescaler: 0, Period (ARR): 4999
- **Encoder**:
  - Phải: `PB4` (CH1), `PB5` (CH2) qua `TIM3`, AF2
  - Trái: `PB6` (CH1), `PB7` (CH2) qua `TIM4`, AF2
  - PPR: 1430
- **Cảm biến IR (6 cặp)**:
  - Thu (ADC1): L90=`PB1` (IN9), L45=`PB0` (IN8), L0=`PA6` (IN6), R0=`PA5` (IN5), R45=`PA4` (IN4), R90=`PA1` (IN1)
  - Phát (GPIO OUT): L90=`PA7`, L45=`PB10`, L0=`PB2`, R0=`PA8`, R45=`PA11`, R90=`PA12`
- **I2C & Ngoại vi**:
  - I2C1: SCL=`PB8`, SDA=`PB9` (AF4). MPU6050=`0x68`, OLED=`0x3C`
  - USART1 (Debug): TX=`PA9`, RX=`PA10` (AF7), Baud: 115200
  - USART2 (JDY-33 BLE): TX=`PA2`, RX=`PA3` (AF7)
  - Phím bấm: KEY=`PA0` (Active LOW: `BUTTON_PRESSED = 0`)
  - Đèn LED trạng thái: `PC13` (**Active LOW**: `LED_ON()` là `RESET_PIN`, `LED_OFF()` là `SET_PIN`)
- **Thông số vật lý robot**:
  - Đường kính bánh xe: `35.0 mm`
  - Chiều rộng cơ sở (Wheel Base): `84.0 mm`
  - Encoder PPR: `1430`

### `src/hardware.c` & `include/hardware.h`
- **Includes**: `"ir_sensor.h"`, `"hardware.h"`, `"tb6612fng.h"`, `"encoder.h"`, `"system_timer.h"`, `<stdio.h>`
- **Biến tĩnh**:
  - `static TB6612_Handle_t motor_driver`
  - `static Encoder_Handle_t encoder_right`, `encoder_left`
- **Hàm đáng chú ý**:
  - `Hardware_DisableJTAG()`: Thiết lập chân `PA15`, `PB3`, `PB4` thoát khỏi trạng thái JTAG để dùng làm PWM và Encoder.
  - `Hardware_Init()`: Bật clock AHB1/APB1/APB2, tắt JTAG, cấu hình GPIO, bật SystemTimer.
  - `Hardware_GetBatteryVoltage()`: Hiện trả về giá trị cố định `8.2f` (chưa đọc kênh ADC đo pin thực).
  - `HardFault_Handler()`: Ghi đè vector ngắt của CMSIS. Khi MCU gặp HardFault: lập tức ghi trực tiếp `TIM2->CCR1 = 0`, `TIM2->CCR2 = 0` ngắt motor, tắt `TIM11` (PID) và `TIM10` (IR scan), khóa vòng lặp vô tận `while(1)`.

### `src/system_timer.c` & `include/system_timer.h`
- **Tác vụ**: Đọc cấu hình RCC (`GetSystemClock`, `GetAPB1TimerClock`) để tự tính toán xung nhịp thực của APB1, cấu hình `TIM5` chạy ở tần số chính xác 1MHz (1us/tick).
- **Hàm chính**:
  - `SystemTimer_Init()`: Kích hoạt `TIM5` (32-bit counter, ARR = `0xFFFFFFFF`).
  - `micros()`: Trả về `TIM5->CNT`.
  - `millis()`: Trả về `TIM5->CNT / 1000`.
  - `delay_us_blocking()`, `delay_ms_blocking()`.

### `src/timer.c` & `include/timer.h`
- Cung cấp wrapper thanh ghi cho toàn bộ timer từ `TIM1` đến `TIM11`.
- Cấu hình cơ chế ngắt cập nhật (UIE) và cấu hình kênh PWM độc lập.

### `src/uart.c` & `include/uart.h`
- Cấu hình USART1 trên chân `PA9/PA10` với baudrate 115200.
- `UART_DMA_Init()`: Cấu hình `DMA2_Stream7_Channel4` để truyền chuỗi non-blocking với bộ đệm vòng `uart_tx_buffer[1024]`.
- **ĐẶC BIỆT**: Các hàm `UART_SendChar()`, `UART_SendString()`, `UART_SendNumber()` trong `uart.c` đã bị chuyển hướng (redirect) gọi trực tiếp `BT_SendChar()`, `BT_SendString()` qua module BLE JDY-33!

### `src/bt_debug.c` & `include/bt_debug.h`
- Cấu hình USART2 trên chân `PA2/PA3` tốc độ 115200.
- Không dùng DMA cho USART2 vì DMA1 Stream 6 bị xung đột với I2C1 TX DMA. Thay vào đó, driver sử dụng ngắt cờ trống thanh ghi truyền (`TXEIE` - `USART2_IRQHandler`) phối hợp bộ đệm vòng `bt_tx_buffer[4096]`.
- Cung cấp hàm định dạng: `BT_Printf(const char *fmt, ...)`, `BT_SendString(const char *str)`. Nếu bộ đệm đầy, driver sẽ tự động bỏ qua (drop) dữ liệu thay vì block để tránh làm treo PID loop.

### `src/tb6612fng.c` & `include/tb6612fng.h`
- Driver cho mạch công suất cầu H đôi TB6612FNG.
- Động cơ A (Right) dùng TIM2 Channel 1; Động cơ B (Left) dùng TIM2 Channel 2.
- `TB6612_SetMotor(handle, motor, dir, speed)`: Chuyển đổi mức phần trăm 0-100 thành duty cycle của Timer ARR (0-4999).

### `src/encoder.c` & `include/encoder.h`
- Sử dụng chế độ đếm phần cứng Encoder Mode 3 trên `TIM3` (Bánh phải) và `TIM4` (Bánh trái).
- `Encoder_GetTotalCount()`: Tính toán delta giữa các lần đọc và cộng dồn vào `total_count` 32-bit (xử lý tràn 16-bit).
- `Encoder_UpdateSpeed()`: Tính vận tốc xung trên giây (PPS - Pulses Per Second) qua bộ lọc thông thấp số EMA (`LOW_PASS_ALPHA = 0.3f`).

### `src/i2c.c` & `include/i2c.h`
- Quản lý giao tiếp bus I2C1 trên chân `PB8/PB9` (100kHz hoặc 400kHz).
- Có cơ chế phục hồi bus kẹt (`I2C_BusRecovery()`: toggle chân SCL 9 lần).
- Hỗ trợ cả 2 chế độ:
  - **Blocking**: Cho các thao tác ghi thanh ghi cấu hình lúc khởi tạo (`I2C_Write`, `I2C_Read`, `I2C_WriteRegister`, `I2C_ReadRegister`).
  - **Non-blocking DMA**: Dùng `DMA1_Stream6_Channel1` cho TX (OLED) và `DMA1_Stream0_Channel1` cho RX (MPU6050).
  - Có timeout bảo vệ (~500us loop counter) trong `I2C_ReadReg_DMA` để không bị deadlock khi gọi từ trong ngắt ngắt mức 0 (TIM11 ISR).

### `src/ssd1306.c` & `include/ssd1306.h`
- Driver hiển thị OLED SSD1306 128x64 pixels.
- Quản lý bộ đệm khung hình `SSD1306_Buffer[1024]`.
- `SSD1306_UpdateScreen_DMA()`: Gửi dữ liệu bộ đệm qua DMA non-blocking theo từng page (tổng cộng 8 trang), phối hợp với hàm `SSD1306_Process_DMA()` được gọi trong vòng lặp chính.

### `src/mpu6050.c` & `include/mpu6050.h`
- Driver cảm biến góc quay MPU6050 qua I2C địa chỉ `0x68`.
- Thang đo Gyro: cấu hình dải `500 deg/s` (`MPU6050_GYRO_RANGE = 1`, tỉ lệ `65.5 LSB/(deg/s)`).
- Bộ lọc: Low-Pass Filter số (`GYRO_Z_LPF_ALPHA = 0.7f`), Deadzone (`0.3 deg/s`), Moving Average (`GYRO_Z_AVG_SAMPLES = 3`).
- `MPU6050_Calibrate()`: Lấy mẫu tĩnh (`CALIBRATION_SAMPLES = 200`) để tính giá trị `gyro_z_offset`.
- `MPU6050_Update()`: Đọc dữ liệu trục Z qua DMA non-blocking và tích phân theo thời gian $\Delta t$ để tính góc `yaw`.

### `src/ir_sensor.c` & `include/ir_sensor.h`
- Hệ thống đo khoảng cách 6 kênh:
  - `L90` (Trước-Trái), `L45` (Chéo-Trái), `L0` (Hông-Trái), `R0` (Hông-Phải), `R45` (Chéo-Phải), `R90` (Trước-Phải).
- Cấu hình ADC1 quét 6 kênh qua DMA2 Stream 0 Channel 0.
- Điều khiển phát xung bằng bộ định thời `TIM10` (chu kỳ 100us/bước) thông qua state machine 7 trạng thái:
  1. `IR_STATE_AMBIENT`: Tắt toàn bộ LED phát, kích hoạt ADC đo ánh sáng môi trường.
  2. `IR_STATE_PAIR1_ON`: Bật cặp `L90` + `R45`.
  3. `IR_STATE_PAIR1_READ`: Lấy mẫu, trừ giá trị ambient, tắt LED.
  4. `IR_STATE_PAIR2_ON`: Bật cặp `L45` + `R90`.
  5. `IR_STATE_PAIR2_READ`: Lấy mẫu, trừ ambient, tắt LED.
  6. `IR_STATE_PAIR3_ON`: Bật cặp `L0` + `R0`.
  7. `IR_STATE_PAIR3_READ`: Lấy mẫu, trừ ambient, tắt LED $\rightarrow$ Chuyển `IR_STATE_READY`.
- Toàn bộ chu trình đo chỉ mất **700 microsecond** và loại trừ hoàn toàn nhiễu chéo (cross-talk) giữa các cảm biến gần nhau.

### `src/ir_simple_calib.c` & `include/ir_simple_calib.h`
- Thuật toán hiệu chuẩn cảm biến tường:
  - Đo mẫu tĩnh hoặc động (robot tịnh tiến 20mm trong quá trình lấy mẫu).
  - Sử dụng Median Filter lọc nhiễu ngoại lai.
  - Cân bằng độ lợi (Gain balance): `gain_l0`, `gain_r0`, `gain_l45`, `gain_r45`, `gain_l90`, `gain_r90` để đưa đáp ứng của hai bên về cùng tỉ lệ.
  - Ngưỡng Hysteresis:
    $$\text{gap} = B - A$$
    $$\text{WALL\_ON} = A + \text{gap} \times \frac{2}{3}$$
    $$\text{WALL\_OFF} = A + \text{gap} \times \frac{1}{7}$$
    (với $A$ là giá trị không có tường - Open space, $B$ là giá trị tại tâm ô - Center).
  - Nhận diện cạnh tường xuất hiện / biến mất thông qua đạo hàm (`l45_drop_th`, `l45_rise_th`).

### `src/pid.c` & `include/pid.h`
- Bộ điều khiển PID chuẩn:
  $$u(t) = K_p \cdot e(t) + K_i \int e(t)dt + K_d \frac{de(t)}{dt}$$
- Kèm giới hạn chống bão hòa tích phân (`limit_integral`) và giới hạn đầu ra (`limit_output`).

### `src/back_align.c` & `include/back_align.h`
- Cơ chế phục hồi vị trí hình học thông minh:
  - Khi robot rẽ góc mà phía sau có bức tường tựa: robot lùi xe với công suất thấp (`REVERSE_PWM = -40`).
  - Phát hiện kẹt (Stall Detection): Nếu số xung Encoder của cả 2 bánh thay đổi nhỏ hơn 5 xung trong 200ms $\rightarrow$ robot đã ép sát thành tường.
  - Tự động gọi `Motion_SnapHeading()` đưa góc Yaw về góc vuông ($0^\circ, 90^\circ, 180^\circ, 270^\circ$).
  - Đặt lại vị trí trục bánh xe và đi tiến ra một khoảng chính xác `BACK_ALIGN_RECOVER_MM` để đặt tâm xe đúng trọng tâm ô cờ.

### `src/sensor_fusion.c` & `include/sensor_fusion.h`
- Dung hợp cảm biến:
  - Dynamic Complementary Filter: Kết hợp tốc độ góc Gyro Z và góc quay tính từ chênh lệch xung Encoder hai bánh. Hệ số tin cậy Gyro $\alpha$ tự động giảm từ 0.98 xuống 0.85 khi quay gắt để tránh trôi góc (drift).
  - Tính toán sai số bám tường `wall_steering_error`: Chuẩn hóa độ lệch sang dải $[-1.0, +1.0]$ dựa trên cảm biến `L0` và `R0`.

### `src/motion_controller.c` & `include/motion_controller.h`
- Trái tim điều khiển chuyển động:
  - Chạy ngắt thời gian thực cứng **TIM11 tại tần số 1000Hz (1ms)** với mức ưu tiên ngắt cao nhất (Priority 0).
  - Quản lý State Machine: `MOTION_IDLE`, `MOTION_STRAIGHT`, `MOTION_STRAIGHT_CONSTANT`, `MOTION_TURN_LEFT`, `MOTION_TURN_RIGHT`, `MOTION_TURN_180`, `MOTION_TURN_STABILIZING`, `MOTION_SMOOTH_LEFT`, `MOTION_SMOOTH_RIGHT`, `MOTION_FRONT_ALIGN`, `MOTION_T1WT_*`.
  - Hỗ trợ cả 2 chế độ:
    1. **Open-loop Profile** (Dùng cho Explore): Khống chế PWM theo hình thang tăng tốc và giảm tốc kết hợp bù hướng PID Gyro.
    2. **Closed-loop Speed PID** (Dùng cho A* Run): Đo vận tốc thực từ Encoder, nội suy vận tốc mong muốn theo quy luật động học $v = \sqrt{2ad}$, điều khiển qua Feedforward + Speed PID.
  - Căn chỉnh tường trước `MOTION_FRONT_ALIGN`: Dùng tổng phản hồi $F_{\text{sum}} = L_{90} + R_{90}$ để chỉnh khoảng cách và hiệu số $F_{\text{diff}} = L_{90} - R_{90}$ để chỉnh góc song song thành tường.

### `src/maze_solver.c` & `include/maze_solver.h`
- Quản lý giải mê cung kích thước $5 \times 5$:
  - Ô đích mặc định: $(4, 4)$ (Macro: `MAZE_CENTER_X1 = 4`, `MAZE_CENTER_Y1 = 4`).
  - `Maze_Persistent_t`: Lưu thông tin bản đồ dài hạn (tường, độ tin cậy, số lượt đi qua ô) tồn tại qua nhiều lượt chạy.
  - `Maze_t`: Bộ nhớ làm việc trong một lần chạy.
  - `Maze_FloodFill()`: Loang khoảng cách ngược từ đích về toàn bộ mê cung.
  - `Maze_GetNextMove_Smart()`: Quyết định hướng rẽ ưu tiên chọn ô có số lượt ghé thăm ít nhất để tối đa hóa độ phủ khám phá.
  - `Maze_CalculateOptimalPath()`: Thuật toán A* tìm đường đi ngắn nhất với ma trận trạng thái $(x, y, \text{direction})$ và chi phí chuyển trạng thái (tiến, rẽ $90^\circ$, rẽ $180^\circ$).
  - Truyền thông tin giám sát: `Maze_SendCellUpdate()` gửi gói `CELL:X,Y,W,RD,FF\r\n`, và `Maze_PrintCompact()` gửi chia nhỏ 8 chunk `MZ0`...`MZ7`.

### `src/flash_storage.c` & `include/flash_storage.h`
- Lưu trữ trên Flash nội bộ vi điều khiển:
  - **Sector 6** (`0x08040000`): Lưu thông số hiệu chuẩn IR (`IR_Simple_Calib_t`, Magic: `0xCA`) và offset MPU6050 (`MPU_Calib_Flash_t`, Offset 256, Magic: `0xEB`).
  - **Sector 7** (`0x08060000`): Lưu bản đồ mê cung lâu dài (`Maze_Persistent_t`, Magic: `0xA5`).
  - Tự động sao lưu và khôi phục khi xóa Sector 6 do cấu trúc dùng chung sector.

### `src/systemTest.c` & `include/systemTest.h`
- Hệ thống menu test phần cứng tương tác qua phím bấm và hiển thị OLED:
  1. `1.IR`: Hiển thị ADC và trạng thái phát hiện tường trực tiếp.
  2. `2.Align`: Kiểm tra thuật toán căn chỉnh vuông góc tường trước.
  3. `3.Pivot`: Kiểm tra quay pivot tại chỗ $90^\circ, 180^\circ$.
  4. `4.TrStr`: Kiểm tra chạy chuyển tiếp thẳng không dừng giữa các ô.
  5. `5.TrTn`: Kiểm tra quay chuyển tiếp liên tục.
  6. `6.SPHT`: Test độ trễ Hysteresis của cảm biến cạnh.
  7. `7.T1WT`: Kiểm tra rẽ một bánh (One-wheel turn).

### `src/main.c` (TÌNH TRẠNG HIỆN TẠI)
- **Đoạn 1 (Dòng 1 đến 2865)**: Toàn bộ khung code chính của Micromouse (Menu điều khiển OLED, Máy trạng thái Explore, Máy trạng thái A* Run, Hiệu chuẩn đa bước, Đón tay bắt đầu chạy `WaitForHandStart`, Lưu Flash) đang **bị comment lại bằng `//`**.
- **Đoạn 2 (Dòng 2868 đến 3237)**: Mã nguồn thực thi độc lập đang hoạt động (`int main(void)` tại dòng 3179):
  - Khởi tạo clock, UART1, Cảm biến IR.
  - Thu thập 256 mẫu ADC trên 6 kênh cảm biến hồng ngoại (`Collect_IR_Data`).
  - Kiểm tra bão hòa ADC (`Check_Saturation`).
  - Gửi dữ liệu thô dạng CSV qua UART (`Send_Raw_Data`).
  - Thực hiện biến đổi Fourier nhanh (FFT) bằng thư viện `arm_math.h` (`Calculate_FFT`) để phân tích phổ tín hiệu nhiễu / đáp ứng tần số của cảm biến.
  - Treo tại `while(1)`.

### `tools/ble_debug_console.html`
- Giao diện web chạy trên máy tính kết nối BLE với JDY-33 qua dịch vụ chuẩn `0xFFE0` / đặc tính `0xFFE1`.
- Phân tích luồng văn bản:
  - Nhận diện `CELL:x,y,w,dir,ff` để cập nhật trực tiếp ô cờ lên lưới $5 \times 5$.
  - Nhận diện `MZ0:` đến `MZ7:` để giải mã toàn bộ bản đồ và hiển thị bản đồ nhiệt Flood-fill.
  - Vẽ đường đi dự kiến của A* và tư thế robot (pose).

---

## 4. Danh Sách Hàm Quan Trọng (Function Catalog)

### Nhóm Quản Lý Phần Cứng & Timing

```text
Function: Hardware_Init
File: src/hardware.c
Input: void
Output: void
Mục đích: Khởi tạo toàn bộ xung nhịp ngoại vi, gỡ bỏ chân JTAG (PB3, PB4, PA15), cấu hình GPIO và bộ đếm thời gian
Được gọi bởi: main() (trong code gốc micromouse)
Gọi các function: Hardware_InitClocks, Hardware_DisableJTAG, Hardware_InitGPIO, Hardware_InitSystemTimer
Tác động đến hardware/state: Bật clock GPIOA/B/C, TIM2/3/4/5; chuyển chân PB3/PA15 thành Output/AF cho motor PWM

Function: HardFault_Handler
File: src/hardware.c
Input: void
Output: void
Mục đích: Xử lý ngoại lệ lỗi phần cứng, bảo vệ cơ khí
Được gọi bởi: CPU NVIC khi phát sinh lỗi ngoại lệ phần cứng (HardFault)
Gọi các function: Không gọi hàm (chỉ ghi trực tiếp thanh ghi)
Tác động đến hardware/state: Ghi TIM2->CCR1 = 0, TIM2->CCR2 = 0 ngắt tức thì PWM động cơ; tắt TIM11 và TIM10; treo while(1)

Function: SystemTimer_Init
File: src/system_timer.c
Input: void
Output: void
Mục đích: Tự động đo xung nhịp APB1 và khởi động TIM5 ở tần số đếm 1MHz
Được gọi bởi: Hardware_InitSystemTimer()
Gọi các function: GetAPB1TimerClock()
Tác động đến hardware/state: Kích hoạt TIM5, ARR=0xFFFFFFFF
```

### Nhóm Cảm Biến & Xử Lý Tín Hiệu

```text
Function: TIM1_UP_TIM10_IRQHandler
File: src/ir_sensor.c
Input: void
Output: void
Mục đích: Trình phục vụ ngắt timer TIM10 (chu kỳ 100us) điều khiển máy trạng thái phát xung IR và kích hoạt ADC DMA
Được gọi bởi: Ngắt phần cứng NVIC (TIM1_UP_TIM10_IRQn)
Gọi các function: Hardware_IRLedAllOff, Hardware_IRLedOn, Hardware_IRLedOff, ADC_TriggerAndWait, memcpy
Tác động đến hardware/state: Bật/tắt các chân GPIO phát IR; đọc kết quả ADC DMA; cập nhật ir_handle.result[]

Function: MPU6050_Update
File: src/mpu6050.c
Input: MPU6050_Handle_t *handle
Output: MPU6050_Status
Mục đích: Đọc vận tốc góc trục Z từ bộ đệm DMA, bù offset, lọc số và tích phân góc Yaw
Được gọi bởi: TIM1_TRG_COM_TIM11_IRQHandler() (mỗi 1ms)
Gọi các function: micros(), LowPassFilter(), DeadzoneFilter(), I2C_ReadReg_DMA()
Tác động đến hardware/state: Kích hoạt DMA1 Stream 0 đọc dữ liệu từ I2C1; cập nhật handle->yaw

Function: SensorFusion_Update
File: src/sensor_fusion.c
Input: Sensor_Fusion_t *sf, float dt
Output: void
Mục đích: Lọc số đọc cảm biến IR, tính vận tốc từ Encoder, chạy bộ lọc Dynamic Complementary Filter cho Heading, tính sai số bám tường
Được gọi bởi: TIM1_TRG_COM_TIM11_IRQHandler() (mỗi 1ms)
Gọi các function: IR_Sensor_GetResults, ReadGyro, ReadEncoders, Hardware_UpdateEncoders, DynamicComplementaryFilter, ComputeWallSteering
Tác động đến hardware/state: Cập nhật vị trí pos_x, pos_y, heading, wall_steering_error
```

### Nhóm Điều Khiển Chuyển Động (Motion Control)

```text
Function: TIM1_TRG_COM_TIM11_IRQHandler
File: src/motion_controller.c
Input: void
Output: void
Mục đích: Vòng lặp điều khiển thời gian thực cứng 1kHz (1ms) cho toàn bộ hệ thống
Được gọi bởi: Ngắt phần cứng NVIC (TIM1_TRG_COM_TIM11_IRQn - Priority 0)
Gọi các function: MPU6050_Update, Hardware_UpdateEncoders, IR_Sensor_StartScan, SensorFusion_Update, Motion_Update
Tác động đến hardware/state: Điều phối luồng xử lý cảm biến và xuất PWM điều khiển động cơ mỗi 1ms

Function: Motion_Update
File: src/motion_controller.c
Input: Motion_Controller_t *mc, float dt
Output: void
Mục đích: Thực thi máy trạng thái điều khiển quỹ đạo, tính toán PID hướng/vận tốc/góc và bám tường, xuất PWM tới TB6612
Được gọi bởi: TIM1_TRG_COM_TIM11_IRQHandler()
Gọi các function: Hardware_GetEncoderCount, MPU6050_GetYaw, PID_Compute, Hardware_SetMotor, Hardware_StopMotors
Tác động đến hardware/state: Xuất xung PWM ra TIM2_CCR1 và TIM2_CCR2

Function: Motion_FrontAlign
File: src/motion_controller.c
Input: Motion_Controller_t *mc, IR_Simple_Calib_t *calib
Output: void
Mục đích: Kích hoạt chế độ căn chỉnh robot tự động song song và chuẩn cự ly với tường trước
Được gọi bởi: Explore / Test loop khi robot cần hiệu chỉnh vị trí trước thành tường
Gọi các function: millis()
Tác động đến hardware/state: Chuyển trạng thái mc->state = MOTION_FRONT_ALIGN

Function: Motion_Turn
File: src/motion_controller.c
Input: Motion_Controller_t *mc, float angle_deg
Output: void
Mục đích: Thiết lập quay tại chỗ một góc mong muốn (-90, +90, 180) bằng bộ điều khiển góc PD
Được gọi bởi: Bộ điều phối Explore / A* / SystemTest
Gọi các function: Hardware_ResetEncoder, MPU6050_GetYaw, PID_Reset
Tác động đến hardware/state: Thiết lập target_angle, chuyển state sang MOTION_TURN_LEFT/RIGHT/180
```

### Nhóm Thuật Toán Mê Cung & Dữ Liệu Flash

```text
Function: Maze_FloodFill
File: src/maze_solver.c
Input: Maze_t *maze, uint8_t target_x, uint8_t target_y
Output: void
Mục đích: Lan truyền sóng (BFS) từ tọa độ đích để tính ma trận khoảng cách flood[][]
Được gọi bởi: Maze_FloodFillCenter(), Execute_MazeExplore()
Gọi các function: HasWall(), IsValidCell()
Tác động đến hardware/state: Cập nhật mảng maze->flood[5][5]

Function: Maze_CalculateOptimalPath
File: src/maze_solver.c
Input: Maze_t *maze, Maze_AStarResult_t *result
Output: uint8_t (1 nếu tìm thấy đường, 0 nếu thất bại)
Mục đích: Chạy thuật toán A* trên đồ thị trạng thái 3D (x, y, hướng) tìm chuỗi lệnh ngắn nhất đến đích
Được gọi bởi: Execute_AStarRun()
Gọi các function: AStar_Heuristic(), HasWall(), IsValidCell()
Tác động đến hardware/state: Điền chuỗi lệnh 'F', 'L', 'R', 'B' vào result->optimal_path[]

Function: Flash_SaveMaze / Flash_LoadMaze
File: src/flash_storage.c
Input: Maze_Persistent_t *pm
Output: void
Mục đích: Ghi/đọc thông tin tường và số lần khám phá vào/ra Flash Sector 7 (0x08060000)
Được gọi bởi: main() / Menu
Gọi các function: Flash_Unlock, Flash_EraseSector, Flash_ProgramWord, Flash_Lock, memcpy
Tác động đến hardware/state: Xóa và lập trình Flash Sector 7
```

---

## 5. Luồng Chương Trình

### 5.1. Luồng Khởi Động & Thực Thi Mã Nguồn Hiện Tại (Active FFT Test)

```text
[Reset Handler / Boot]
         ↓
    SystemInit()           (Clock nội HSI 16MHz hoặc PLL)
         ↓
    UART_Init()            (Khởi tạo USART1 + DMA2 Stream 7)
         ↓
  IR_Sensor_Init()         (ADC1 SCAN mode + DMA2 Stream 0 + TIM10 100us)
         ↓
  Collect_IR_Data()        (Lấy 256 mẫu liên tục trên 6 kênh)
         ↓
  Check_Saturation()       (Kiểm tra vượt ngưỡng bão hòa ADC 4090)
         ↓
   Send_Raw_Data()         (Xuất bảng số liệu CSV qua UART/BLE)
         ↓
   Calculate_FFT()         (Chạy arm_rfft_fast_f32 trên 6 kênh, tìm phổ peak)
         ↓
    while (1) {}           (Dừng chương trình)
```

---

### 5.2. Luồng Chương Trình Gốc Micromouse (Kiến Trúc Dòng 1-2865 trong `main.c`)

```text
[Startup / Boot]
       ↓
Hardware_Init()            (Xung nhịp, tắt JTAG PB3/PA15, cấu hình GPIO)
UART_Init() & BT_Init()    (Khởi tạo USART1 và USART2 JDY-33 BLE)
I2C_Init()                 (Khởi tạo I2C1 400kHz)
SensorFusion_Init()        (MPU6050 init & calib, IR Sensor init)
SSD1306_Init_DMA()         (Khởi tạo OLED hiển thị)
Motion_Init()              (Cấu hình TIM11 1kHz, nạp thông số PID)
Maze_Init()                (Khởi tạo bản đồ 5x5)
Flash_Load*()              (Nạp Maze Map, IR Calib, MPU offset từ Flash)
       ↓
[OLED Main Menu Loop]      (Xử lý nút nhấn PA0: Nhấn ngắn = Đổi mode; Nhấn dài = Chọn)
       ├── 1. Calibration  → Interactive IR & MPU Calibration (Lưu Flash Sector 6)
       ├── 2. Run Slow     → Khám phá tốc độ thấp (Explore State Machine)
       ├── 3. Run Faster   → Khám phá tốc độ cao
       ├── 4. A* Run       → Chạy đường tối ưu bằng thuật toán A*
       ├── 5. Speedrun     → Chạy đua tốc độ (Placeholder)
       ├── 6. Reset Map    → Xóa bản đồ đã lưu trong Flash (Sector 7)
       └── 7. System Test  → Chuyển sang SystemTest_Run()
```

---

### 5.3. Luồng Chi Tiết Vòng Lặp Điều Khiển Ngắt Thời Gian Thực (TIM11 - 1kHz)

```text
TIM11 Update Event (Mỗi 1000us / 1ms)
  │
  ├── 1. MPU6050_Update()
  │        └── Gửi request DMA1 Stream 0 đọc I2C1 Gyro Z
  │        └── Lọc LPF, Deadzone, tích phân Yaw
  │
  ├── 2. Hardware_UpdateEncoders()
  │        └── Đọc TIM3_CNT, TIM4_CNT, tính delta xung & vận tốc PPS
  │
  ├── 3. Kiểm tra IR_Sensor_IsReady()
  │        └── Nếu nhàn rỗi, gọi IR_Sensor_StartScan() kích hoạt TIM10
  │
  ├── 4. SensorFusion_Update()
  │        └── EMA Filter trên 6 kênh IR
  │        └── Dynamic Complementary Filter (Gyro + Encoder)
  │        └── Tính sai số bám tường wall_steering_error
  │
  └── 5. Motion_Update()
           └── Tính khoảng cách đã đi: traveled_distance
           └── Máy trạng thái (STRAIGHT, TURN, FRONT_ALIGN, T1WT, ...)
           └── Tính PID Hướng / Vận tốc / Góc quay / Bám tường
           └── Xuất PWM ra TIM2_CCR1 và TIM2_CCR2 cho TB6612
```

---

## 6. Hardware Mapping Chi Tiết

### 6.1. Bảng Phân Bổ Chân GPIO (Pin Assignment)

| Chân | Hướng I/O | Chức Năng Cấu Hình | Ngoại Vi Liên Quan | Ghi Chú Đặc Biệt |
|---|---|---|---|---|
| `PA0` | Input Pull-up | User Push Button (KEY) | GPIOA | Nhấn = 0, Thả = 1 |
| `PA1` | Analog | IR Receiver R90 (Bên Phải Ngoài) | ADC1_IN1 | Cảm biến thành trước/phải |
| `PA2` | Alternate Func (AF7) | JDY-33 BLE RXD (Robot TX) | USART2_TX | 115200 baud |
| `PA3` | Alternate Func (AF7) | JDY-33 BLE TXD (Robot RX) | USART2_RX | 115200 baud |
| `PA4` | Analog | IR Receiver R45 (Chéo Phải) | ADC1_IN4 | Cảm biến phát hiện cột |
| `PA5` | Analog | IR Receiver R0 (Hông Phải) | ADC1_IN5 | Cảm biến bám tường phải |
| `PA6` | Analog | IR Receiver L0 (Hông Trái) | ADC1_IN6 | Cảm biến bám tường trái |
| `PA7` | Output | IR Emitter LED L90 | GPIOA | Transistor kích LED IR |
| `PA8` | Output | IR Emitter LED R0 | GPIOA | Transistor kích LED IR |
| `PA9` | Alternate Func (AF7) | Debug UART TX | USART1_TX | Chuyển tiếp sang BLE |
| `PA10` | Alternate Func (AF7) | Debug UART RX | USART1_RX | Đang kéo trở pull-up |
| `PA11` | Output | IR Emitter LED R45 | GPIOA | Transistor kích LED IR |
| `PA12` | Output | IR Emitter LED R90 | GPIOA | Transistor kích LED IR |
| `PA15` | Alternate Func (AF1) | Motor Right PWM | TIM2_CH1 | **Chân JTDI - Đã disable JTAG** |
| `PB0` | Analog | IR Receiver L45 (Chéo Trái) | ADC1_IN8 | Cảm biến phát hiện cột |
| `PB1` | Analog | IR Receiver L90 (Trước Trái) | ADC1_IN9 | Cảm biến thành trước/trái |
| `PB2` | Output | IR Emitter LED L0 | GPIOB | Chân BOOT1 |
| `PB3` | Alternate Func (AF1) | Motor Left PWM | TIM2_CH2 | **Chân JTDO - Đã disable JTAG** |
| `PB4` | Alternate Func (AF2) | Encoder Right Channel A | TIM3_CH1 | **Chân JNTRST - Đã disable JTAG** |
| `PB5` | Alternate Func (AF2) | Encoder Right Channel B | TIM3_CH2 | Bộ đếm Encoder bánh phải |
| `PB6` | Alternate Func (AF2) | Encoder Left Channel A | TIM4_CH1 | Bộ đếm Encoder bánh trái |
| `PB7` | Alternate Func (AF2) | Encoder Left Channel B | TIM4_CH2 | Bộ đếm Encoder bánh trái |
| `PB8` | Alternate Func (AF4) | I2C SCL (Bus tốc độ cao) | I2C1_SCL | Kéo trở ngoài (OLED, MPU6050) |
| `PB9` | Alternate Func (AF4) | I2C SDA (Bus tốc độ cao) | I2C1_SDA | Kéo trở ngoài (OLED, MPU6050) |
| `PB10` | Output | IR Emitter LED L45 | GPIOB | Transistor kích LED IR |
| `PB12` | Output | Motor Left BIN2 (Direction) | GPIOB | Cầu H TB6612FNG |
| `PB13` | Output | Motor Left BIN1 (Direction) | GPIOB | Cầu H TB6612FNG |
| `PB14` | Output | Motor Right AIN2 (Direction) | GPIOB | Cầu H TB6612FNG |
| `PB15` | Output | Motor Right AIN1 (Direction) | GPIOB | Cầu H TB6612FNG |
| `PC13` | Output | Status LED Xanh | GPIOC | **Active LOW** (0=Sáng, 1=Tắt) |

---

### 6.2. Phân Bổ Ngoại Vi & DMA (Peripheral & DMA Mapping)

```text
ADC1:
  ├── Kênh quét: IN9 (PB1), IN8 (PB0), IN6 (PA6), IN5 (PA5), IN4 (PA4), IN1 (PA1)
  ├── Chế độ: Scan Mode, DDS=1
  └── DMA: DMA2 Stream 0 Channel 0 (Chuyển 6 word vào ir_handle.dma_buffer)

I2C1:
  ├── Tốc độ: 400kHz (Fast Mode), SCL=PB8, SDA=PB9
  ├── TX DMA (OLED): DMA1 Stream 6 Channel 1
  └── RX DMA (MPU6050): DMA1 Stream 0 Channel 1

USART1 (Wired Debug):
  ├── Chân: PA9 (TX), PA10 (RX)
  └── TX DMA: DMA2 Stream 7 Channel 4

USART2 (JDY-33 BLE):
  ├── Chân: PA2 (TX), PA3 (RX)
  └── Ngắt: USART2_IRQn (Priority 8) - Truyền qua vòng đệm cờ TXE (không dùng DMA)

TIM2 (Động cơ PWM):
  ├── Tần số: 20kHz, ARR = 4999, PSC = 0
  ├── Channel 1: PA15 (Motor Phải)
  └── Channel 2: PB3  (Motor Trái)

TIM3 (Encoder Phải):
  ├── Chế độ: Encoder Mode 3 (Đếm cả 2 sườn A và B)
  └── Chân: PB4 (CH1), PB5 (CH2)

TIM4 (Encoder Trái):
  ├── Chế độ: Encoder Mode 3 (Đếm cả 2 sườn A và B)
  └── Chân: PB6 (CH1), PB7 (CH2)

TIM5 (System High-Precision Timebase):
  ├── Tần số đếm: 1MHz (1us/tick), 32-bit free-running counter
  └── Chức năng: micros() và millis()

TIM10 (Trình quét xung cảm biến IR):
  ├── Chu kỳ: 100us (PSC = 95, ARR = 99 tại 96MHz)
  └── Ngắt: TIM1_UP_TIM10_IRQn (Priority 1)

TIM11 (Vòng lặp điều khiển thời gian thực cứng PID Loop):
  ├── Chu kỳ: 1000us = 1ms (PSC = 95, ARR = 999 tại 96MHz)
  └── Ngắt: TIM1_TRG_COM_TIM11_IRQn (Priority 0 - Cao nhất)
```

---

## 7. Data Flow (Luồng Dữ Liệu)

### 7.1. Luồng Dữ Liệu Cảm Biến Khoảng Cách (IR Sensing Pipeline)

```text
TIM10 (100us ISR)
   │
   ├── Bật cặp LED phát (GPIO)
   ├── Kích hoạt ADC1 qua DMA2 Stream 0
   ├── ADC đọc 6 kênh Photodiode
   ├── Lấy giá trị thô: Raw_ADC[i]
   ├── Trừ giá trị môi trường: Result[i] = Raw_ADC[i] - Ambient[i]
   └── Cập nhật ir_handle.result[] (Chu kỳ hoàn tất: 700us)
         │
         ▼
TIM11 (1ms ISR)
   │
   ├── SensorFusion_Update(): Lọc thông thấp số EMA:
   │     sf->ir_sensors[i] = α * raw + (1 - α) * prev
   │
   ├── Cân bằng độ lợi (Gain balance):
   │     L0_eq = L0 * gain_l0, R0_eq = R0 * gain_r0
   │
   ├── Chuẩn hóa độ lệch tâm:
   │     wall_steering_error = (L0_norm - R0_norm)
   │
   └── Motion_Update():
         correction = PID_Compute(&pid_wall, wall_steering_error)
         pwm_left  = speed + correction
         pwm_right = speed - correction
```

### 7.2. Luồng Dữ Liệu Điều Khiển Chuyển Động (Motion Control Pipeline)

```text
MPU6050 Gyro Z (I2C1 DMA) ────┐
                              ├──► Dynamic Complementary Filter ──► Heading (Yaw)
Encoder L/R (TIM3/TIM4) ──────┘           │                               │
                                          │                               ▼
                                          │                     pid_heading / pid_angle
                                          │                               │
                                          ▼                               ▼
                                Traveled Distance ─────────────► Trajectory Planner
                                                                (ComputeRampedSpeed)
                                                                          │
                                                                          ▼
                                                                Motor PWM Output (TIM2)
```

---

## 8. State Machines (Máy Trạng Thái)

### 8.1. State Machine Quét IR (Trong `ir_sensor.c`)
- `IR_STATE_IDLE`: Chờ lệnh bắt đầu quét.
- `IR_STATE_AMBIENT`: Tắt hết LED, đọc mức ánh sáng nền môi trường.
- `IR_STATE_PAIR1_ON`: Bật `L90` + `R45`.
- `IR_STATE_PAIR1_READ`: Đọc ADC, trừ ambient, tắt LED.
- `IR_STATE_PAIR2_ON`: Bật `L45` + `R90`.
- `IR_STATE_PAIR2_READ`: Đọc ADC, trừ ambient, tắt LED.
- `IR_STATE_PAIR3_ON`: Bật `L0` + `R0`.
- `IR_STATE_PAIR3_READ`: Đọc ADC, trừ ambient, tắt LED $\rightarrow$ `IR_STATE_READY`.

### 8.2. State Machine Điều Khiển Chuyển Động (Trong `motion_controller.c`)
- `MOTION_IDLE`: Dừng động cơ, chờ lệnh mới.
- `MOTION_STRAIGHT`: Đi thẳng khoảng cách xác định kèm giảm tốc hình thang khi gần tới đích.
- `MOTION_STRAIGHT_CONSTANT`: Đi thẳng không giảm tốc (cho chạy chuyển tiếp liên tục giữa các ô).
- `MOTION_TURN_LEFT` / `MOTION_TURN_RIGHT` / `MOTION_TURN_180`: Quay tại chỗ dùng PD góc.
- `MOTION_TURN_STABILIZING`: Dừng động cơ chủ động (passive braking), đợi vận tốc góc $\omega_z < 5^\circ/\text{s}$ hoặc timeout 500ms để kết thúc quay.
- `MOTION_FRONT_ALIGN`: Căn lề vuông góc với tường phía trước qua $F_{\text{sum}}$ và $F_{\text{diff}}$.
- `MOTION_T1WT_APPROACH` $\rightarrow$ `MOTION_T1WT_PIVOT` $\rightarrow$ `MOTION_T1WT_EXIT`: Rẽ bo cua mượt 1 bánh (One-wheel turn chain).

### 8.3. State Machine Khám Phá Mê Cung (Trong `main.c` dòng 770-1660)
- `EXPLORE_READ_SENSORS`: Dừng tại tâm ô cờ, đọc và lọc cảm biến IR để chốt có/không có tường.
- `EXPLORE_DECIDE`: Cập nhật tường vào `maze`, chạy `Maze_FloodFillCenter()`, tính toán bước đi tiếp theo.
- `EXPLORE_MOVE`: Thực hiện lệnh chuyển động tương ứng ('F', 'L', 'R', 'B').
- `EXPLORE_BACK_ALIGN`: Kích hoạt căn lùi tường đuôi nếu điều kiện vị trí hợp lệ.
- `EXPLORE_GOAL_REACHED`: Đạt ô đích $(4, 4)$, dừng xe, đồng bộ dữ liệu vào `persistent_map`, lưu vào Flash.

---

## 9. Phân Tích Cấu Hình & Build System

### File `platformio.ini`
```ini
[env:genericSTM32F411CE]
platform = ststm32
board = genericSTM32F411CE
framework = cmsis
upload_protocol = stlink
upload_flags =
	-c
	adapter speed 100
	-c
	reset_config none
```

- **Framework**: Thuần `cmsis` (bare-metal trực tiếp thanh ghi), hoàn toàn **KHÔNG DÙNG STM32 HAL / LL / Arduino**. Mã nguồn can thiệp trực tiếp vào con trỏ cấu trúc thanh ghi CMSIS (`RCC`, `GPIOA`, `GPIOB`, `TIM2`, `TIM5`, `USART1`, v.v.).
- **Tốc độ nạp (adapter speed)**: 100 kHz (được hạ thấp để đảm bảo nạp ổn định qua dây ST-Link khi phần cứng bị nhiễu do động cơ).
- **reset_config none**: Tránh reset chân cứng NRST khi mạch nạp không nối chân NRST.

---

## 10. Dependencies (Thư Viện & Phụ Thuộc)

1. **CMSIS Core Cortex-M4**: Định nghĩa thanh ghi và ngoại vi chuẩn của ARM Cortex-M4 (`stm32f4xx.h`, `core_cm4.h`).
2. **CMSIS-DSP (`arm_math.h`)**: Sử dụng thư viện toán học DSP của ARM trong `main.c` (phần active) cho hàm biến đổi Fourier nhanh `arm_rfft_fast_f32`, `arm_rfft_fast_init_f32`, `arm_sqrt_f32`.
3. **C Standard Libraries**: `<stdint.h>`, `<stdbool.h>`, `<stdio.h>`, `<string.h>`, `<math.h>`, `<stdarg.h>`, `<stdlib.h>`.
4. **Không có phụ thuộc bên thứ ba (External Dependencies)** ngoài các thư viện chuẩn CMSIS của ARM/ST.

---

## 11. Các Điểm Quan Trọng Cần Lưu Ý (Notes & Gotchas)

> [!WARNING]
> **1. TÌNH TRẠNG FILE `src/main.c`**:
> Toàn bộ logic chính của Micromouse (Menu OLED, Exploration State Machine, A* Run, Hiệu chuẩn) từ dòng 1 đến dòng 2865 đang bị **COMMENT HÓA BẰNG `//`**.
> Hiện tại chỉ có đoạn code test FFT cảm biến IR (dòng 2868 - 3237) là code hoạt động. Claude khi làm việc cần lưu ý điểm này nếu được yêu cầu khôi phục ứng dụng Micromouse hoặc sửa lỗi giải mê cung.

> [!NOTE]
> **2. CHUYỂN HƯỚNG UART SANG BLUETOOTH (UART REDIRECTION)**:
> Trong `src/uart.c`, các hàm `UART_SendString()`, `UART_SendChar()`, `UART_SendNumber()` không truyền ra chân UART1 (`PA9/PA10`) mà được redirect sang `BT_SendString()` trên UART2 (`PA2/PA3`) nối với JDY-33 BLE. Muốn xuất ra UART có dây, phải dùng hàm `UART_SendString_DMA()`.

> [!IMPORTANT]
> **3. LED TRẠNG THÁI `PC13` LÀ ACTIVE LOW**:
> Lệnh `LED_ON()` tương ứng với `GPIO_RESET_PIN(GPIOC, 13)` (kéo xuống 0V). Lệnh `LED_OFF()` tương ứng với `GPIO_SET_PIN(GPIOC, 13)` (kéo lên 3.3V).

> [!NOTE]
> **4. VÔ HIỆU HÓA JTAG PINS**:
> Các chân `PA15` (JTDI), `PB3` (JTDO), `PB4` (JNTRST) là chân debug JTAG mặc định sau khi boot. Hàm `Hardware_DisableJTAG()` can thiệp ghi đè `MODER` và `BSRR` để biến chúng thành chân Timer IO cho Motor PWM và Encoder. Không được xóa hàm này.

> [!NOTE]
> **5. HÀM ĐO PIN `Hardware_GetBatteryVoltage()`**:
> Đang trả về giá trị hằng số cứng `8.2f`. Chưa có mạch chia áp hoặc kênh ADC thực tế nào nối vào để đo điện áp pin thật.

> [!NOTE]
> **6. KÍCH THƯỚC MÊ CUNG**:
> Cấu hình mê cung hiện tại là **5x5** (`MAZE_SIZE = 5`), đích tại ô $(4, 4)$. Thuật toán Flood-Fill và bộ nhớ Flash đang được tối ưu cho kích thước này.

> [!IMPORTANT]
> **7. NGẮT THỜI GIAN THỰC CỨNG TIM11 (PRIORITY 0)**:
> Ngắt `TIM1_TRG_COM_TIM11_IRQHandler` có mức ưu tiên cao nhất trong hệ thống. Trong ngắt này có gọi `I2C_ReadReg_DMA()`. Do đó, bất kỳ thao tác I2C nào ở `main loop` (như cập nhật OLED) **bắt buộc phải non-blocking hoặc có cờ kiểm tra bus busy**, nếu không ngắt TIM11 sẽ chiếm quyền CPU và gây ra **deadlock I2C**.

> [!NOTE]
> **8. FLASH CO-EXISTENCE TRONG SECTOR 6**:
> Sector 6 (128KB) dùng chung cho cả thông số hiệu chuẩn IR (offset 0) và MPU gyro offset (offset 256). Thao tác ghi một trong hai cấu trúc đều phải đọc backup phần tử còn lại, xóa Sector 6, rồi ghi lại cả hai để tránh mất dữ liệu.

---

## 12. Sơ Đồ Quan Hệ Phụ Thuộc Module (Dependency Tree)

```text
main (src/main.c)
 ├── hardware (hardware.c / hardware.h)
 │    ├── pinout.h
 │    ├── tb6612fng (tb6612fng.c / tb6612fng.h)
 │    │    └── timer (timer.c / timer.h)
 │    ├── encoder (encoder.c / encoder.h)
 │    └── system_timer (system_timer.c / system_timer.h)
 ├── uart (uart.c / uart.h)
 │    └── bt_debug (bt_debug.c / bt_debug.h)
 ├── i2c (i2c.c / i2c.h)
 ├── ssd1306 (ssd1306.c / ssd1306.h)
 │    ├── i2c
 │    └── fonts (fonts.c / fonts.h)
 ├── ir_sensor (ir_sensor.c / ir_sensor.h)
 │    └── adc / dma / TIM10
 ├── mpu6050 (mpu6050.c / mpu6050.h)
 │    └── i2c (DMA RX Stream 0)
 ├── ir_simple_calib (ir_simple_calib.c / ir_simple_calib.h)
 │    └── ir_sensor
 ├── sensor_fusion (sensor_fusion.c / sensor_fusion.h)
 │    ├── mpu6050
 │    ├── encoder
 │    └── ir_simple_calib
 ├── motion_controller (motion_controller.c / motion_controller.h)
 │    ├── sensor_fusion
 │    ├── tb6612fng
 │    ├── pid (pid.c / pid.h)
 │    └── TIM11 (1kHz ISR loop)
 ├── back_align (back_align.c / back_align.h)
 │    └── motion_controller
 ├── maze_solver (maze_solver.c / maze_solver.h)
 │    └── uart / bt_debug (Telemetry packets)
 ├── flash_storage (flash_storage.c / flash_storage.h)
 │    ├── ir_simple_calib
 │    └── maze_solver
 └── systemTest (systemTest.c / systemTest.h)
      ├── motion_controller
      ├── sensor_fusion
      └── ssd1306
```

---
*Tài liệu được sinh tự động bởi quá trình rà soát và phân tích tĩnh toàn bộ mã nguồn của dự án Nezumi Chan Micromouse Firmware.*
