# TỔNG QUAN TOÀN BỘ CODEBASE FIRMWARE MICROMOUSE STM32F411CEU6
**Branch:** `giomuadongbac` (Gió Mùa Đông Bắc - Minimal Core Baseline)  
**Ngày lập:** Tháng 10/2026  
**Nền tảng:** STM32F411CEU6 ("Black Pill"), CMSIS (thao tác trực tiếp thanh ghi), PlatformIO.

---

## 1. GIỚI THIỆU & MỤC TIÊU CỦA BRANCH `giomuadongbac`

Branch `giomuadongbac` là phiên bản **tái cấu trúc tối giản (Minimal Core)** được chuẩn hóa từ mã nguồn gốc của robot Micromouse.
- **Giữ nguyên 100% sơ đồ phần cứng và pinout PCB** (định nghĩa cố định trong `pinout.h`).
- **Mục tiêu kiến trúc**: Tối giản hóa chu trình thời gian thực (real-time path), phân định rõ ràng quyền sở hữu ngoại vi (Peripheral Ownership), mã nguồn trực quan, tường minh ở mức thanh ghi vi điều khiển (Direct Register Access qua CMSIS), không phụ thuộc vào thư viện HAL cồng kềnh.
- **Phạm vi kiểm thử hiện tại**: Tập trung hoàn thiện và ổn định hệ thống điều khiển chuyển động cơ bản (Đi thẳng 180mm, Quay trái 90 độ, Quay phải 90 độ, Phanh khẩn cấp, Reset góc Yaw) và luồng giám sát không dây qua Bluetooth JDY-33 / Web Console trước khi tích hợp lại các module nâng cao (giải mê cung Flood-fill, màn hình OLED, căn tường tự động, bộ nhớ Flash/EEPROM).

---

## 2. BẢNG TỔNG HỢP DANH MỤC CÁC TỆP TIN TRONG DỰ ÁN

| Đường dẫn tệp | Loại tệp | Kích thước | Chức năng chính |
| :--- | :--- | :--- | :--- |
| **`platformio.ini`** | Cấu hình | ~560 B | Cấu hình PlatformIO cho STM32F411CEU6, CMSIS framework, ST-Link uploader. |
| **`README.md`** | Tài liệu | ~2.7 KB | Tài liệu tổng quan nhánh, phân quyền ngoại vi, quy trình nạp và bring-up robot. |
| **`docs/architecture.md`** | Tài liệu | ~2.7 KB | Bản đặc tả kiến trúc: 8 nguyên tắc sở hữu ngoại vi, luồng thực thi ngắt thời gian thực. |
| **`include/pinout.h`** | Header | ~7.8 KB | Bảng đấu nối chân phần cứng PCB: Động cơ, Encoder, IR Sensors, I2C, UART, LED, Nút bấm. |
| **`include/board.h`**<br>**`src/board.c`** | C/Header | ~3.4 KB | Quản lý xung nhịp hệ thống (System/APB Clocks), GPIO bo mạch (LED, nút bấm), xử lý ngắt HardFault an toàn. |
| **`include/system_timer.h`**<br>**`src/system_timer.c`** | C/Header | ~7.2 KB | Timer đếm thời gian hệ thống độ phân giải 1µs không chặn (non-blocking) dựa trên TIM5 (32-bit). |
| **`include/motor.h`**<br>**`src/motor.c`** | C/Header | ~4.1 KB | Điều khiển PWM 20kHz 2 kênh động cơ DC (TIM2 CH1/CH2) và logic cầu H đảo chiều trên GPIOB. |
| **`include/encoder.h`**<br>**`src/encoder.c`** | C/Header | ~3.9 KB | Đọc encoder 2 kênh N20 bằng chế độ Quadrature Encoder phần cứng (TIM3, TIM4), tính vận tốc LPF. |
| **`include/i2c.h`**<br>**`src/i2c.c`** | C/Header | ~26 KB | Driver bus I2C1 (PB8/PB9) hỗ trợ chế độ chặn, DMA TX (Stream 6), DMA RX (Stream 0) và bus recovery. |
| **`include/mpu6050.h`**<br>**`src/mpu6050.c`** | C/Header | ~19 KB | Driver IMU MPU-6050: Hiệu chuẩn trôi tĩnh (calibration), đọc Gyro Z không chặn qua DMA, lọc số (LPF, Deadzone), tích phân Yaw. |
| **`include/ir_sensor.h`**<br>**`src/ir_sensor.c`** | C/Header | ~8.0 KB | Điều khiển quét 6 mắt hồng ngoại (ADC1 + DMA2 + TIM10 100µs), thuật toán khử nhiễu quang môi trường (Ambient Rejection). |
| **`include/pid.h`**<br>**`src/pid.c`** | C/Header | ~1.8 KB | Thuật toán PID toán học thuần túy: P, I, D, bộ kẹp chống bão hòa tích phân (Anti-windup), độc lập phần cứng. |
| **`include/motion.h`**<br>**`src/motion.c`** | C/Header | ~7.1 KB | Máy trạng thái chuyển động (`IDLE`, `STRAIGHT`, `TURN`), ramping PWM, bám hướng PID và dừng chính xác. |
| **`include/control.h`**<br>**`src/control.c`** | C/Header | ~3.1 KB | Bộ lập lịch thời gian thực 1 kHz (TIM11 ISR), thu thập đồng bộ cảm biến và kích hoạt vòng điều khiển chuyển động. |
| **`include/bt_debug.h`**<br>**`src/bt_debug.c`** | C/Header | ~9.9 KB | Driver giao tiếp Bluetooth JDY-33 qua USART2: hàng đợi vòng (ring buffer 4KB), ngắt TXE không gây trễ CPU, hàm `BT_Printf`. |
| **`src/main.c`** | Source | ~3.4 KB | Khởi động toàn hệ thống, tiếp nhận mã lệnh Bluetooth và phát telemetry 10 Hz ra Web Console. |
| **`tools/ble_debug_console.html`** | Web App | ~55 KB | Web Console chạy trên trình duyệt hỗ trợ Web Bluetooth kết nối JDY-33, vẽ đồ thị real-time và bản đồ mê cung. |
| **`tools/README.md`** | Tài liệu | ~1.0 KB | Hướng dẫn kết nối Web Bluetooth Console với robot. |

---

## 3. CHI TIẾT TỪNG MODULE & TÁC DỤNG CỤ THỂ

### 3.1. Cấu hình chân & Phần cứng: `include/pinout.h`
- **Tác dụng**: Là bản thiết kế dây cứng (Hardware Pinout Contract) duy nhất của toàn bộ robot. Các driver khác bắt buộc phải tham chiếu chân từ file này.
- **Thông số kỹ thuật định nghĩa**:
  - **Động cơ**:
    - Motor Right (A): PWM trên `PA15` (TIM2_CH1), IN1 trên `PB15`, IN2 trên `PB14`.
    - Motor Left (B): PWM trên `PB3` (TIM2_CH2), IN1 trên `PB13`, IN2 trên `PB12`.
    - Tần số PWM: 20 kHz (ARR = 4999).
  - **Encoder (N20 GA12)**:
    - Right: `PB4` (TIM3_CH1), `PB5` (TIM3_CH2).
    - Left: `PB6` (TIM4_CH1), `PB7` (TIM4_CH2).
    - Độ phân giải: 1430 xung/vòng (PPR sau nhân 4).
  - **Thông số cơ khí robot**:
    - Đường kính bánh xe: `35.0 mm`.
    - Chiều rộng cơ sở (Wheel Base): `84.0 mm`.
  - **Cảm biến khoảng cách hồng ngoại (6 cặp)**:
    - 6 kênh thu quang (ADC1): `PB1` (L90, CH9), `PB0` (L45, CH8), `PA6` (L0, CH6), `PA5` (R0, CH5), `PA4` (R45, CH4), `PA1` (R90, CH1).
    - 6 kênh phát hồng ngoại (GPIO Out): `PA7` (L90), `PB10` (L45), `PB2` (L0), `PA8` (R0), `PA11` (R45), `PA12` (R90).
  - **Giao tiếp ngoại vi**:
    - I2C1 (MPU6050 & OLED): `PB8` (SCL), `PB9` (SDA) - Địa chỉ MPU6050: `0x68`.
    - Bluetooth JDY-33 (USART2): `PA2` (TX), `PA3` (RX) - Baudrate 115200.
    - Nút bấm User: `PA0` (tích cực mức thấp).
    - LED trạng thái: `PC13` (tích cực mức thấp).

---

### 3.2. Quản lý bo mạch & Xung nhịp: `board.h` / `board.c`
- **Tác dụng**: Cung cấp các tiện ích phần cứng chung của bo mạch STM32F411, tự động tính toán tần số xung nhịp bus mà không cố định một giá trị clock giả định.
- **Các hàm chủ chốt**:
  - `Board_Init()`: Bật xung clock cho GPIOA, GPIOB, GPIOC, SYSCFG; khởi tạo chân LED PC13 và nút nhấn PA0.
  - `Board_GetSystemClockHz()`, `Board_GetAPB1ClockHz()`, `Board_GetAPB2ClockHz()`: Đọc các trường thanh ghi `RCC->CFGR` và `RCC->PLLCFGR` để tính toán chính xác tần số xung clock hiện tại (HSI 16MHz, HSE 25MHz hoặc PLL).
  - `Board_GetAPB1TimerClockHz()`, `Board_GetAPB2TimerClockHz()`: Tính toán tần số clock cấp cho các khối Timer (nhân 2 nếu prescaler APB khác 1 theo đặc tả STM32).
  - `HardFault_Handler()`: Bẫy lỗi phần cứng nghiêm trọng. Ngay lập tức ngắt toàn bộ xung PWM motor (TIM2 CCR1/CCR2 = 0) và dừng timer điều khiển (TIM11, TIM10) nhằm bảo vệ phần cứng tránh trường hợp robot lao tự do khi gặp sự cố crash phần mềm.

---

### 3.3. Bộ đếm thời gian hệ thống: `system_timer.h` / `system_timer.c`
- **Tác dụng**: Cung cấp hàm đo thời gian không chặn độ chính xác micro-giây cho hệ thống.
- **Cơ chế hoạt động**:
  - Độc quyền sở hữu **TIM5** (Timer 32-bit của STM32F411).
  - Cấu hình Prescaler tự động từ xung nhịp APB1 Timer để TIM5 đếm đúng 1 MHz (1 tick = 1 µs).
  - Thanh ghi `ARR = 0xFFFFFFFF` cho phép đếm liên tục khoảng 71.5 phút trước khi tràn số.
- **Hàm cung cấp**:
  - `micros()`: Lấy số micro-giây hiện tại (`TIM5->CNT`).
  - `millis()`: Lấy số mili-giây hiện tại (`TIM5->CNT / 1000`).
  - `delay_elapsed_us(start, delay)`, `delay_elapsed_ms(start, delay)`: Hàm inline kiểm tra thời gian trôi qua không chặn.
  - `delay_us_blocking()`, `delay_ms_blocking()`: Delay chặn (chỉ dùng trong khâu khởi tạo ngoại vi).

---

### 3.4. Điều khiển động cơ: `motor.h` / `motor.c`
- **Tác dụng**: Điều chế xung PWM và logic cầu H điều khiển 2 động cơ DC độc lập.
- **Cơ chế hoạt động**:
  - Độc quyền sở hữu **TIM2**: Cấu hình chế độ PWM Mode 1 trên CH1 (PA15 - Motor Phải) và CH2 (PB3 - Motor Trái). Tần số 20 kHz nằm ngoài dải nghe thấy của tai người (chống rít động cơ).
  - Đảo chiều bằng 4 chân logic (IN1, IN2) trên cùng port **GPIOB** (`PB12`, `PB13`, `PB14`, `PB15`).
  - Sử dụng thanh ghi `GPIOB->BSRR` để cập nhật trạng thái cả 4 chân đảo chiều **cùng một thời điểm (atomic)** trong một lệnh ghi thanh ghi duy nhất, loại bỏ hiện tượng lệch pha hoặc ngắn mạch cầu H.
  - Sử dụng thanh ghi Shadow/Preload của TIM2 (`TIM_CCMR1_OCxPE`) giúp giá trị PWM của cả 2 kênh cùng có hiệu lực tại sườn chu kỳ kế tiếp.
- **Hàm cung cấp**:
  - `Motor_Init()`: Khởi tạo GPIO và TIM2 PWM 20kHz.
  - `Motor_SetPair(left_pwm, right_pwm)`: Nhận dải giá trị -100% đến +100%. Tự động đảo chiều và đặt độ rộng xung tương ứng.
  - `Motor_Stop()`: Cho xe thả trôi (Coasting - PWM = 0, IN1 = IN2 = 0).
  - `Motor_Brake()`: Phanh hãm động năng (Braking - IN1 = IN2 = 1).

---

### 3.5. Thu thập tín hiệu Encoder: `encoder.h` / `encoder.c`
- **Tác dụng**: Đọc phản hồi vị trí và vận tốc bánh xe từ cảm biến từ Encoder pha A/B.
- **Cơ chế hoạt động**:
  - Độc quyền sở hữu **TIM3** (Motor Phải - PB4/PB5) và **TIM4** (Motor Trái - PB6/PB7).
  - Cấu hình chế độ giải mã phần cứng `Encoder Mode 3` (đếm trên cả 2 sườn của cả 2 kênh TI1 và TI2 -> nhân 4 độ phân giải).
  - `Encoder_Update(dt_s)`: Được gọi định kỳ đúng 1 kHz trong ngắt TIM11. Đọc hiệu số xung delta, bù trừ tràn số 16-bit, cộng dồn vào `total_left`, `total_right`.
  - Tính toán vận tốc tức thời xung/giây (`speed_pps`) và lọc qua bộ lọc thông thấp IIR số bậc nhất (`alpha = 0.30`).
- **Nguyên tắc kiến trúc**: `Encoder_Update()` là hàm duy nhất có quyền ghi dữ liệu đếm xung. Các hàm `Encoder_GetLeftCount()`, `Encoder_GetRightCount()`, `Encoder_Get...SpeedPPS()` là các hàm đọc thuần túy (read-only).

---

### 3.6. Giao tiếp I2C & DMA: `i2c.h` / `i2c.c`
- **Tác dụng**: Giao tiếp tốc độ cao với MPU6050 (và chuẩn bị cho OLED) trên bus I2C1 (PB8/PB9).
- **Cơ chế hoạt động**:
  - Hỗ trợ chế độ chuẩn 100 kHz và Fast Mode 400 kHz.
  - **Tách biệt cơ chế DMA**:
    - DMA1 Stream 6 cho I2C1 TX (phục vụ truyền dữ liệu OLED).
    - DMA1 Stream 0 cho I2C1 RX (phục vụ đọc nhanh dữ liệu IMU MPU6050).
  - Cơ chế **I2C Bus Recovery**: Nếu phát hiện bus bị treo (đường SDA bị slave giữ mức thấp), hàm tự động chuyển SCL sang GPIO đẩy 9 xung clock để giải phóng đường truyền.
  - Cung cấp cơ chế Callback không đồng bộ ngắt khi DMA hoàn thành.

---

### 3.7. Cảm biến quán tính IMU: `mpu6050.h` / `mpu6050.c`
- **Tác dụng**: Cung cấp góc hướng Yaw và vận tốc góc xoay quanh trục Z (`Gyro Z`) để giữ thăng bằng hướng thẳng và điều khiển góc quay chính xác.
- **Cơ chế hoạt động**:
  - Cấu hình dải đo Gyro: `±500 dps` (hệ số tỉ lệ `65.5 LSB / (deg/s)`).
  - Quy trình khởi tạo: Kiểm tra định danh `WHO_AM_I` (0x68), cài đặt tần số lấy mẫu và tắt bộ lọc phần cứng quá trễ.
  - **Hiệu chuẩn tĩnh (`MPU6050_Calibrate`)**: Lúc khởi động, đọc 200 mẫu khi xe đứng yên để tính giá trị trôi tĩnh `gyro_z_offset`.
  - **Cập nhật trong thời gian thực (`MPU6050_Update`)**:
    - Chạy trong ngắt 1 kHz. Gửi yêu cầu đọc thanh ghi Gyro Z qua I2C DMA không chặn.
    - Khi nhận xong dữ liệu: Trừ offset tĩnh -> Lọc thông thấp số (`GYRO_Z_LPF_ALPHA = 0.7`) -> Lọc vùng chết (`GYRO_Z_DEADZONE = 0.3 deg/s` triệt tiêu trôi góc tích lũy khi đứng yên) -> Tích phân theo `dt` để tính góc Yaw (`yaw += gz_final * dt`).
    - Có cơ chế kiểm tra chống deadlock: Nếu bus I2C đang bận, bỏ qua chu kỳ hiện tại thay vì đợi gây trễ ngắt cứng.

---

### 3.8. Cảm biến khoảng cách hồng ngoại: `ir_sensor.h` / `ir_sensor.c`
- **Tác dụng**: Đo khoảng cách tới thành tường của 6 mắt hồng ngoại phân bố xung quanh robot (Trái 90°, Trái 45°, Trái thẳng 0°, Phải thẳng 0°, Phải 45°, Phải 90°).
- **Cơ chế hoạt động độc đáo (Khử nhiễu môi trường - Ambient Rejection)**:
  - Sử dụng **ADC1** kết hợp **DMA2 Stream 0** và được điều phối tuần tự bởi ngắt **TIM10** (chu kỳ 100 µs mỗi bước).
  - Máy trạng thái ngắt TIM10 gồm các giai đoạn:
    1. `IR_AMBIENT`: Tắt toàn bộ 6 đèn LED phát hồng ngoại. Kích hoạt ADC quét cả 6 kênh lưu giá trị quang môi trường vào `ambient[]`.
    2. `IR_PAIR1_ON` & `READ`: Bật cặp LED phát L90 và R45. Chờ 100µs cho ánh sáng ổn định, kích hoạt ADC đọc và tính `result = ADC - ambient`. Tắt LED.
    3. `IR_PAIR2_ON` & `READ`: Bật cặp LED phát L45 và R90. Đọc ADC, trừ môi trường, tắt LED.
    4. `IR_PAIR3_ON` & `READ`: Bật cặp LED phát L0 và R0 (phía trước). Đọc ADC, trừ môi trường, tắt LED.
    5. Chuyển trạng thái sang `IR_READY` và dừng TIM10.
  - Việc bật so le từng cặp đối xứng (L90 đi với R45, L45 đi với R90,...) giúp **triệt tiêu hiện tượng phát quang chéo (optical cross-talk)** giữa các mắt cảm biến liền kề.
  - Trong ngắt 1 kHz của `control.c`, sau khi đọc kết quả xong sẽ tự động gọi `IR_StartScan()` để kích hoạt chu kỳ quét mới.

---

### 3.9. Giải thuật PID toán học: `pid.h` / `pid.c`
- **Tác dụng**: Cung cấp cấu trúc và hàm tính toán bộ điều khiển PID tổng quát, hoàn toàn không phụ thuộc vào bất kỳ thanh ghi phần cứng nào.
- **Cấu trúc & Thuật toán**:
  - `PID_Handle_t`: Chứa các hệ số `Kp`, `Ki`, `Kd`, biến lưu sai số cũ `prev_error`, khâu tích phân `integral` và các ngưỡng giới hạn đầu ra (`limit_output`), ngưỡng chống bão hòa (`limit_integral`).
  - Hàm `PID_Compute(pid, setpoint, input, dt)`:
    - Tính sai số: $e = setpoint - input$.
    - Khâu tỉ lệ: $P = K_p \cdot e$.
    - Khâu tích phân (có Anti-windup clamping): $I = K_i \cdot \int e \cdot dt$.
    - Khâu vi phân: $D = K_d \cdot \frac{e - e_{prev}}{dt}$.
    - Tổng hợp: $Output = \text{clamp}(P + I + D, -\text{limit}, +\text{limit})$.

---

### 3.10. Điều khiển chuyển động: `motion.h` / `motion.c`
- **Tác dụng**: Đóng vai trò "bộ não điều hành hành vi cơ học", quyết định robot cần làm gì và điều khiển động cơ thực thi lệnh.
- **Các trạng thái (`Motion_State_t`)**:
  - `MOTION_IDLE`: Robot đứng yên, động cơ dừng.
  - `MOTION_STRAIGHT`: Đi thẳng một quãng đường định trước (ví dụ 180 mm = 1 ô mê cung chuẩn).
    - Sử dụng bộ ramping gia tốc tăng dần PWM (`STRAIGHT_RAMP_PWM_S = 120.0f/s`) từ 15% lên 35%.
    - Đo quãng đường bằng trung bình cộng 2 encoder: $d = \frac{Left_{mm} + Right_{mm}}{2}$.
    - Dùng bộ PID góc hướng (`heading_pid`) giữ robot luôn chạy theo góc thẳng ban đầu (`start_yaw_deg`).
    - Khi quãng đường đạt mục tiêu, chuyển trạng thái về `MOTION_IDLE` và dừng động cơ.
  - `MOTION_TURN`: Quay tại chỗ một góc định trước (quay trái +90° hoặc quay phải -90°).
    - Dùng bộ PID quay (`turn_pid`) tính toán chênh lệch giữa góc đã quay và góc mục tiêu.
    - Điều khiển 2 bánh quay ngược chiều nhau (Differential steering in-place).
    - Điều kiện hoàn thành: Sai số góc $\le 3^\circ$, vận tốc góc $\le 10^\circ/s$ hoặc quá thời gian Timeout 1.5 giây.
- **Tính nguyên tử (Atomic Commands)**: Các hàm nhận lệnh như `Motion_CommandStraight()`, `Motion_CommandTurn()`, `Motion_Stop()` đều vô hiệu hóa ngắt tạm thời (`__disable_irq()`) trong lúc gán biến để tránh xung đột dữ liệu với ngắt TIM11 1 kHz.

---

### 3.11. Bộ lập lịch thời gian thực 1 kHz: `control.h` / `control.c`
- **Tác dụng**: Đóng vai trò nhịp tim thời gian thực cứng (Hard Real-Time Clock) của hệ thống.
- **Cơ chế hoạt động**:
  - Sở hữu độc quyền **TIM11** với tần số ngắt chính xác **1000 Hz** (1 ms một lần).
  - Mức ưu tiên ngắt NVIC = 0 (mức cao nhất trong hệ thống).
  - **Chuỗi thực thi trong hàm ngắt `TIM1_TRG_COM_TIM11_IRQHandler()`**:
    ```text
    1. Encoder_Update(0.001s)       --> Cập nhật xung và vận tốc bánh xe
    2. MPU6050_Update()             --> Cập nhật dữ liệu góc Yaw từ IMU
    3. Thu nhận dữ liệu IR & gọi IR_StartScan() --> Chu kỳ cảm biến khoảng cách
    4. Motion_Update()              --> Tính toán điều khiển, PID và xuất xung Motor
    ```
- **Quy tắc nghiêm ngặt**: Tuyệt đối **không** thực hiện in chuỗi debug, format chuỗi hay xử lý UART/Bluetooth bên trong ngắt này để đảm bảo chu kỳ thực thi luôn kết thúc trong thời gian vài chục micro-giây.

---

### 3.12. Giao tiếp Bluetooth JDY-33: `bt_debug.h` / `bt_debug.c`
- **Tác dụng**: Truyền nhận dữ liệu không dây với máy tính/điện thoại qua cổng USART2 (PA2 TX, PA3 RX, Baudrate 115200).
- **Cơ chế hoạt động**:
  - Không dùng DMA do kênh DMA của USART2 trùng tài nguyên với I2C trên STM32F411.
  - Sử dụng hàng đợi vòng **Circular Buffer 4096 bytes** kết hợp ngắt truyền trống thanh ghi `TXE` (`USART2_IRQHandler`).
  - Khi người dùng gọi `BT_SendString()` hoặc `BT_Printf()`, dữ liệu được đẩy vào buffer và bật cờ ngắt `TXEIE`. CPU lập tức quay lại làm việc khác mà không bị chặn (Non-blocking).
  - Chính sách **Drop-on-full**: Nếu hàng đợi bị đầy do tốc độ truyền không kịp, các byte vượt mức sẽ bị hủy bỏ chứ không làm đứng chương trình.
  - Cung cấp `BT_Available()` và `BT_ReceiveChar()` để đọc lệnh từ giao diện điều khiển.

---

### 3.13. Chương trình chính: `src/main.c`
- **Tác dụng**: Nhạc trưởng điều phối toàn bộ chương trình (System Orchestrator).
- **Quy trình hoạt động**:
  1. Khởi tạo tuần tự phần cứng: `Board_Init()`, `SystemTimer_Init()`, `Motor_Init()`, `Encoder_Init()`, `I2C_Init()`, `BT_Init()`, `IR_Init()`, `MPU6050_Init()`.
  2. Hiệu chuẩn cảm biến MPU-6050 (`MPU6050_Calibrate`). Nếu lỗi, dừng robot, bật LED báo lỗi và treo an toàn.
  3. Bật chu trình quét IR (`IR_StartScan()`) và khởi động timer ngắt điều khiển 1 kHz (`Control_Init()`, `Control_Start()`).
  4. Vòng lặp `while(1)` nền:
     - **Nhận lệnh Bluetooth**: Đọc ký tự điều khiển qua `BT_ReceiveChar()` và gọi `HandleBleCommand()`:
       - `F` / `f`: Đi thẳng 180 mm (nếu robot đang rảnh).
       - `L` / `l`: Quay trái 90 độ.
       - `R` / `r`: Quay phải 90 độ.
       - `S` / `s`: Dừng khẩn cấp.
       - `Z` / `z`: Reset góc Yaw về 0.
     - **Gửi Telemetry định kỳ 10 Hz (100 ms)**: Tạo một bản chụp nhanh an toàn trạng thái robot (Encoder, Yaw, Gyro, Distance, State) và dùng `BT_Printf()` gửi gói tin định dạng chuỗi ra Web Console:
       ```text
       t=%lu state=%u encL=%ld encR=%ld yaw=%.2f gz=%.2f d=%.1f e=%.2f\r\n
       ```

---

### 3.14. Ứng dụng Web Console: `tools/ble_debug_console.html` & `tools/README.md`
- **Tác dụng**: Giao diện người dùng đồ họa hoàn chỉnh chạy trên nền Web (Chrome / Edge), giao tiếp trực tiếp với robot qua Web Bluetooth API (Service `0xFFE0` của module JDY-33).
- **Tính năng**:
  - Kết nối / Ngắt kết nối Bluetooth chỉ với 1 click.
  - Hiển thị bảng điều khiển nút bấm: Forward 180mm, Turn Left 90°, Turn Right 90°, Stop, Zero Yaw.
  - Vẽ đồ thị thời gian thực đa kênh: Góc Yaw, Gyro Z, Quãng đường, Vận tốc Encoder.
  - Khung hiển thị log terminal dạng text.
  - Khung giả lập ma trận mê cung 2D (5x5 / 16x16) sẵn sàng cho tính năng giải mê cung trong tương lai.

---

## 4. SƠ ĐỒ LIÊN KẾT & KIẾN TRÚC HỆ THỐNG

### 4.1. Bảng phân quyền ngoại vi (Hardware Ownership Matrix)
Một trong những nguyên tắc cốt lõi của nhánh này là **mỗi ngoại vi phần cứng chỉ do đúng một module phần mềm làm chủ**:

```text
┌──────────────────────┬─────────────────────┬──────────────────────────────────────────┐
│ Ngoại vi (Hardware)  │ Module làm chủ      │ Nhiệm vụ chi tiết                        │
├──────────────────────┼─────────────────────┼──────────────────────────────────────────┤
│ TIM2 (CH1, CH2)      │ motor.c             │ Tạo xung PWM 20kHz điều khiển động cơ     │
│ TIM3                 │ encoder.c           │ Đếm xung Quadrature Encoder kênh Phải    │
│ TIM4                 │ encoder.c           │ Đếm xung Quadrature Encoder kênh Trái    │
│ TIM5 (32-bit)        │ system_timer.c      │ Đếm tick 1 µs cho micros() và millis()   │
│ TIM10                │ ir_sensor.c         │ Định thì 100 µs cho máy trạng thái quét IR│
│ TIM11                │ control.c           │ Lập lịch ngắt chu kỳ 1 kHz thời gian thực│
│ ADC1 + DMA2 Stream 0 │ ir_sensor.c         │ Đọc tuần tự điện áp 6 kênh thu quang IR  │
│ I2C1 + DMA1 Stream 0 │ i2c.c / mpu6050.c   │ Giao tiếp đọc dữ liệu Gyro MPU-6050      │
│ USART2 (PA2, PA3)    │ bt_debug.c          │ Truyền nhận UART không dây với JDY-33    │
│ GPIOA, GPIOB, GPIOC  │ board.c & pinout.h  │ Quản lý LED PC13, Nút bấm PA0, Clock     │
└──────────────────────┴─────────────────────┴──────────────────────────────────────────┘
```

---

### 4.2. Sơ đồ phân tầng kiến trúc (Architecture Layering)

```text
                     ┌────────────────────────┐
                     │   Host / Web Browser   │
                     │ (ble_debug_console.js) │
                     └───────────┬────────────┘
                                 │ Web Bluetooth API (JDY-33)
                                 ▼
                     ┌────────────────────────┐
                     │       main.c           │ ◄── Vòng lặp nền 10Hz (Telemetry & Lệnh)
                     └───────────┬────────────┘
                                 │ Điều phối & Khởi tạo
                                 ▼
                     ┌────────────────────────┐
                     │      control.c         │ ◄── Ngắt cứng TIM11 @ 1000 Hz
                     └─────┬────────────┬─────┘
       Cập nhật dữ liệu    │            │ Ra quyết định điều khiển
       cảm biến đồng bộ    │            ▼
                           │   ┌────────────────────────┐
                           │   │       motion.c         │ (Máy trạng thái chuyển động)
                           │   └─────┬────────────┬─────┘
                           │         │            │
                           ▼         ▼            ▼
                     ┌──────────┐ ┌──────┐ ┌────────────┐
                     │ SENSORS  │ │pid.c │ │  motor.c   │
                     │(Enc, MPU,│ └──────┘ └─────┬──────┘
                     │   IR)    │                │ PWM / Direction
                     └──────────┘                ▼
                                           ┌────────────┐
                                           │ Hardware:  │
                                           │ TIM2, DC   │
                                           └────────────┘
```

---

### 4.3. Luồng dữ liệu chi tiết trong ngắt thời gian thực 1 kHz (`TIM11_IRQHandler`)

Mỗi 1 mili-giây, ngắt `TIM1_TRG_COM_TIM11_IRQHandler` kích hoạt và xử lý tuần tự theo quy trình khép kín:

```text
                       [ Ngắt TIM11: 1 kHz ]
                                │
        ┌───────────────────────┼───────────────────────┐
        ▼                       ▼                       ▼
1. Encoder_Update()     2. MPU6050_Update()     3. IR_GetLatest()
   - Đọc TIM3, TIM4        - Lấy dữ liệu DMA       - Lấy kết quả quét trước
   - Tính tổng xung delta  - Trừ offset tĩnh       - Khởi động quét mới:
   - Lọc vận tốc LPF       - Lọc LPF & Deadzone      IR_StartScan()
                           - Tích phân ra Yaw
        │                       │                       │
        └───────────────────────┼───────────────────────┘
                                ▼
                       4. Motion_Update()
                          - Tính quãng đường d = 0.5 * (L + R) mm
                          - Ramping PWM gia tốc
                          - So sánh góc với start_yaw_deg
                                │
                                ▼
                         5. PID_Compute()
                            - Straight: Heading PID hiệu chỉnh lệch góc
                            - Turn: Turn PID tính công suất bẻ lái
                                │
                                ▼
                        6. Motor_SetPair()
                           - Ghi GPIOB->BSRR chọn chiều quay (Atomic)
                           - Ghi TIM2->CCR1/CCR2 cập nhật độ rộng xung
```

---

## 5. BẢNG MÃ LỆNH & GIAO THỨC TRUYỀN THÔNG BLUETOOTH

### 5.1. Các lệnh điều khiển từ Host gửi xuống Robot (1 ký tự ASCII)
| Ký tự lệnh | Hành vi của robot | Điều kiện chấp thuận |
| :---: | :--- | :--- |
| **`F` / `f`** | Chạy thẳng quãng đường **180 mm** (1 ô mê cung) | Chỉ nhận khi robot đang rảnh (`Motion_IsDone() == true`) |
| **`L` / `l`** | Quay tại chỗ sang **Trái 90°** | Chỉ nhận khi robot đang rảnh (`Motion_IsDone() == true`) |
| **`R` / `r`** | Quay tại chỗ sang **Phải 90°** | Chỉ nhận khi robot đang rảnh (`Motion_IsDone() == true`) |
| **`S` / `s`** | **Dừng khẩn cấp** lập tức (`Motion_Stop()`) | Luôn luôn có hiệu lực trong mọi trạng thái |
| **`Z` / `z`** | Đặt lại góc Yaw hiện tại về **0.00°** | Chỉ nhận khi robot đang đứng yên |

### 5.2. Định dạng gói tin Telemetry gửi lên Host (Tần số 10 Hz)
Gói tin dạng chuỗi ASCII kết thúc bằng `\r\n`:
```text
t=12450 state=1 encL=1432 encR=1430 yaw=0.12 gz=0.01 d=180.0 e=0.12
```
- **`t`**: Thời gian hệ thống tính theo mili-giây (`millis()`).
- **`state`**: Trạng thái chuyển động hiện tại (`0`: IDLE, `1`: STRAIGHT, `2`: TURN).
- **`encL`, `encR`**: Tổng xung tích lũy bánh trái và bánh phải.
- **`yaw`**: Góc hướng hiện tại (độ).
- **`gz`**: Vận tốc góc xoay quanh trục Z hiện tại (deg/s).
- **`d`**: Quãng đường robot đã di chuyển trong lệnh hiện tại (mm).
- **`e`**: Sai số góc so với mục tiêu (độ).

---

## 6. HƯỚNG DẪN BIÊN DỊCH VÀ NẠP FIRMWARE

### 6.1. Yêu cầu môi trường
- Python và PlatformIO Core (`pio`) hoặc VSCode gắn extension PlatformIO.
- Mạch nạp ST-Link V2 kết nối với chân SWD của bo STM32F411 (`SWCLK`, `SWDIO`, `GND`, `3V3`).

### 6.2. Các câu lệnh biên dịch & nạp code (PowerShell / Terminal)
```powershell
# 1. Chuyển sang đúng branch
git switch giomuadongbac

# 2. Dọn dẹp bản build cũ
python -m platformio run -t clean

# 3. Biên dịch mã nguồn
python -m platformio run

# 4. Nạp firmware vào vi điều khiển qua ST-Link
python -m platformio run -t upload
```

---

## 7. KẾ HOẠCH MỞ RỘNG (ROADMAP)
Sau khi kiểm tra ổn định phần cứng chuyển động trên branch `giomuadongbac`, các tính năng tiếp theo sẽ được tích hợp theo lộ trình:
1. **Wall Following / Steering**: Dùng 6 cảm biến khoảng cách hồng ngoại đưa vào vòng lặp PID để tự động giữ xe ở chính giữa lòng máng mê cung.
2. **Sensor Calibration UI**: Lưu thông số hiệu chuẩn cảm biến vào Flash ROM.
3. **Maze Algorithm**: Tích hợp lại thuật toán giải mê cung kinh điển **Flood-fill** (ma trận 16x16).
4. **OLED Display**: Khôi phục hiển thị menu và trạng thái qua I2C DMA.
5. **Speedrun (A*)**: Chạy tối ưu hóa đường đi với tốc độ cao và cua chéo (Diagonal turns).
