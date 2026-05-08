# Firmware Architecture

Tài liệu này mô tả cách đọc codebase nhanh, dành cho người cần sửa firmware hoặc tái sử dụng từng module.

## 1. Tư duy tổ chức

Dự án hiện giữ file `.c/.h` tương đối phẳng để PlatformIO build đơn giản, nhưng về mặt kiến trúc có 6 nhóm rõ ràng:

| Nhóm | File chính | Vai trò |
|---|---|---|
| Application | `main.c`, `systemTest.c` | Điều phối mode, menu, state machine cấp hệ thống |
| Motion | `motion_controller.c`, `pid.c`, `back_align.c` | Primitive chuyển động và ổn định pose |
| Sensing | `ir_sensor.c`, `ir_simple_calib.c`, `mpu6050.c`, `sensor_fusion.c` | Đo, lọc, hiệu chuẩn, suy ra trạng thái |
| Navigation | `maze_solver.c`, `flash_storage.c` | Học map, tìm đường, lưu dữ liệu |
| Platform | `hardware.c`, `timer.c`, `system_timer.c`, `encoder.c`, `i2c.c`, `uart.c`, `bt_debug.c`, `tb6612fng.c` | Driver và bare-metal services |
| UI | `ssd1306.c`, `fonts.c` | Giao diện OLED |

## 2. Runtime flow

## Boot

1. `Hardware_Init()`
2. `UART_Init()` và `BT_Init()`
3. `I2C_Init()` rồi `SensorFusion_Init()`
4. `SSD1306_Init_DMA()`
5. `Motion_Init()`, `Maze_Init()`
6. Nạp flash:
   - `Flash_LoadMaze()`
   - `Flash_LoadIRCalib()`
   - `Flash_LoadMPUCalib()`

## Main menu

- Một nút duy nhất điều khiển menu.
- Short press: đổi selection.
- Long press: confirm mode.

## Explore/A*

Chu trình logic thực tế:

1. lấy tường từ IR hiện tại
2. ghi vào maze model
3. flood-fill hoặc A* để ra hành động
4. gọi motion primitive tương ứng
5. cập nhật `robot_x/y/dir`
6. phát log và `CELL:` update qua BLE

## 3. State ownership

### Motion state

`Motion_Controller_t` là context trung tâm của subsystem motion:

- target distance / angle
- speed profile
- PID heading / angle / omega / speed
- wall steering state
- front alignment state
- T1WT chain state

Nó được ghi bởi cả main loop và ISR, vì vậy nhiều field là `volatile`.

### Maze state

`Maze_t` là working state cho một lần chạy:

- `walls`
- `flood`
- `visited`
- `robot_x/y/dir`

`Maze_Persistent_t` là state lâu dài giữa nhiều lần chạy:

- wall knowledge
- confidence theo cell
- visit count
- tổng số run

## 4. Các subsystem đáng chú ý

## Sensor fusion

`sensor_fusion.c` không chỉ "fusion" theo nghĩa IMU, mà còn là nơi gom:

- raw IR values
- wall centering error
- steering error
- collision heuristics
- odometry-derived motion state

Đây là cầu nối giữa sensing và control.

## Motion controller

`motion_controller.c` là file lõi thứ hai sau `main.c`.

Các primitive chính:

- `Motion_Straight`
- `Motion_StraightConstant`
- `Motion_Turn`
- `Motion_SmoothTurn`
- `Motion_FrontAlign`
- `Motion_Start_T1WT`

Tùy mode, controller có thể chạy:

- open-loop/PWM biased profile cho explore
- closed-loop speed PID cho A*

## Maze solver

`maze_solver.c` gồm 3 lớp logic:

1. model tường và flood-fill cơ bản
2. smart exploration dựa trên visit count
3. A* optimal path cho run lại

Ngoài ra file này cũng chịu trách nhiệm serialize maze ra BLE.

## 5. Các điểm chỉnh sửa phổ biến

### Đổi phần cứng

Sửa theo thứ tự:

1. `include/pinout.h`
2. `hardware.c`
3. `motion_controller.c` nếu chiều motor/encoder đổi

### Đổi kích thước maze

Hiện tại maze là `5x5`.

Các điểm phải xem lại:

- `include/maze_solver.h`
- `tools/ble_debug_console.html`
- chiến lược gửi full maze qua BLE nếu kích thước tăng lớn

### Tinh chỉnh điều khiển

File nên xem:

- `motion_controller.c`
- `sensor_fusion.c`
- `ir_simple_calib.c`
- `main.c` cho timeout/safety logic ở tầng orchestration

## 6. Nợ kỹ thuật đang tồn tại

- `main.c` rất lớn, đang chứa cả UI, explore state machine, A* orchestration và calibration flow
- `systemTest.c` cũng là file monolith theo cùng kiểu
- một số tên `center` trong solver mang tính lịch sử hơn là ngữ nghĩa hiện tại

## 7. Hướng refactor tiếp theo

Nếu muốn làm repo sạch hơn nữa mà vẫn an toàn:

1. tách `main.c` thành `app_menu.c`, `app_calibration.c`, `app_explore.c`, `app_astar.c`
2. tách `systemTest.c` theo từng nhóm test
3. gom header vào subfolders khi đã chuẩn hóa include path
4. thêm test harness cho `maze_solver.c` chạy trên host
