# Motion primitives — module hóa chuyển động Micromouse

**Trạng thái:** đã tạo lớp motion runtime; motion xoay tại chỗ 90° và 180°
được port từ logic SINGLEBRAKE v15, có kiểm thử native và build MCU.
**Chưa nghiệm thu phần cứng đường runtime mới**, đặc biệt là chiều quay phải.

## Cấu trúc

```text
Maze solver / Navigation / BLE (future opt-in)
           |
           | Motion_Turn90(LEFT/RIGHT, yaw)
           | Motion_Turn180(LEFT/RIGHT, yaw)
           | Motion_RequestPrimitive(kind, direction, yaw)
           v
include/motion.h + src/motion.c       <- cấp phát motion duy nhất
           |
           | MotionPrimitive_Start / Update / Abort
           v
include/motion_primitives.h           <- contract/state/result
src/motion_primitives.c               <- controller C thuần (không HAL)
           |
           | signed left_pm,right_pm (-1000..1000)
           v
Motor_SetPairPermille() / Motor_Stop()
           ^
           |
TIM11 1kHz -> Control IRQ -> 200Hz primitive control
                      ^
                      |
                MPU6050 yaw, gyro
```

Không copy PID khắp navigator. Điều hướng gửi **lệnh**,
controller quản lý pha và trả **trạng thái**.

## Motion được hỗ trợ

| Kind | Ý nghĩa | Đầu vào | Trạng thái |
|---|---|---|---|
| `MOTION_PRIMITIVE_PIVOT_90` | Xoay tại chỗ 90° | `LEFT / RIGHT` | Đã code, cần test runtime |
| `MOTION_PRIMITIVE_PIVOT_180` | Xoay tại chỗ 180° | `LEFT / RIGHT` | Đã code, cần test runtime |
| `MOTION_PRIMITIVE_SLALOM_90` | Cua slalom 90° khi robot đang tiến | — | Dự phòng, trả UNSUPPORTED |
| `MOTION_PRIMITIVE_STRAIGHT` | Motion đi thẳng theo profile mới | — | Dự phòng, trả UNSUPPORTED |

Quy ước: **LEFT = yaw dương**, **RIGHT = yaw âm**.
Cả góc quay và tốc độ gyro được đổi sang hệ tọa độ dọc theo
chiều quay đã chọn; output motor được đổi dấu ngược lại khi
quay phải. Phần đối xứng logic chỉ được native-test, **chưa**
được chứng minh bằng thử nghiệm tốc độ cao quay phải.

## API sử dụng

```c
#include "motion.h"

/* Chỉ gọi khi có snapshot MPU6050 mới, hợp lệ và <=3ms tuổi,
 * và không có motion khác đang chạy. */
MotionPrimitiveResult rc =
    Motion_Turn90(MOTION_DIR_LEFT, snapshot.yaw_deg);

/* Hoặc */
rc = Motion_Turn90(MOTION_DIR_RIGHT, snapshot.yaw_deg);
rc = Motion_Turn180(MOTION_DIR_LEFT, snapshot.yaw_deg);
rc = Motion_Turn180(MOTION_DIR_RIGHT, snapshot.yaw_deg);

/* Chương trình chính có thể kiểm tra trạng thái: */
MotionPrimitiveStatus status = Motion_GetPrimitiveStatus();
MotionPrimitiveResult reason = Motion_GetPrimitiveResult();
MotionPivotPhase phase = Motion_GetPrimitivePhase();

/* Dừng cưỡng bức bất kỳ lúc nào: */
Motion_Stop();
```

Không gọi các lệnh liên tiếp khi primitive vẫn ở `RUNNING`.
Hàm command trả `MOTION_PRIMITIVE_BUSY`, `BAD_ARGUMENT` hoặc
`UNSUPPORTED` nếu yêu cầu không hợp lệ. `MOTION_PRIMITIVE_DONE`
chỉ báo điều kiện góc ±2° và gyro <=12°/s giữ ~150ms ở runtime,
KHÔNG tương đương bài nghiệm thu 800ms post-stop của firmware
đo kiểm độc lập.

## Điều khiển SINGLEBRAKE v15

- 1ms TIM11: cập nhật snapshot và motor driver.
- 5ms: cập nhật PID vận tốc/pha quỹ đạo.
- 90°: tăng tốc → phanh ngược một lần → thả trôi → tiếp cận → dừng.
- 180°: cùng chuỗi, nhưng kết thúc phanh khi gyro còn ~330°/s;
  cho phép vào pha tiếp cận sau ~30ms coast, gyro <=120°/s.
- Cấm tái gia tốc cao sau khi đã vào phanh, giới hạn phần
  điều chỉnh cuối <=155‰.
- Bảo vệ sai chiều, gyro >875°/s, góc vượt >2°, timeout,
  gyro không hợp lệ, đứng yên ngoài vùng đích, trôi góc khi đang xác nhận.
- Điều khiển primitive mới **không thay đổi**
  `src/turn_pid_diag.c`, profile đo kiểm và chuỗi log của nó.

Các thông số pha được chuyển sang code C độc lập từ v15
và chưa được hiệu chuẩn bằng dữ liệu runtime ở cả 2 chiều.
Đặc biệt `Motor_Stop()` là trạng thái Hi-Z theo driver,
không đồng nghĩa dừng cơ học tức thời.

## Tích hợp và tương thích

- `Motion_Update()` vẫn là vòng được gọi bởi TIM11.
  Khi `MOTION_PIVOT_PRIMITIVE` hoạt động nó gọi controller mới
  và xuất PWM hai bánh. Khi lệnh hoàn tất hoặc lỗi, motor dừng.
- Các API cũ `Motion_CommandTurn()`, `Motion_CommandStraight()`,
  BLE `L/R/F` **chưa chuyển sang primitive mới**, tránh âm thầm
  thay các động tác đang có. Muốn dùng điều khiển mới phải gọi
  `Motion_Turn90/180` rõ ràng.
- `Motion_Stop()` dừng theo yêu cầu người dùng, `Motion_SensorFault()`
  báo lý do `SENSOR_FAULT` khi MPU6050 không có snapshot hợp lệ;
  cả hai đều buộc motor về trung tính. Robot phải có nguồn ngắt
  motor độc lập và kiểm tra thao tác dừng khẩn cấp trước mỗi test.
- Module mới không phụ thuộc UART, BLE, web, IR, mê cung. Slalom
  sau này có thể thêm bộ tạo quỹ đạo cung cong với vận tốc vào/ra
  khác 0 mà không sửa các hàm gọi ở navigation.

## Kiểm tra

```bash
python tests/regression/motion_primitives_native_test.py
python -m platformio run -e genericSTM32F411CE
python -m platformio run -e turn_pid_diag_96m
```

Native kiểm tra: kiểu motion hợp lệ/bị từ chối, trái/phải đảo PWM,
unwrap yaw 180°, ngắt do lỗi sensor/góc/rate, trạng thái dừng,
nhả phanh 180° khi còn vận tốc dương, không tăng công suất cao lại
sau phanh. Kiểm thử này **không mô phỏng robot thực** và không
thay thế thử nghiệm thực địa.

**Hạng mục nghiệm thu sau:** quay phải 90°, hai chiều 180° nếu cần,
kiểm chứng deadtime, độ lặp, hồi tiếp encoder, đối chiếu với
log SINGLEBRAKE diagnostic; chỉ sau đó mới chuyển lệnh BLE
hoặc navigator sang API mới.
