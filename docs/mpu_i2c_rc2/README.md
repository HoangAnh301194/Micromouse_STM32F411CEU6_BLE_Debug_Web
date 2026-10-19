# Biên bản bàn giao mô-đun I2C1–MPU6050 (RC2)

**Dự án:** Micromouse STM32F411CEU6 · **Nhánh:** `giomuadongbac` · **Ngày tổng hợp:** 19/10/2026 · **Kết luận:** chấp nhận bàn giao chức năng trong phạm vi đã thử nghiệm, kèm các giới hạn kỹ thuật dưới đây.

## 1. Ranh giới mô-đun

Mô-đun được đóng gói theo **giao diện và quyền sở hữu ngoại vi**. Các tệp driver vẫn ở vị trí hiện có để bảo toàn cấu hình build, thứ tự ngắt và các phụ thuộc đã kiểm chứng; không tạo một bản sao thứ hai của I2C1.

| Thành phần | Vị trí | Trách nhiệm |
|---|---|---|
| Giao diện tích hợp | [`include/mpu6050_i2c_module.h`](../../include/mpu6050_i2c_module.h) | Điểm `#include` duy nhất cho ứng dụng |
| Điều khiển MPU6050 | [`include/mpu6050.h`](../../include/mpu6050.h), [`src/mpu6050.c`](../../src/mpu6050.c) | Khởi tạo, hiệu chuẩn, đọc gyro, tích phân yaw, nhiệt độ chip |
| I2C1 | [`include/i2c.h`](../../include/i2c.h), [`src/i2c.c`](../../src/i2c.c) | Giao tiếp PB8/PB9, giao dịch bất đồng bộ bằng ngắt EV/ER |
| Hẹn giờ | [`src/system_timer.c`](../../src/system_timer.c) | Thời gian µs và hẹn đánh thức không chặn |
| Chủ sở hữu TIM11 | [`src/control.c`](../../src/control.c) | Nhịp 1 kHz, nhận bản chụp dữ liệu, không chờ I2C trong ISR |

**Cấu hình hiện tại:** STM32F411CEU6, đồng hồ danh định 96 MHz với HSI trim 25; MPU6050 địa chỉ 7-bit `0x68`, I2C1 ưu tiên 400 kHz và có fallback 300/200/100 kHz, dải gyro ±2000 °/s, DLPF 42 Hz, LPF phần mềm α = 0,7, vùng chết 0,3 °/s. Tần số lấy mẫu mục tiêu 1 kHz; dữ liệu điều khiển được công bố dưới dạng snapshot nhất quán. Tốc độ bus/CPU là giá trị cấu hình và tính toán RCC, chưa phải phép đo dao động ký.

### Hợp đồng tích hợp

1. Khởi tạo clock và bộ đếm thời gian trước khi gọi `MPU6050_Init(&handle)`.
2. Kiểm tra trạng thái trả về; bước nhận diện đọc thanh ghi `WHO_AM_I` mong đợi `0x68`.
3. Khởi động `MPU6050_Calibrate(&handle)` **khi robot cố định**. Quá trình không chặn gồm 5 giây làm ấm và 4.000 mẫu gyro hợp lệ (xấp xỉ 4 giây nếu đạt 1 kHz); giới hạn tối đa ba lần thử. Chỉ chấp nhận `cal_state=3`.
4. Duy trì `MPU6050_Update(&handle)` trong vòng lặp chính. TIM11 chỉ thông báo nhịp qua `MPU6050_NotifyControlTick()` và tiêu thụ `MPU6050_GetSnapshot()`; không thực hiện I2C blocking/printf trong ngắt.
5. Ứng dụng phải kiểm tra `valid` và tuổi của snapshot trước khi sử dụng góc/tốc độ góc cho điều khiển; giới hạn hiện hành là khoảng 3 ms.
6. Giao diện nhiệt độ `MPU6050_GetTemperatureMC(&handle, &temp_mc, &age_ms)` trả về nhiệt độ **chip MPU** theo milli-°C nếu đã có số đo hợp lệ. Không cần chân mới: `TEMP_OUT_H/L` tại `0x41/0x42` đọc qua I2C1 khoảng 1 Hz, dùng chung state machine bất đồng bộ. Phải kiểm tra tuổi mẫu. Chức năng này được bật bằng `MPU_TEMP_MONITOR=1` trong các cấu hình PlatformIO đang giữ.

Gói không chuyển thành một thư viện phần cứng độc lập có thể biên dịch tùy ý cho MCU khác: mã I2C phụ thuộc STM32F4 CMSIS, TIM5, TIM11 và sơ đồ chân của bo hiện tại. Đây là **bàn giao tích hợp trên bo STM32F411CEU6**, không phải chứng nhận tính di động đa nền tảng.

## 2. Kết quả nghiệm thu phần cứng

Ba phiên đo độc lập, mỗi phiên kéo dài 180 giây **sau khi MPU được hiệu chuẩn**, đã ghi nhận dữ liệu MPU thật, TIM11, I2C, IR và UART. Ở chế độ SHADOW và LIVE có thêm phép tính PID với đối tượng điều khiển giả lập; chỉ LIVE có PWM motor thật và số xung encoder phản hồi.

| Hạng mục | STILL (motor khóa) | SHADOW (PID giả lập 1 kHz, motor khóa) | LIVE (2S VM, PWM tối đa 100%) |
|---|---:|---:|---:|
| Bộ tiêu chí telemetry | PASS | PASS | PASS |
| Thời gian đo | 180 s | 180 s | 180 s |
| Bản tin MPU/yaw ở ~60 Hz | 10.799 | 10.799 | 10.799 |
| Tốc độ lấy mẫu gyro thực tế | ~998,2 Hz | ~998,5 Hz | ~999 Hz |
| Lỗi I2C / timeout | 0 / 0 | 0 / 0 | 0 / 0 |
| TIM11 trễ / mất chu kỳ | 0 / 0 | 0 / 0 | 0 / 0 |
| Tuổi mẫu MPU lớn nhất quan sát qua log | 1.609 µs | 1.533 µs | 1.598 µs |
| Thời gian giao dịch I2C lớn nhất | 134 µs | 133 µs | 155 µs |
| Yaw tích phân thô sau trừ bias, thay đổi cuối–đầu | +0,458° | +0,801° | +0,516° |
| Yaw có vùng chết, thay đổi cuối–đầu | +0,003° | −0,011° | +0,020° |
| Đỉnh hàng đợi UART TX / 1.024 B | 965 B | 979 B | 689 B |
| Nhiệt độ chip MPU6050 | Không thu | Không thu | 32,345–32,460 °C |
| PWM motor thực và phản hồi encoder | Không | Không | Có, áp dụng 80%, 90%, 100% |

**Cách hiểu kết quả:** LIVE có lệnh PWM công suất cao, encoder ghi nhận chuyển động, đồng thời MPU/I2C vẫn duy trì dữ liệu; điều này chứng minh hoạt động tích hợp trong phạm vi phép thử. Không có cảm biến đo dòng/áp motor, nhiệt độ H-bridge, từ trường EMI hay thiết bị đo góc tuyệt đối. Vì vậy kết quả không định lượng mức nhiễu điện từ, không chứng nhận độ chính xác khi ôm cua ở tốc độ cao. Thay đổi yaw trong LIVE cũng có thể bao gồm rung động hoặc chuyển động thật.

Yaw có vùng chết gần bằng 0 **không chứng minh** gyro không trôi. Hai thử nghiệm tĩnh cho thấy độ trôi thô sau hiệu chuẩn khoảng 0,458° và 0,801° trong 180 giây. Chưa có đủ phép thử lặp ở các mức nhiệt độ để quy sai lệch này cho trôi bias nhiệt, hay cho tải PID.

Nhiệt độ LIVE có **179 bản tin một giây**, bộ đếm nhiệt độ ghi nhận **189 lần đọc thành công, 0 lỗi**, thay đổi từ 32,416 °C đến 32,363 °C; khoảng biến thiên đo được là 0,115 °C. Tương quan nhiệt độ–giá trị gyro dư của phiên này là −0,0988 và không đủ để khẳng định quan hệ nhân quả.

### Nguồn kết quả được giữ nguyên

- [STILL và SHADOW: tổng hợp](../../results/imu_soak_20261019_115313/summary.json) · [toàn bộ CSV](../../results/imu_soak_20261019_115313/measurements.csv).
- [LIVE PWM 80–100%: tổng hợp](../../results/imu_soak_20261019_123733/summary.json) · [toàn bộ CSV](../../results/imu_soak_20261019_123733/measurements.csv).
- Các kết quả UART và motor tích hợp đã có từ trước vẫn nằm tại [`results/`](../../results/), không bị sửa hoặc rút gọn.

Đã gỡ các chương trình thử 180 giây riêng khỏi nhánh sau bàn giao. Bản cài đặt thí nghiệm có thể được truy xuất qua **lịch sử Git** trước thay đổi này; không suy diễn rằng vẫn còn lệnh chạy pipeline sau khi đã gỡ.

## 3. Quyết định bàn giao và những giới hạn còn mở

| Tiêu chí | Kết luận |
|---|---|
| Khởi tạo/nhận dạng MPU, cấu hình I2C và khả năng fallback | Đã kiểm chứng ba phiên gần nhất ở 400 kHz. Từng xuất hiện một lần fallback xấp xỉ 296 kHz trong bài đo cũ; nguyên nhân chưa xác định |
| Hiệu chuẩn bias và đọc gyro ~1 kHz | Đạt theo dữ liệu đã thu; có các khoảng trống mẫu ngắn, chưa định lượng đầy đủ phân phối P95/P99 |
| Tích phân yaw và công bố snapshot | Đạt về sự liên tục; sai số góc tuyệt đối khi robot quay nhanh chưa nghiệm thu |
| Đọc nhiệt độ chip MPU qua I2C | Đạt trong bài LIVE; không đại diện cho nhiệt độ nguồn/motor |
| TIM11 và I2C cùng hoạt động khi có PID mô phỏng | Đạt trong bài 180 giây; tải thuật toán speed-run đầy đủ chưa đo |
| I2C/MPU hoạt động khi motor quay PWM cao | Đạt về số mẫu, lỗi bus và bộ đếm TIM11 trong một lượt thử 180 giây |
| Mức pin, nhiệt độ, độ rung và EMI ở mọi tình huống | Chưa xác minh toàn bộ; cần nghiên cứu lặp và đo vật lý ngoài MCU |

**Trạng thái chính thức:** chấp nhận sử dụng mô-đun I2C–MPU6050 làm nền tảng phát triển các bộ điều khiển tiếp theo trên bo hiện tại. Đây là **nghiệm thu chức năng và tích hợp theo điều kiện thử**, không phải chốt độ chính xác định vị/góc quay của Micromouse tốc độ cao.

## 4. Dự kiến tối ưu tiếp theo

1. Đo góc quay trái/phải ±90°, ±180° và ±360° so với mốc góc độc lập để đánh giá scale, sai số cuối và độ trễ của bộ lọc.
2. Thử nghiệm lặp cùng một trạng thái cơ khí nhưng khác thời gian làm ấm, mức pin và nhiệt độ chip; chỉ thiết kế bù bias theo nhiệt độ nếu số liệu lặp lại cho thấy sự phụ thuộc ổn định.
3. Ghi P95/P99 khoảng cách mẫu và thời gian giao dịch I2C, đối chiếu `INT_STATUS`, trạng thái retry và các sự kiện >2 ms.
4. So sánh DLPF 42 Hz với 98 Hz ở cả tĩnh lẫn khi quay thật. Không thay cấu hình chỉ từ độ trôi tĩnh.
5. Đánh giá nhiễu khi motor quay nhiều mức PWM trên mặt sàn, kết hợp đo nguồn/dòng và góc tham chiếu; phân biệt tác động điện, cơ và rung.
6. Giữ biên dự phòng truyền UART phù hợp với tải log cụ thể. STILL/SHADOW cũ có đỉnh hàng đợi 94–96%, LIVE giảm còn 689 B; không suy rộng mức này sang mọi khung log.

## 5. Quản lý phiên bản

Giao diện mô-đun: `include/mpu6050_i2c_module.h`. Các thay đổi về clock, I2C1, ngưỡng tuổi mẫu, DLPF, hệ số hiệu chuẩn hoặc trình tự ngắt phải được đánh giá lại dựa trên bộ log gốc. Không sửa các log lịch sử để phản ánh một cấu hình mới.

**Đã loại bỏ:** `tools/imu_soak_pipeline.py`, `tools/test_imu_soak_pipeline.py` và firmware riêng cho chiến dịch thử 180 giây. **Được giữ:** mã nguồn I2C/MPU tích hợp, các bộ dữ liệu đo và công cụ thuộc Motor, UART, Clock vì các mô-đun đó có vòng đời và yêu cầu đánh giá riêng.
