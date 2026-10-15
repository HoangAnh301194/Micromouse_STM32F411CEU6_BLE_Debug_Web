# Rà soát yêu cầu chạy motor đến 100% PWM — 2026-10-15

## Điều kiện do chủ dự án báo

- Pin 2S 8,4 V, bánh nâng khỏi mặt đất. Không phải điện áp MCU đo được.
- Dấu encoder đã xác nhận; không khảo sát lại VCC/logic.

## Kết quả thực hiện

- COM6: `PASS_LOCKED_IDLE`, 12 snapshot, driver revision 2, trung bình 108,0263671875 cycles/call.
- Native C: PASS 6.000 duty có dấu, gồm ±1000 permille; 10.000 thay đổi lệnh; timeout, heartbeat 1500 ms, STOP và khóa.
- Regression chuyển trạng thái: PASS source/model; vẫn báo chuyển mạch vật lý chưa được xác minh.
- Không upload, không gửi START, không gỡ P0, không chạy motor hoặc PWM 100% trên phần cứng.
- Không sửa driver, Motion/PID, encoder, IMU hoặc IR trong lượt này.

## Chặn thực nghiệm

- `src/motor_auto_bench.c` giữ `MOTOR_AUTO_P0_ARM_LOCKED=1` và gọi `Motor_LockOutputs()`; không đổi khóa chỉ để nhận lệnh START.
- `src/motor.c` vẫn ghi nhận chuyển LOW/LOW/PWM-low trong STOP/drive chưa được xác minh điện. Mô hình không chứng minh dạng sóng thực; đây không phải khẳng định đã đo thấy hỏng phần cứng.
- Hồ sơ chỉ ghi motor N20 12 V; mã đầy đủ, tỷ số truyền, dòng khởi động/kẹt trục chưa biết. Bánh treo không tự xác minh các thông số này.
- Không có cảm biến dòng/nhiệt/áp sụt tích hợp; không suy các đại lượng đó từ encoder, không yêu cầu thêm kênh sensing giả vào firmware.

## Điều kiện cho phiên chạy tiếp

Cần rà soát chuyển STOP/drive bằng bằng chứng dạng sóng và xác định giới hạn motor/driver từ đúng mã linh kiện hoặc phép đo ngoài phù hợp; chuẩn bị phương án giới hạn/cắt dòng độc lập. Sau đó mới duyệt bench hữu hạn thời gian, tăng PWM từng mức, dừng giữa các lần thử, không thử stall/phanh ngắn mạch/đảo chiều đang quay.

100% trong native test chỉ xác minh ánh xạ lệnh sang thanh ghi. Không phải vận tốc đã đo, không xác nhận chạy tốc độ cao có tải hoặc tối ưu hoàn toàn.
