# Kết quả startup bench — 16/10/2026

- Đã build và upload `motor_startup_96m_trim25` qua ST-Link; xem `build.log`, `upload.log`.
- Native C và 22 regression Python PASS; xem `native.log`, `self_test.log`.
- VM 8,4 V, bánh treo do chủ dự án xác nhận; không phải số đo điện áp tại board.
- Watchdog gây reset thực tế trước ARM; 66 xung khởi động, 66 BEGIN/END.
- Trái: có count từ 6,5%; ứng viên 8,5%, xác nhận quay duy trì 10/10.
- Phải: có count từ 7,0%; 7,5% chỉ đạt một lần khi quét, xác nhận 0/10; ứng viên 9,5%, xác nhận 10/10.
- Trái 8,5%: 119–134 count/250 ms, trung bình 124,8, CV 4,53%.
- Phải 9,5%: 129–139 count/250 ms, trung bình 134,9, CV 3,25%.
- UART không drop/lỗi RX; TIM11 không tăng late/gap trong capture.
- Đã STOP và khóa đầu ra: state 4, fault 5, applied trái/phải 0, CCR trái/phải 3200, direction ODR 0.
- Chưa thử 100% PWM; không sửa Motion/PID hoặc deadband mặc định của driver.

Chi tiết mẫu, hash nguồn/firmware: `report.json`. UART nguyên bản: `raw_uart.log`.
Định nghĩa tiêu chí và giới hạn: `docs/motor_driver/STARTUP_ENCODER_BENCH.md`.
