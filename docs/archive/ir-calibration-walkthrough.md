# Legacy Note: IR Calibration Walkthrough

Đây là ghi chú archive cho một phiên thay đổi calibration trước đây.

## Những gì từng được thêm

- mở rộng `IR_Simple_Calib_t` để giữ min/max của cụm sensor trái/phải
- thêm các bước calib khi ép robot sát tường trái và tường phải
- tính lại threshold từ dữ liệu min/max thay vì chỉ dựa vào hệ số cố định
- mở rộng phần in log calibration để dễ kiểm tra raw range

## Khi nào nên đọc tài liệu này

Chỉ cần đọc nếu bạn đang:

- sửa lại quy trình calibration
- thay đổi hình học robot hoặc vị trí cảm biến
- kiểm tra vì sao ngưỡng phát hiện tường được suy ra theo cặp `open/wall`

Đây là ghi chú lịch sử, không phải hướng dẫn vận hành chính của repo.
