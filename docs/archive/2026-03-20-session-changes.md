# Session Notes: 2026-03-20

Đây là bản tóm tắt đã làm sạch từ ghi chú phiên chỉnh sửa cũ.

## Nội dung chính

- tinh chỉnh hysteresis threshold cho IR wall detection
- đổi cooldown của back-align theo mode chạy
- ổn định handoff tốc độ sau các turn trong explore/A*
- dọn warning và biến unused
- tinh chỉnh A* speed PID, stall timeout và T1WT trigger
- bổ sung logic chain turn cho U-turn và S-curve

## Giá trị của ghi chú này

Tài liệu này hữu ích nếu cần hiểu tại sao một số hằng số trong `main.c` và
`motion_controller.c` lại được chọn như hiện tại, đặc biệt với:

- watchdog timeout
- post-detection pivot logic
- T1WT dynamic threshold
- reset tốc độ sau turn

Nó không còn là tài liệu chính để onboarding. Hãy đọc `README.md` và
`docs/architecture.md` trước.
