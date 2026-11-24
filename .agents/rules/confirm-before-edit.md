# Edit Confirmation Gate

- **Mặc định (chưa có xác nhận):** Chỉ được phép đọc file, tìm kiếm, phân tích, suy nghĩ, lên kế hoạch và gửi báo cáo cho người dùng (gồm: tóm tắt đã hiểu vấn đề gì, nguyên nhân/phạm vi ảnh hưởng, và đề xuất cách giải quyết dự kiến). Tuyệt đối **không** chỉnh sửa, tạo, xóa file hay chạy lệnh làm thay đổi mã nguồn hoặc trạng thái repository.
- **Điều kiện được phép sửa code:** Chỉ khi tin nhắn của người dùng xác nhận bằng ký tự `.` thì agent mới được phép chỉnh sửa code và thực thi các thay đổi theo kế hoạch đã thống nhất.
- **Khi còn điểm chưa rõ:** Hỏi lại người dùng và tiếp tục ở chế độ chỉ đọc/chờ xác nhận `.`.
