# Multi-Agent Coordination & Live Sync

Quy tắc này áp dụng khi nhiều agent cùng làm việc, đọc, chỉnh sửa hoặc build/test trên cùng repository này. Không chia cứng phạm vi sở hữu file; thay vào đó các agent đồng bộ qua một bảng tin chung duy nhất và tách `git worktree` khi chạy song song.

## 1. Kênh giao tiếp chung duy nhất
- Bảng tin chung đặt tại **main worktree**: `<main-worktree>/.agents/AGENT_SYNC.md` (file này được liệt kê trong `.gitignore`, không commit vào Git).
- Dù agent đang làm việc ở main worktree hay worktree phụ, mọi agent đều phải đọc và ghi vào **cùng một file** `<main-worktree>/.agents/AGENT_SYNC.md` (có thể xác định main worktree bằng `git worktree list`).
- Mỗi mục ghi ngắn gọn theo mẫu:
  `- [YYYY-MM-DD HH:MM] [<Agent/Task>] [<IN_PROGRESS|NOTICE|BLOCKED|DONE>] (branch/worktree) — tóm tắt ngắn; file liên quan; lưu ý cho agent khác.`

## 2. Trước khi chỉnh sửa (Check-in)
- Kiểm tra `git status --short`, `git diff --stat` và đọc các mục gần nhất trong `<main-worktree>/.agents/AGENT_SYNC.md`.
- Ghi một dòng `IN_PROGRESS` báo rõ task đang làm, branch/worktree đang dùng và các file/module dự kiến chạm tới.
- Luôn **đọc lại nội dung mới nhất của file ngay trước khi sửa** để tránh ghi đè thay đổi mà agent khác vừa thực hiện.

## 3. Khi làm việc chung trên cùng repo
- **Tuyệt đối không** dùng các lệnh xóa/hủy thay đổi diện rộng (`git checkout -- .`, `git reset --hard`, `git clean -fd`) hoặc tự ý revert/ghi đè thay đổi chưa commit của agent khác.
- Khi cùng sửa trên một file với agent khác: chỉ chỉnh đúng vùng logic thuộc task của mình. Nếu phát hiện xung đột về thiết kế hoặc logic với thay đổi đang có của agent khác, ghi mục `BLOCKED` vào `AGENT_SYNC.md` và báo người dùng thay vì tự ý ghi đè.
- Khi thay đổi giao diện hoặc cấu hình dùng chung (header trong `include/`, struct/enum chung, `platformio.ini`, pinout, format telemetry/protocol, v.v.), phải ghi ngay một mục `NOTICE` vào `AGENT_SYNC.md` để các agent khác biết và cập nhật theo.

## 4. Quy tắc dùng `git worktree` độc lập (có điều kiện)
- **Đơn lẻ / tuần tự** (chỉ 1 agent đang sửa code, các agent khác chỉ đọc/phân tích): làm trực tiếp trên main worktree, không cần tạo worktree mới.
- **Song song** (từ 2 agent trở lên cùng sửa code hoặc cùng chạy build PlatformIO / test): agent thứ 2 trở đi **nên tạo một `git worktree` độc lập** trên branch tạm tách từ branch đích (`giomuadongbac`) để tránh ghi đè file trực tiếp và tránh đụng độ thư mục build `.pio/`.
- Khi tạo worktree phụ:
  - Ghi rõ đường dẫn worktree và tên branch vào mục `IN_PROGRESS` trong `<main-worktree>/.agents/AGENT_SYNC.md`.
  - Sau khi công việc được merge/tích hợp xong (theo yêu cầu người dùng), dọn worktree bằng `git worktree remove <path>` và cập nhật `DONE` vào `AGENT_SYNC.md`.

## 5. Khi kết thúc hoặc bàn giao (Check-out & Git)
- Ghi mục `DONE` hoặc `BLOCKED` vào `<main-worktree>/.agents/AGENT_SYNC.md`: liệt kê file đã sửa, trạng thái build/test thực tế, và mọi lưu ý hoặc blocker còn mở cho agent khác.
- Khi commit (chỉ khi người dùng yêu cầu), **chỉ stage đúng các file/hunk thuộc phần việc của mình**; không dùng `git add -A` khi working tree đang có thay đổi của agent khác.
- `AGENT_SYNC.md` là nhật ký đồng bộ trong lúc làm việc; bàn giao mức milestone/workstream vẫn cập nhật vào `PROJECT_STATUS.md` theo `AGENTS.md`.
- Khi `AGENT_SYNC.md` quá dài (ví dụ > 50 mục), agent có thể dọn các mục `DONE` cũ sau khi các thông tin quan trọng đã được phản ánh vào `PROJECT_STATUS.md`.
