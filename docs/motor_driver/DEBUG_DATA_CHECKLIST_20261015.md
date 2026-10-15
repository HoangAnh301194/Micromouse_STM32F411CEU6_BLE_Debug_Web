# Checklist debug và thu dữ liệu Motor Driver — 2026-10-15

## Phạm vi và giới hạn

- Chỉ Motor Driver, firmware bench, thu thập và phân tích dữ liệu. Không sửa Motion/PID, thuật toán encoder, IMU hoặc IR.
- Không khảo sát lại VCC/logic theo yêu cầu chủ dự án.
- Không có cảm biến dòng, nhiệt, điện áp nguồn: ghi `NOT_MEASURED`, không giả định PASS hoặc suy ra các đại lượng này từ encoder.
- Không tự mở khóa P0, upload, START, thử kẹt trục, phanh ngắn mạch hoặc đảo chiều đang quay. Phiên này chỉ gửi `MOTOR PING` và `MOTOR DRIVER`.
- Đo encoder đánh giá tổ hợp driver–motor–cơ khí trong điều kiện thử; chưa đủ quy nguyên nhân riêng cho driver.
- Các phép đo chức năng không cần hoàn thành 12 capture oscilloscope trước khi phân tích. Tuy nhiên, thanh ghi/encoder không thay thế chứng cứ dạng sóng và đầu ra điện. [R1, R2]

## Dữ liệu mới đã thu

Thời điểm bắt đầu UART: **2026-10-15 22:45:24 +07:00**. Thư mục bằng chứng:

`motor_logs/driver_checklist_20261015_224503/`

| Hạng mục | Kết quả mới | Giới hạn kết luận |
| --- | --- | --- |
| Git HEAD | `e6e59b19381b185c485d8197d658c6e46f6d2c34` | Mốc mã nguồn trên máy, không phải hash firmware đọc từ MCU |
| COM6 handshake | Profile v14, driver revision 2, `p0_locked=1`, idle | Firmware tự báo; chưa có hash image trên MCU |
| Native test | PASS: 6.000 duty có dấu, 10.000 thay đổi lệnh | Register mocks; không mô phỏng điện hoặc preload vật lý |
| Probe self-test | PASS | Kiểm tra parser, register, chu kỳ, allowlist |
| Offline batch | **9/9 PASS_OFFLINE** | Tổng kết: `REVIEW_AWAITING_FUNCTIONAL_EVIDENCE` |
| Neutral microbenchmark | 12 snapshot, tổng 3.072 lần gọi | IRQ bị tắt riêng trong từng lần đo; không phải phép đo đang kéo tải |
| Driver idle | Trung bình **108,026367 cycles/call**, min 108, max 114 | Chưa trừ overhead đo; không phải mức sử dụng CPU toàn hệ thống |
| TIM11 ISR max được báo | **863 cycles** | Giá trị cực đại tích lũy tới snapshot, không phải WCET bảo đảm |
| `late`, `gaps`, `tx_drop` | Đều 0 trong các snapshot | Chưa có phân phối jitter từng nhịp hoặc thử dài hạn/có tải |
| Encoder trái/phải | 0, không thay đổi giữa các snapshot | Không chạy motor; không có phép đo đáp ứng cơ học mới |
| TIM2 | `CCR1=CCR2=3200`, period counts 3200; direction GPIO thấp | Trạng thái thanh ghi neutral, không đo điện áp chân hoặc Hi-Z thực |
| Tham số hiện tại | Deadband trái/phải 0; rise/fall 20/30 permille/ms | Chưa được hiệu chuẩn bằng thử nghiệm chạy firmware hiện tại |

Clock 96 MHz và PWM 30 kHz là giá trị danh định/tự báo; không phải kết quả đo độc lập bằng thiết bị ngoài.
So với baseline revision 1 lưu ngày 2026-10-14 (233,173828 cycles/call), idle hiện giảm khoảng 53,67%.
Đây là so sánh tham chiếu lịch sử, **không phải A/B ghép cặp vừa chạy lại**, không chứng minh motor nhanh hơn hoặc hiệu suất điện tốt hơn.
Các log chạy motor cũ không được coi là xác nhận đáp ứng của revision hiện tại.

Các file chính:

- `collection_summary.json`: manifest, kết quả tổng hợp, các phép đo chưa có.
- `idle_samples.csv`: 12 snapshot chuẩn hóa, đơn vị và bộ đếm tích lũy ghi trong tên cột.
- `uart/raw_uart.log`, `uart/driver_probe.json`: dữ liệu UART gốc và dữ liệu parse.
- `native_test.log`, `probe_self_test.log`: kết quả kiểm thử host.
- `offline/p0_batch_report.json`, `offline/p0_batch_summary.txt`: bằng chứng từng gate.

Hash `motor.c` trong manifest là hash file trên PC. Không gán nó cho image đang chạy trên MCU nếu chưa có build ID/hash image tương ứng.
Dòng chú thích lịch sử về board v7 trong báo cáo offline không phải nhận dạng firmware hiện tại; phiên UART mới trả profile v14.

## Checklist theo thứ tự

### A. Không chạy động cơ

- [x] A1. Lưu commit, hash nguồn, profile UART, tham số PWM/slew/deadband và raw log. Lần build/upload tới phải lưu thêm environment, build flags và hash image; không upload trong phiên này.
- [x] A2. Chạy native test và probe self-test; yêu cầu không có lỗi assertion/parser/allowlist.
- [x] A3. Chạy batch offline; yêu cầu toàn bộ 9 gate PASS, nhưng không đổi trạng thái điện thành PASS.
- [x] A4. Thu neutral probe COM6; yêu cầu lock, idle, GPIO chiều thấp, CCR neutral, encoder không đổi, scheduler tiếp tục chạy, không mất UART.
- [ ] A5. Thu trace SRAM 512 mẫu ở 1 kHz để tính phân phối chu kỳ TIM11. Công cụ hiện yêu cầu chủ dự án xác nhận VM đã ngắt vật lý; phiên này chưa có xác nhận nên chưa thực hiện. Không tự truyền `--confirm-vm-disconnected`.
- [x] A6. Chủ dự án đã xác nhận dấu encoder; không kiểm tra lại. Counts/vòng chỉ cần khi quy đổi sang RPM/m/s, không chặn debug driver bằng counts/s. Pipeline mới ghi nhận `USER_CONFIRMED_NOT_RETESTED`.

### B. Có chạy motor, chỉ sau khi bench được cho phép arming

Tất cả B1–B6 hiện **PENDING** vì firmware còn P0 lock. Không gửi START để vượt khóa. Trước mỗi phép đo, phải có người giám sát, bánh/robot được giữ an toàn và cách ngắt nguồn vật lý. Dừng phiên nếu có chuyển động bất ngờ, mất liên lạc hoặc encoder không đúng dự kiến. PWM thấp không được xem là bảo vệ dòng điện.

- [ ] B1. Đo vùng khởi động và vùng duy trì quay riêng cho trái/phải, mỗi chiều; tăng/giảm duty theo từng mức nhỏ, ghi cả thành công lẫn thất bại. Ghi target, applied, CCR và GPIO chiều, không chỉ phần trăm lệnh. Đề xuất 10–20 lần ở các điểm gần ngưỡng. Phân biệt ngưỡng khởi động với ngưỡng giữ quay; không lấy một lần có xung encoder làm deadband đã hiệu chuẩn. Cách khảo sát phi tuyến tĩnh/hysteresis tham khảo [R3], không sao chép giá trị của motor khác.
- [ ] B2. Đo độ lặp lại của xung chạy: giữ cùng duty, slew, thời gian và điều kiện cơ khí; đưa bánh về trạng thái nghỉ quan sát được trước lần kế tiếp. Đề xuất 10–20 lần mỗi cấu hình; xuất mean/std/CV số đếm, tỷ lệ khởi động và độ phân tán thời gian đạt 5 counts. Đảo thứ tự A/B giữa các nhóm nếu so sánh hai cấu hình để giảm ảnh hưởng trôi theo thời gian; không giả định điện áp pin được giữ cố định.
- [ ] B3. Đo tốc độ ở duty cố định: chỉ tính CV đoạn plateau sau khi chứng minh đã hết tăng tốc. Profile hiện tại chỉ chạy 200 ms, log 20 ms, chưa bảo đảm có plateau. Nếu không thấy plateau đủ dài, báo `INSUFFICIENT_PLATEAU`; không tự kéo dài thử nghiệm hoặc tăng duty. Dùng counts thô và timestamp MCU để phân tích, không lấy riêng tốc độ đã lọc làm chứng cứ độ êm. [R3, R4]
- [ ] B4. So sánh slew: thay một tham số mỗi lần, cùng duty và cửa sổ đo; ghi thời gian applied đạt target, thời gian bắt đầu quay, quãng quay và quãng trôi sau STOP. Profile cũ đã có rise 5 so với 20 permille/ms; chưa có xác nhận mới trên motor. Chỉ kết luận đánh đổi độ trễ/độ lặp lại; không khẳng định giảm dòng hoặc nhiệt khi chưa đo chúng.
- [ ] B5. Kiểm tra STOP/watchdog ở mức thử đã được duyệt: ghi thời điểm lệnh/timeout, target/applied/CCR/GPIO sau STOP, counts trôi thêm và chuyển động ở cuối cửa sổ. Phân biệt software neutral với bánh đã dừng. Thử mất heartbeat không được biến thành thử phanh hay đảo chiều. Hiện stop tail 400 ms; hết cửa sổ vẫn chuyển động thì ghi chưa quan sát được dừng, không tự rút ra thời gian dừng chính xác.
- [ ] B6. So sánh một bánh/hai bánh ở cùng cấu hình đã được duyệt: ghi mean/CV mỗi bánh, sai khác trái–phải, timing TIM11 và lỗi UART. Nếu thử cùng tải phần mềm IMU/IR, chỉ quan sát telemetry có sẵn, không đổi các module đó. Không coi chênh tốc hai bánh tại cùng PWM là lỗi driver hoặc bù nó bằng PID trong checklist này.

### C. Không có dữ liệu, không tự động thử

- [ ] C1. Dạng sóng PWM, thứ tự cạnh GPIO/PWM, OUT1/OUT2 và Hi-Z thật: `NOT_MEASURED`, cần thiết bị ngoài nếu muốn xác nhận điện. Preload CCR đọc được không phải latch đầu ra đang hoạt động. [R1, R2]
- [ ] C2. Dòng, nhiệt, áp nguồn/sụt áp: `NOT_MEASURED`; không nằm trong phần đo tự động trên phần cứng hiện tại.
- [ ] C3. Kẹt trục, short-brake khi đang quay nhanh, đảo chiều gấp, duty cao hoặc chạy kéo dài: không tự thực hiện. Bộ chờ đảo chiều 30 ms là quy tắc phần mềm hiện tại, không phải thông số chứng nhận an toàn của TB6612. [R1]
- [ ] C4. So sánh PWM 20/30/40 kHz: chưa ưu tiên. Chỉ có encoder thì có thể so sánh đáp ứng chức năng, chưa kết luận tổn hao/nhiệt/hiệu suất; giữ baseline 30 kHz. Blog micromouse [R7] là ví dụ thực tế vì sao cần kiểm tra clock timer, không phải bằng chứng 30 kHz tối ưu cho robot này.

## Công thức và cách đọc kết quả

Các ngưỡng thử, số lần lặp và điều kiện plateau dưới đây là thiết kế thí nghiệm đề xuất cho dự án, không phải tiêu chuẩn micromouse chung.

| Chỉ số | Cách tính / báo cáo | Điều kiện hợp lệ |
| --- | --- | --- |
| Tốc độ encoder | `speed_pps = 1e6 * (count_now - count_prev) / delta_time_us` | Timestamp MCU cùng nguồn, `delta_time_us > 0`, không có mẫu mất; xử lý wrap timestamp |
| Độ lặp lại xung chạy | `cv_count_pct = 100 * sample_std(counts_per_trial) / abs(mean(counts_per_trial))` | Cùng thời gian/slew/trạng thái ban đầu; mean khác 0, ít nhất 2 lần hợp lệ; đề xuất 10–20 lần |
| Dao động tốc độ plateau | `cv_speed_pct = 100 * sample_std(speed_pps) / abs(mean(speed_pps))` | Chỉ một bánh/một chiều/một mức duty trên plateau; không trộn đoạn tăng tốc, STOP hoặc đổi chiều |
| Trôi theo thời gian | So sánh mean ở đầu/cuối plateau; lưu cả tốc độ thô và xu hướng | Không xóa xu hướng rồi chỉ báo CV đẹp; chưa đo pin thì không gán nguyên nhân cho driver |
| Khởi động | `successful_starts / attempted_starts`; mean/std/median/max thời gian đạt 5 counts | Không bỏ các lần thất bại khỏi mẫu số; mốc 5 counts là phép quan sát, không phải tốc độ hữu ích đã đạt |
| Đáp ứng STOP | Counts ròng và tổng `abs(delta_counts)` sau STOP; chuyển động ở 100 ms cuối cửa sổ | Net counts bằng 0 chưa chứng minh không rung; một mẫu delta bằng 0 chưa chứng minh dừng hẳn |
| Jitter TIM11 | `period_us = timestamp_now - timestamp_prev`; báo min/max, P95/P99 của `abs(period_us - 1000)` | Cần trace từng nhịp, không suy ra P99 từ bộ đếm lỗi hay ISR max |
| Thời gian ISR | Cycles và `cycles / (reported_core_hz / 1e6)` microseconds danh định | Cần đo có tải đúng profile; max quan sát không phải WCET bảo đảm |

NIST định nghĩa CV là `s/mean`; biểu diễn phần trăm theo trị tuyệt đối ở đây tương ứng relative standard deviation. [R5]
Không dùng CV khi mean gần 0 hoặc tín hiệu quá ít counts. Với cửa sổ 20 ms, bước phân giải một count tương ứng 50 counts/s; đây là bước phân giải, không phải sai số được bảo đảm. Có thể gộp cửa sổ để giảm lượng tử hóa nhưng phải công bố cửa sổ và độ trễ; không sửa thuật toán encoder trong phạm vi này. [R4]
Timestamp `ms` hiện có độ phân giải ms; muốn phân tích jitter mức microsecond phải dùng trace TIM5, không dùng timestamp host UART.
Encoder hiện có cửa sổ 10 ms và bộ lọc `alpha=0.30`; vì vậy phải giữ raw total counts để không chỉ đánh giá tín hiệu đã làm mượt.

Tiêu chí chức năng cứng đề xuất: không có mẫu thiếu/trùng/reset giữa phép đo, encoder đúng dấu, stop phần mềm về neutral, không có UART drop hoặc gap TIM11. Counter `late` hiện đếm chu kỳ ngoài 950–1050 us; `gaps` đếm trên 1500 us. Lưu counter đầu/cuối và xét phần tăng trong phiên.
CV không có ngưỡng PASS 3%/5% phổ quát; phải chốt mục tiêu sau baseline, độ phân giải và điều kiện tải. `20/20` khởi động thành công chỉ mô tả 20 lần quan sát, không chứng nhận độ tin cậy 100%.

## Dữ liệu cần giữ cho phiên chạy tiếp theo

```text
session_id, git_head, host_source_sha256, flashed_build_id, firmware_profile,
wheel, direction, trial_id, load_condition, start_condition, requested_permille,
applied_permille, deadband, rise, fall, timer_hz_reported, arr, ccr_l, ccr_r,
direction_gpio, mcu_timestamp, timestamp_unit, encoder_l, encoder_r,
phase, stop_reason, tim11_late, tim11_gaps, isr_max_cycles, tx_drop
```

- Mỗi dòng có timestamp/phase; lưu raw log trước khi lọc hoặc tổng hợp. Quy cách này tham khảo cách nghiên cứu [R6] giữ lệnh PWM, tốc độ và khoảng lấy mẫu; không sao chép các kênh cảm biến mà robot không có.
- Metadata pin chỉ ghi do người dùng báo, kèm thời điểm; không chuyển thông tin 7,7 V của phiên trước thành điện áp đo hiện tại.
- Lưu thứ tự lần thử, các lần lỗi và lý do loại mẫu. Không cộng 3.072 lần gọi idle thành 3.072 lần thử motor độc lập.
- Không quy đổi `CCR=ARR+1` khi GPIO neutral thành đang drive 100%; phải đọc cùng trạng thái chiều.
- Mỗi báo cáo tách `PASS_OFFLINE`, `PASS_LOCKED_IDLE`, `MEASURED_FUNCTIONAL`, `NOT_MEASURED` và lỗi thật. Không gộp thành nhãn “tối ưu hoàn toàn”.

## Nguồn đối chiếu

Đọc/đối chiếu ngày 2026-10-15. Chỉ vận dụng phần đặc tính cơ cấu chấp hành, phép đo và thời gian thực; không lấy phần thiết kế controller để mở rộng sang PID/Motion.

| ID | Nguồn gốc | Điểm áp dụng / giới hạn |
| --- | --- | --- |
| R1 | Toshiba, **TB6612FNG datasheet**, bản ngày 2026-05-13, trang 4, mục Control Function; `https://toshiba.semicon-storage.com/info/docget.jsp?did=10660&prodName=TB6612FNG` | Phân biệt Stop và Short brake bằng truth table, không suy trạng thái điện chỉ từ encoder. |
| R2 | STMicroelectronics, **AN4776 Rev 4 — How to use general-purpose timer peripheral on STM32 MCUs**, February 2026, mục 1.4.2; `https://www.st.com.cn/resource/en/application_note/an4776-how-to-use-generalpurpose-timer-peripheral-on-stm32-mcus-stmicroelectronics.pdf` | Preload và active register khác nhau, chuyển nội dung theo update event; không áp mọi tính năng timer STM32 mới cho F411. |
| R3 | Von Chong et al., **OpenMCT: an open-source DC motor control educational kit**, HardwareX 26 (2026), e00794, DOI `10.1016/j.ohx.2026.e00794`; `https://pmc.ncbi.nlm.nih.gov/articles/PMC13224385/` | Khảo sát dead zone/hysteresis và giữ PWM, tốc độ, khoảng lấy mẫu khi nhận dạng. Phần cứng DRV8874/Teensy có sensing khác robot này; không sao chép ngưỡng hoặc bảo vệ dòng. |
| R4 | Texas Instruments, **SPRU790D — TMS320x280x, 2801x, 2804x Enhanced Quadrature Encoder Pulse Module**, trang 11; `https://www.ti.com/lit/ug/spru790d/spru790d.pdf` | So sánh đếm vị trí theo cửa sổ và đo khoảng thời gian cạnh; giới hạn lượng tử hóa theo tốc độ/cửa sổ. Chỉ lấy nguyên lý phép đo, không dùng thanh ghi C2000 cho STM32. |
| R5 | NIST Dataplot, **Coefficient of Variation**; `https://www.itl.nist.gov/div898/software/dataplot/refman2/auxillar/coefvari.htm` | Công thức CV/RSD và giới hạn khi mean gần 0. Không cung cấp ngưỡng đạt chung cho motor/micromouse. |
| R6 | Von Chong, Cárdenas, **Experimental data from an open-source DC motor control educational kit for calibration, system identification, and controller validation**, Data in Brief 69 (2026), 113274, DOI `10.1016/j.dib.2026.113274`; `https://pubmed.ncbi.nlm.nih.gov/42835008/` | Đối chiếu abstract/mô tả bộ dữ liệu: giữ raw/processed files, lệnh, tốc độ, loop interval và metadata để tái lập. Chỉ áp dụng các kênh sẵn có. |
| R7 | Harjit Singh, **PWM frequency and power consumption**, Micromouse Online, 2010-09-05; `https://micromouseonline.com/2010/09/05/pwm-frequency-and-power-consumption/` | Trải nghiệm thực tế lỗi hiểu clock TIM2/APB làm PWM sai hệ số hai; không phải nghiên cứu định lượng chọn tần số tối ưu cho robot hiện tại. |

## Việc làm tiếp

1. Chạy pipeline hợp nhất theo `docs/motor_driver/DEBUG_PIPELINE.md`; đã có báo cáo độ lặp lại/CV, plateau, slew và stop tail trên host. Các mục chưa có dữ liệu vẫn được báo riêng.
2. Khi chủ dự án xác nhận VM đã ngắt, thu trace A5. Dấu encoder đã xác định; không lặp lại mục này.
3. Chỉ lên lịch B1–B6 sau khi việc cho phép arming bench được review riêng. Không coi checklist, native test hoặc neutral probe là quyết định mở P0.
