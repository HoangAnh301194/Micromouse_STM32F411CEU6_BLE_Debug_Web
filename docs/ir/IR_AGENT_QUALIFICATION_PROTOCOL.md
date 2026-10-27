# Quy trình thí nghiệm và nghiệm thu IR 6 kênh — dành cho Coding/Hardware Agent

**Mục tiêu:** xây dựng bằng chứng đủ để quyết định bộ lọc IR cho Micromouse, sau đó nghiệm thu từng phần. Không tự động tuyên bố toàn bộ IR/WALL PID đã đạt chỉ từ CI, số liệu tĩnh hoặc một lần chạy.

**Nguồn triển khai:** docs/ir/IR_FILTER_CALIBRATION_PLAN.md, src/ir_sensor.c, include/ir_sensor.h, src/ir_filter.c, src/ir_filter_diag.c, tools/ir_filter_pipeline.py, tests/regression/ir_filter_native_test.py, tools/test_ir_filter_pipeline.py, platformio.ini.

**Điểm chuẩn source khi viết tài liệu:** giomuadongbac @ 4c320200d1a036073993fc8f8e76b8290b756753. Khi thực thi phải đọc lại HEAD, CI và mã nguồn thật; không dùng commit này để bỏ qua thay đổi mới.

## A. Quy tắc không được vi phạm

1. Tất cả phép đo IR được tiến hành **motor VM tháo vật lý**, firmware khóa motor: Motor_LockOutputs(); IRH.locked=1. Không phát lệnh điều khiển bánh xe, không chạy pivot/slalom khi đo IR. Chỉ nạp qua ST-Link khi người vận hành đã xác nhận cấp nguồn và kết nối an toàn. Nếu không có quyền điều khiển máy tính/board: cung cấp lệnh và nhận log người vận hành, **không giả vờ tự chạy**.
2. Không sửa hoặc retune Motion/PID/turn_pid_diag, BLE, flash maze, tham số motor. Không tự tích hợp bộ lọc vào IR_GetLatest production khi chưa có kết luận.
3. Không chỉnh ngưỡng PASS sau khi nhìn thấy dữ liệu chỉ để hợp thức hóa kết quả. Nếu thay đổi tiêu chí, ghi version, lý do, người phê duyệt và chạy lại phép đo độc lập.
4. Không tự suy khoảng cách từ ADC. Người vận hành phải đo khoảng cách vật lý từ đầu thu-phát IR đến mặt chuẩn theo đúng hướng từng cảm biến. Ghi độ không chắc chắn (ví dụ ±1 mm nếu chỉ dùng thước tay).
5. Không dùng log 40 Hz để tuyên bố độ trễ 1–10 ms. Nếu cần độ trễ cỡ mili giây, bổ sung test bước với lấy mẫu MCU tốc độ cao hoặc mốc GPIO xác định sự kiện. Không sửa luồng production để tăng logging.
6. Tuyệt đối không coi dữ liệu dựng/synthetic/native-test là dữ liệu phần cứng; luôn phân biệt TEST_SOFTWARE và HARDWARE_MEASUREMENT. Không tự đặt status PASS cho các mục chưa được đo.

## B. Bản đồ cảm biến và hợp đồng dữ liệu

| Chỉ số | Kênh | Lắp đặt/ứng dụng | Định hướng phép thử |
|---|---|---|---|
| 0 | L90 | ở phía sau, nhìn **về trước** | vách trước bên trái |
| 1 | L45 | chéo bên trái | mép tường, góc tiếp cận |
| 2 | L0 | đầu xe, nhìn **sang trái** | lệch tâm so với tường trái |
| 3 | R0 | đầu xe, nhìn **sang phải** | lệch tâm so với tường phải |
| 4 | R45 | chéo bên phải | mép tường, góc tiếp cận |
| 5 | R90 | ở phía sau, nhìn **về trước** | vách trước bên phải |

Đầu vào xử lý phải là d=max(LED_ON-LED_OFF,0), theo **cùng chu kỳ scan**. Các ứng viên: RAW, Median3, EMA(alpha=0.10), EMA(alpha=0.25), Median3→EMA(alpha=0.25). Tuyệt đối không gộp ON/OFF của hai scan khác nhau. Không nhầm số thứ tự 90/45/0 với hướng nhìn vật lý.

IRF xuất ở 40 Hz gồm d/med/e10/e25/hy cho cả sáu kênh. IRB 5 Hz chứa OFF/ON; IRH 1 Hz gồm scan/lỗi/lock/drop. Bộ lọc cập nhật theo các lần quét mới gần 1 kHz chứ không phải 40 Hz.

## C. Điều kiện đầu vào / chuẩn bị

**Phần người vận hành cần xác nhận trước khi agent chạy:** COM7 hoặc cổng thực tế; dây PA9/PA10/GND UART 3.3 V; ST-Link; VM đã ngắt; tường thử có vị trí cố định; thước kẹp/thước đo, vị trí đầu cảm biến, vật liệu vách, hướng vách, chiếu sáng; robot cố định, không lăn; không có OLED/MPU tranh quyền trong firmware IR isolated.

**Phần agent tự động hóa:** git status/branch/head; kiểm tra diff; build và native test; nạp sau xác nhận; đọc IR_FILTER HELLO; capture với metadata; kiểm tra integrity; lập bảng CSV, biểu đồ, thống kê và báo cáo; cảnh báo theo từng gate; tạo commit rõ ràng nếu có sửa code hoặc tài liệu.

~~~powershell
git pull --ff-only origin giomuadongbac
python tests/regression/ir_filter_native_test.py
python tools/test_ir_filter_pipeline.py
python -m platformio run -e ir_filter_diag_96m
# Chỉ nạp sau xác nhận ngắt VM và khóa motor:
python -m platformio run -e ir_filter_diag_96m -t upload
~~~

Chuỗi HELLO phải chứa: IR_FILTER,HELLO,ir_filter_v1_motor_locked. Mọi bước có lỗi giữ trạng thái BLOCKED, không chuyển giai đoạn.
Firmware diagnostic phát HELLO khi khởi động và lặp lại theo nhịp IRH 1 Hz để host có thể mở COM sau khi board đã chạy. Collector chờ HELLO trước khi tính đủ thời lượng capture; nếu hết timeout không có HELLO, dữ liệu vẫn được lưu nhưng `valid_capture=false`.

## D. Ma trận thực nghiệm và thứ tự chạy

### P0 — tính toàn vẹn sensor/ADC/DMA/UART

1. Không vách, sáng phòng bình thường, 30 giây. Ghi metadata distance_mm rỗng, scenario=no-wall.
2. Vách trắng mờ, tĩnh, khoảng cách thực đo 20 mm, 30 giây, scenario=static.
3. Lặp cùng cấu hình thêm một lần. Kiểm tra IRF, IRB, IRH và đồ thị 6 kênh. Không kết luận tất cả kênh tốt chỉ vì một cặp thay đổi.
4. Nếu thấy một kênh bằng 0 suốt: phân biệt tường ngoài vùng đo, sai đấu dây/LED, bão hòa OFF, hay dữ liệu không thu được; ghi FAULT/OUT_OF_RANGE, không ép qua gate.

Ví dụ thu dữ liệu:

~~~powershell
python tools/ir_filter_pipeline.py --port COM7 --seconds 30 --scenario no-wall --surface none --light room --trial 1 --plot
python tools/ir_filter_pipeline.py --port COM7 --seconds 30 --scenario static --distance-mm 20 --surface white_matte --light room --trial 1 --plot
~~~

### P1 — đo theo khoảng cách, từng kênh

Dải đề xuất: 10, 15, 20, 25, 30, 40, 50, 60, 80 mm. Mỗi mức và mỗi bố trí: **3 lượt độc lập × 20 giây** (nhấc ra đặt lại gá trước lượt tiếp theo). Chỉ tiếp tục trong vùng IR thực sự có độ nhạy; nếu mất tín hiệu, đánh dấu giới hạn thay vì ngoại suy.

Nhóm L0/R0: đặt vách bên theo hình học ngang. Nhóm L45/R45: đặt vách chéo đúng hướng; ghi góc vách và cự ly vuông góc. Nhóm L90/R90: đo với vách phía trước, tính từ vị trí đầu cảm biến phía sau robot. Nếu một vị trí vách đồng thời kích thích nhiều kênh thì **vẫn xuất kết quả từng kênh**, không gán tất cả cùng cự ly trừ khi đã đo thực.

Giữ cố định ít nhất 2 giây sau khi đổi gá rồi mới bắt đầu capture; luôn ghi light/surface/distance-mm/trial/notes (gồm góc tới, sai số định vị và kênh mục tiêu). Có thể chia test thành nhóm kênh để giảm công sức, nhưng tối thiểu 3 lượt/kênh/vùng cự ly được dùng trong LUT.

### P2 — tính nhiễu và chọn ứng viên lọc

Từ ir_filter_trace.csv, lấy mean, STD, MAD, percentiles p95/p99, số mẫu spike, thống kê độ bền qua 3 lượt. Với kênh c và bộ lọc f:

- noise_reduction(c,f)=1-std(f)/std(raw), nếu std(raw)>0; nếu raw không dao động, ghi N/A.
- bias(c,f)=mean(f)-mean(raw); nhận xét tác động lên ngưỡng phát hiện.
- SNR(c)=|median(d_wall)-median(d_no_wall)| / max(std(d_no_wall),std(d_wall),1 count); công thức chỉ là tiêu chí kỹ thuật đề xuất.
- kiểm tra ADC ON/OFF có clipping (4095), ambient thay đổi và cross-talk.

**Bộ lọc ưu tiên để đánh giá:** Median3→EMA 0.25, nhưng chỉ chọn nếu vượt qua cả nhiễu và độ trễ. So sánh EMA 0.25 như đối chứng nhanh, EMA 0.10 là đối chứng mạnh. Khuyến khích hệ số theo nhóm/kênh nếu chứng minh khác biệt thống kê.

### P3 — đáp ứng động và thời gian trễ

Thử vách xuất hiện/mất đi tối thiểu 10 lần mỗi cấu hình điển hình (đầu dải, giữa dải, cuối dải), gắn nhãn rising/falling. Chạy --scenario step. Báo t10/t50/t90 và trễ so với raw từ chuỗi 40 Hz dưới nhãn **COARSE_40HZ_ONLY**; không suy diễn tới 5 ms. Muốn nghiệm thu high-speed Wall PID phải dùng event-timestamp/MCU trace được xác nhận mốc, ưu tiên >=200 Hz hoặc 1 kHz và dấu thời gian nguồn gốc; nếu thiếu thì P3=INCONCLUSIVE.

### P4 — robust/crosstalk

- Chạy không tường, ánh sáng phòng, đèn chiếu mạnh và/hoặc ánh sáng tự nhiên.
- Vách trắng mờ, vách đen mờ, vách bóng; ghi rõ chất liệu.
- Dời robot ±5 mm về trái/phải giữa hai vách; xoay ±5°, ±10° cho kênh chéo.
- Đặt riêng một vách trái, một vách phải, một vách trước; kiểm tra biến thiên kênh đích và không đích để phát hiện nhiễu chéo.
- Lặp cấu hình kiểm thử sau khi đổi nguồn chiếu và nguồn 3.3 V nếu có thể đo.

### P5 — LUT và hồi quy độc lập, chỉ sau khi P0–P4 đạt

1. Chọn vùng ADC–khoảng cách đơn điệu có SNR đủ; tạo LUT **riêng từng kênh**, không dùng một LUT chung cho cả 6 vị trí.
2. Giữ hẳn một tập thử độc lập (một lượt lắp đặt chưa dùng để fit, hoặc một số cự ly xen giữa). Không đánh giá bằng chính mẫu vừa fit LUT.
3. Báo MAE_mm, RMSE_mm, P95_abs_mm, sai lệch gần/xa, điểm mất tín hiệu, bão hòa; trả OUT_OF_RANGE khi ADC ngoài khoảng LUT, không ngoại suy.
4. Không đưa LUT/bộ lọc vào production hoặc mở motor trong phiên này nếu chưa được chấp thuận triển khai riêng.

## E. Gate nghiệm thu (không bịa số đo)

Mỗi gate có trạng thái PASS / FAIL / INCONCLUSIVE / BLOCKED, ghi rõ observed/required/evidence. Ngưỡng dưới đây là **đề xuất kỹ thuật ban đầu** để agent áp dụng nhất quán, KHÔNG phải yêu cầu được xác nhận bởi thí nghiệm thực tế. Agent phải ghi chúng vào test manifest trước khi chạy.

| Gate | Điều kiện tối thiểu | Bằng chứng |
|---|---|---|
| G0 — SAFETY | VM rời vật lý được người vận hành xác nhận; mọi IRH.locked=1; không lệnh motor | checklist + IRH |
| G1 — DATA INTEGRITY | native/CI/build PASS; frame hợp lệ; malformed=0; UART drop delta=0; DMA err/to/ovr/fail delta=0; IRF đủ 6 kênh; IRB cùng scan | raw_uart.log, summary.json |
| G2 — COVERAGE | 3 lượt/cự ly/kênh thực cần hiệu chuẩn; OFF/ON có nhãn; cự ly, góc, vật liệu, ánh sáng và sai số gá đã ghi | manifest + các CSV |
| G3 — STATIC FILTER | mọi kênh được dùng có SNR>=5 trong vùng được nhận; lọc được chọn giảm STD ít nhất 25% so với raw tại phần lớn cự ly; không làm mất biến thiên do vách thật | metric + đồ thị |
| G4 — DYNAMIC | có ít nhất 10 cạnh xuất hiện/mất đi cho mỗi vùng quan trọng; không mất edge; trễ định lượng phải được đo ở tốc độ lấy mẫu đủ để đánh giá mục tiêu robot | event log + t10/t50/t90 |
| G5 — ROBUSTNESS | không có lỗi nhầm vách gây nguy hiểm trong tập test; định lượng false positive/negative trong ánh sáng/chất liệu/góc; đạt ngưỡng đã chốt trước chạy | ma trận confusion + cross-talk |
| G6 — DISTANCE LUT | LUT đơn điệu trong vùng dùng; holdout MAE <=2 mm và P95 absolute error <=5 mm như **mục tiêu tạm đề xuất**; ghi rõ vùng không hỗ trợ | calibration + holdout |

G1 bổ sung kiểm tra hiệu dụng mạnh hơn flag valid_capture hiện có: với 20 giây, IRF mong đợi khoảng 800 bản ghi, IRB khoảng 100, IRH khoảng 20; agent so số thực/độ dài đo, khởi động hệ thống, mất liên tục và sai số dấu thời gian. Gợi ý yêu cầu >=95% lượng kỳ vọng sau khi trừ warm-up; nếu sampling thực khác danh nghĩa phải giải thích trước khi thay ngưỡng. Tần số scan phải đo từ IRH.scans, tham chiếu prior ~1 kHz, không mặc định đạt vì UART 40 Hz.

**Độ trễ để chạy wall PID ở tốc độ cao:** đề xuất ràng buộc v_target × latency<=5 mm (ví dụ v=1m/s thì latency<=5ms). Đây là ngân sách thiết kế, chưa chứng minh cảm biến đáp ứng được. Nếu chỉ có log 40 Hz thì G4 không thể PASS giới hạn 5ms. Nêu rõ điều kiện HOLD thay vì ép kết luận.

**Quy tắc công nhận:**
- ACQUISITION_ACCEPTED khi G0 & G1 PASS.
- STATIC_FILTER_ACCEPTED khi G0–G3 PASS; được chọn bộ lọc ở mức tĩnh, **chưa** gắn Wall PID tốc độ cao.
- FILTER_DYNAMIC_ACCEPTED khi G0–G5 PASS, có latency thỏa ngân sách thiết kế.
- IR_CALIBRATION_ACCEPTED khi G0–G6 PASS trên tập holdout.
- WALL_PID_READY chỉ là đề xuất chuyển tích hợp; cần bài kiểm chứng bộ điều khiển trên robot riêng.
- Bất kỳ thiếu bằng chứng an toàn, một gate FAIL, hoặc INCONCLUSIVE thì **không đánh dấu IR_CALIBRATION_ACCEPTED**.

## F. Kết quả bắt buộc của agent

Trong docs/ir/reports/IR_VALIDATION_YYYYMMDD.md:
1. HEAD/CI, công cụ, firmware, môi trường đo, xác nhận người vận hành, danh sách tình huống/manifest.
2. Bảng coverage 6 kênh x khoảng cách x trial x surface x light; link tới raw logs và thông tin vị trí thật.
3. Bảng lọc cho từng kênh/cự ly, mean/std/MAD, noise reduction, bias, saturation và SNR.
4. Đồ thị 6 kênh trước/sau lọc; ON/OFF vs d, biểu đồ ADC-distance, step response và đánh giá robust.
5. So sánh ứng viên, quyết định ưu tiên cho từng kênh, có đánh đổi độ trễ, vùng đo, ảnh hưởng PID.
6. Bảng gate G0–G6 có PASS/FAIL/INCONCLUSIVE/BLOCKED, observed/threshold/evidence và các lỗi hiện hữu.
7. Kết luận chính xác một trong: NOT_QUALIFIED / ACQUISITION_ACCEPTED / STATIC_FILTER_ACCEPTED / FILTER_DYNAMIC_ACCEPTED / IR_CALIBRATION_ACCEPTED. Nghiệm thu không đồng nghĩa đã merge vào production.
8. Các thay đổi code/test/commit và lệnh kiểm thử lại; không tự xóa raw files, không overwrite/force-push.

Ngoài báo cáo Markdown, xuất ít nhất manifest.csv, metrics_by_channel.csv, acceptance.json, log thô, 6 hình từng kênh; tên folder dưới results/ir_qualification_YYYYMMDD/... hoặc liên kết rõ đến results/ir_filter_.../ đã tạo.

Nếu phần cứng chưa được giao cho agent, dừng ở PREPARED / BLOCKED_HARDWARE, chỉ xuất lệnh thao tác người vận hành, tài liệu, tool và test phần mềm. Không gán giá trị ADC giả làm kết quả thật.
