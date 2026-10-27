# BÁO CÁO NGHIỆM THU IR 6 KÊNH — YYYY-MM-DD

**Status:** NOT_QUALIFIED | ACQUISITION_ACCEPTED | STATIC_FILTER_ACCEPTED | FILTER_DYNAMIC_ACCEPTED | IR_CALIBRATION_ACCEPTED

**Nguồn:** docs/ir/IR_AGENT_QUALIFICATION_PROTOCOL.md

## 1. Cấu hình, provenance và an toàn

| Trường | Giá trị |
|---|---|
| Repo/branch/HEAD | |
| Firmware/PlatformIO env | ir_filter_diag_96m |
| Compiler/CI URL | |
| Máy đo/COM port | |
| Người vận hành vật lý | |
| Thời gian và nơi đo (chỉ thông tin đủ xác định điều kiện đo) | |
| Xác nhận motor VM tháo vật lý | |
| Các bản ghi IRH.locked=1 | |
| Loại tường/chất liệu/màu/ánh sáng | |
| Cách định vị vách, độ không chắc chắn | |
| 6 kênh/thứ tự [L90,L45,L0,R0,R45,R90] | |
| Kênh loại trừ và lý do (nếu có) | |

## 2. Manifest thí nghiệm và bằng chứng

| Scenario | Channel/group | d thực (mm) | Góc/bề mặt/ánh sáng | Trial | Thời lượng | Thư mục log | Chất lượng |
|---|---|---:|---|---:|---:|---|---|
| no-wall | 6 channels | n/a | | 1 | 30s | | |
| static | 6 channels | 20 | white_matte/room | 1 | 30s | | |
| static | ... | 10,15,... | | 1,2,3 | 20s | | |
| step | ... | | | >=10 events | | | |
| ambient / crosstalk | ... | | | | | | |

Không coi manifest được điền trước là log có thật. Mỗi dòng hoàn thành phải trỏ tới raw_uart.log và summary.json.

## 3. Audit dữ liệu

| Tiêu chí | Ngưỡng/kỳ vọng đã chốt trước đo | Đo được | PASS/FAIL |
|---|---|---|---|
| CI/native build | Success | | |
| IRH motor locked | 100% | | |
| Lỗi DMA/ADC/UART | delta = 0 | | |
| Malformed frames | 0 | | |
| Số IRF/giây | ~40 Hz, >=95% sau warm-up (tùy source) | | |
| Số IRB/giây | ~5 Hz | | |
| Số IRH/giây | ~1 Hz | | |
| Scan rate thực | đo từ IRH.scans | | |
| 6 kênh đủ, không trùng/lẫn scan | 100% | | |
| ADC bão hòa OFF/ON | phải xác định và loại vùng không hợp lệ | | |

## 4. Bảng lọc theo từng kênh/vị trí

| Kênh | d (mm) | Ứng viên | Mean ADC | STD | MAD | STD reduction % | Bias ADC | Spike rate | SNR | Kết luận |
|---|---:|---|---:|---:|---:|---:|---:|---:|---:|---|
| L90 | | raw | | | | | | | | |
| L90 | | EMA 0.10 | | | | | | | | |
| L90 | | EMA 0.25 | | | | | | | | |
| L90 | | median3 | | | | | | | | |
| L90 | | median3+EMA 0.25 | | | | | | | | |
| L45/L0/R0/R45/R90 | | | | | | | | | | |

Phân tích riêng trường hợp nền 0 ADC, bão hòa 4095, out-of-range và chất liệu khác; không dùng mean/STD đơn lẻ để công nhận.

## 5. Đáp ứng động và độ trễ

| Kênh | Cạnh | d (mm) | Bộ lọc | 10–90% rise ms | Trễ so raw ms | Tốc độ log / timestamp | Số cạnh/mất cạnh | Đạt latency budget? |
|---|---|---:|---|---:|---:|---|---|---|
| | rising/falling | | | | | COARSE_40HZ_ONLY / MCU_TIMESTAMPED | | |

Bộ lọc dùng Wall PID tốc độ cao cần kiểm tra ngân sách dịch chuyển v_robot × latency; không kết luận <=5ms từ log 40Hz.

## 6. Cross-talk, tính ổn định vật liệu/ánh sáng

Ghi bảng confusion hoặc ảnh hưởng biên độ trên kênh khác khi chỉ có một vách trái, phải hoặc trước. Tách nguyên nhân do hình học (tia thực sự nhìn thấy vách) và nhiễu điện/quang.

## 7. LUT theo kênh và holdout

| Kênh | Loại filter | Vùng hợp lệ mm | Chiều đơn điệu | Holdout MAE mm | Holdout P95 abs mm | OOR rule | Kết luận |
|---|---|---|---|---:|---:|---|---|
| L90 | | | | | | | |
| L45 | | | | | | | |
| L0 | | | | | | | |
| R0 | | | | | | | |
| R45 | | | | | | | |
| R90 | | | | | | | |

Không có holdout thì LUT gate INCONCLUSIVE; không tự suy khoảng cách khi chưa calibration.

## 8. Bảng quyết định G0–G6

| Gate | Status | Ngưỡng | Quan sát thực tế | Evidence path | Blocking issue / retry |
|---|---|---|---|---|---|
| G0 Motor safety | BLOCKED | | | | |
| G1 Pipeline/data integrity | INCONCLUSIVE | | | | |
| G2 Coverage | INCONCLUSIVE | | | | |
| G3 Static filter/SNR | INCONCLUSIVE | | | | |
| G4 Dynamic latency | INCONCLUSIVE | | | | |
| G5 Robustness/crosstalk | INCONCLUSIVE | | | | |
| G6 LUT on holdout | INCONCLUSIVE | | | | |

**QUYẾT ĐỊNH:** ... (không có gate = PASS theo mặc định).

**Bộ lọc tạm chọn theo kênh:** ...

**Mức đủ điều kiện chuyển sang Wall PID:** ...

**Việc phải làm tiếp:** ...

**Repo changes (nếu có):** commit hash, tests/CI và link.

**Người xác nhận nghiệm thu vật lý:** ... (nếu không có xác nhận, để trống, không tự ký).
