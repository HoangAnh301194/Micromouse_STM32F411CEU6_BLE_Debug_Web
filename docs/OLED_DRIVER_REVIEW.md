# OLED Driver, I2C Integration and Firmware Review

**Review date:** 2026-10-27  
**Scope:** SSD1306 driver, OLED/MPU6050 shared-I2C implementation, application menu/button integration, firmware builds, and available automated/hardware evidence.

## Executive summary

| Area | Finding |
|---|---|
| SSD1306 driver | Implemented: initialization, cursor, text, framebuffer, graphics primitives, blocking refresh, and legacy DMA APIs are present. |
| Shared OLED/MPU transport | A separate cooperative OLED DMA transport exists for the motor-locked `oled_mpu_diag_96m` profile. It arbitrates I2C1 with MPU6050 and sends short dirty framebuffer runs. |
| Production OLED/status | A read-only status display is now available in opt-in `production_oled_status_96m`. The default `genericSTM32F411CE` build remains unchanged and OLED-disabled. |
| Production menu/button UI | Not implemented. No `Display_Menu()` implementation or seven-item OLED menu was found. |
| OLED button UI | Not implemented. PA0 button support exists in the separate headless system-test profile, not as OLED menu navigation. |
| Local firmware builds | Passed on the reviewed worktree for production, OLED/IR, and OLED/MPU profiles. |
| GitHub Actions | One recent run passed the OLED build and synthetic parser steps; a newer run failed in an unrelated IR native-test step before the OLED steps ran. |
| Hardware | A historical motor-locked OLED/MPU telemetry capture is recorded as passing. This review did not flash the board or independently verify visible pixels, wiring, motor-on operation, or long-duration stability. |

## 1. SSD1306 driver and API

The driver is in [`src/ssd1306.c`](../src/ssd1306.c), with declarations in [`include/ssd1306.h`](../include/ssd1306.h).

| Capability | Status | Notes |
|---|---|---|
| Initialize display | Implemented | `SSD1306_Init()` sends the initialization sequence using blocking I2C writes. It clears the framebuffer, calls `SSD1306_UpdateScreen()`, and then turns the display on. |
| Set cursor and write text | Implemented | `SSD1306_SetCursor()`, `SSD1306_WriteChar()` and `SSD1306_WriteString()` are available. |
| Draw pixels and shapes | Implemented | Pixel, line, rectangle, circle, triangle and bitmap drawing functions are present. Drawing changes the MCU framebuffer; it does not by itself transmit pixels to the OLED. |
| Update display | Implemented | `SSD1306_UpdateScreen()` performs a blocking full-screen transfer. |
| Legacy full-screen DMA API | Implemented, but not the shared-bus path | `SSD1306_Init_DMA()`, `SSD1306_UpdateScreen_DMA()` and `SSD1306_Process_DMA()` are declared. In builds with `OLED_SHARED_I2C`, initialization returns `SSD1306_ERROR` and the legacy update returns `0`; use `OLED_SharedService()` instead. |
| Shared-bus DMA declarations | Implemented conditionally | `include/i2c.h` declares `I2C_OledAsync_Start()` and `I2C_OledAsync_Poll()` under `OLED_SHARED_I2C`. This is a separate asynchronous, short-transfer transport. |

### Driver issues fixed in this update

1. `SSD1306_WriteFloat()` now rejects more than seven fractional digits and non-finite/out-of-range values by drawing `ERR`; the accepted maximum fits the output buffer.
2. `SSD1306_WriteInt()` now obtains the magnitude through a wider signed type, so `INT32_MIN` formats correctly.
3. `SSD1306_DMA_Callback()` is compiled only when the legacy DMA path is enabled, removing its unused-function warning from shared-I2C builds.

The host regression exercises these numeric edge cases against the real framebuffer rendering code; this is not physical-panel validation.

## 2. Current application, menu and button status

The normal application entry point is [`src/main.c`](../src/main.c). The default `genericSTM32F411CE` build initializes motor, encoder, BLE, IR, MPU6050 and control modules without initializing the OLED. The new opt-in `production_oled_status_96m` profile uses the same application and adds a read-only status display. OLED diagnostic/bench entry points remain available in the `OLED_MPU_DIAG` or `IR_OLED_TEST` profiles.

| Requested feature | Current status |
|---|---|
| `Display_Menu()` | Not found in the repository. |
| Seven menu items | Not present in the current production OLED application. |
| Short press to change menu item / long press to select | Not implemented as OLED-menu behavior. |
| PA0 button input | Present as a board-level active-low input. The opt-in `system_test_96m` profile uses press/release to start a test and a press to stop; its documentation explicitly describes it as **no OLED**. |
| Return to an OLED menu after motion | Not implemented because there is no production OLED menu integration. |

The status profile displays a `MICROMOUSE` startup screen followed by MPU calibration/readiness, motion state, BLE command hints and yaw. It performs blocking initialization and one blocking splash refresh before calibration/sampling begins. After sampling starts, it renders in main context and updates through `OLED_SharedService()`. Claims that it calls `SSD1306_Init_DMA()` or implements a seven-item menu do **not** match the source.

## 3. I2C wiring and addresses

The firmware configures I2C1 on PB8/PB9. OLED and MPU6050 connect **in parallel to the same two signal lines**; they do not have separate I2C buses.

| STM32F411 signal | MCU pin | Connect OLED pin | Connect MPU6050 pin | Notes |
|---|---:|---|---|---|
| I2C1 SCL | PB8 | SCL | SCL | Alternate function 4 in firmware. |
| I2C1 SDA | PB9 | SDA | SDA | Alternate function 4 in firmware. |
| Ground | GND | GND | GND | All modules and MCU need a common ground. |
| Module supply | Board/module-dependent | VCC | VCC | Confirm the module's supported voltage before wiring. STM32 GPIO is 3.3 V; ensure I2C pull-ups do not pull the bus to an unsafe voltage. |

| Device | Address used by firmware | Address caveat |
|---|---:|---|
| SSD1306 OLED | `0x3C` | The driver is fixed to this 7-bit address; some modules use `0x3D`. |
| MPU6050 | `0x68` | The code uses this address; the AD0 pin/board configuration must match. |

The pin and address definitions are in [`include/pinout.h`](../include/pinout.h); I2C1 GPIO and timing setup is in [`src/i2c.c`](../src/i2c.c). The configured pull-ups and nominal SCL rate in code do not prove that the physical bus has suitable pull-up resistance, rise time, voltage levels, or signal integrity. Confirm those on the assembled board.

## 4. DMA, blocking updates and shared-bus safety

### What the shared-I2C implementation does

- [`src/oled_mpu_diag.c`](../src/oled_mpu_diag.c) and the `production_oled_status_96m` path initialize the OLED before MPU calibration and `Control_Start()`.
- After MPU/control startup, the diagnostic renders to the framebuffer in main context and calls `OLED_SharedService()`. It does not use a blocking full-screen refresh.
- The production status path follows the same rule: the blocking splash is sent before sampling begins; subsequent updates use `OLED_SharedService()` from the main loop.
- [`src/oled_shared.c`](../src/oled_shared.c) compares framebuffer snapshots and schedules short changed-pixel runs. It limits work based on MPU state, the current sample timing, and configured I2C clock.
- [`src/i2c.c`](../src/i2c.c) arbitrates bus ownership, starts OLED DMA on DMA1 Stream6, and advances the I2C transaction through event/error IRQs. The shared renderer is cooperative; it does not perform OLED rendering from TIM11.
- The `ir_oled_test_96m` profile has exclusive OLED/I2C use and deliberately does not start the MPU6050. Its blocking screen refresh must not be used as evidence of shared-bus coexistence.

### Why a DMA driver alone does not establish stability

DMA only moves bytes for a particular transaction. It does not prove that two clients can safely access one physical I2C bus, that the MPU read timing remains acceptable, or that electrical conditions meet the I2C requirements. The shared profile therefore needs software ownership/arbitration, bounded transfer windows, error/timeout handling, sensor-read monitoring, and real hardware tests with both devices active. A standalone OLED test or a successful compile cannot replace those checks.

### API usage rule

Do **not** call blocking `SSD1306_UpdateScreen()` after MPU6050 sampling has started on the shared I2C1 bus. It sends full-screen blocking writes without going through the shared renderer's bus-admission policy. Similarly, do not attempt to combine the legacy full-screen `SSD1306_*_DMA()` path with the shared MPU configuration: the shared build disables the legacy API, and it must use `OLED_SharedService()` instead.

The statement “`main.c` calls `SSD1306_Init_DMA()` and then `SSD1306_UpdateScreen()`” is not true for the reviewed source. The potentially unsafe combination should still be prohibited in future integrations; the current shared diagnostic already keeps blocking initialization before MPU startup and uses the cooperative service afterward.

## 5. Build and automated-test evidence

### Local build performed for this review

Command:

```powershell
python -m platformio run -e genericSTM32F411CE -e production_oled_status_96m -e oled_mpu_diag_96m -e ir_oled_test_96m -e system_test_96m
python tests/regression/ssd1306_native_test.py
```

Result: **all five firmware environments and both host tests succeeded**.

Additional host test:

```powershell
python tools/oled_mpu_pipeline.py --self-test
```

Result: **`SELF_TEST_PASS`** for parser, frame assembly, and PASS/FAIL guard. This is a host-side synthetic test, not a physical OLED test.

### GitHub Actions evidence checked on 2026-10-27

- [Run #147](https://github.com/HoangAnh301194/Micromouse_STM32F411CEU6_BLE_Debug_Web/actions/runs/40494254234) on `giomuadongbac` succeeded. Its job reports successful steps for building `ir_oled_test_96m`, running the synthetic OLED-MPU parser test, and building `oled_mpu_diag_96m`. This verifies CI compilation/parser behavior for that run's commit; it is not hardware evidence.
- The newer [run #148](https://github.com/HoangAnh301194/Micromouse_STM32F411CEU6_BLE_Debug_Web/actions/runs/40494354956) failed in the **native six-channel IR filtering/capture regression** step. The OLED build and parser steps were skipped after that failure, so this latest run gives no new OLED result.
- These observed workflow runs are for `giomuadongbac`; they do not constitute an Actions run for the local `nhi` branch.

## 6. Available physical evidence and acceptance checklist

The repository contains a historical motor-locked OLED/MPU A/B capture at [`results/oled_mpu_20261024_082616/report.json`](../results/oled_mpu_20261024_082616/report.json), recorded as **PASS** for its telemetry checks. It reports 15-second OLED-off and OLED-on phases, 1 kHz TIM11, 1000 to 992.73 MPU reads/second, 1056/1056 OLED DMA transfers completed, 73 frames, no recorded OLED/I2C/DMA faults, and a 161 us maximum OLED transaction.

That result is limited to the captured motor-locked diagnostic configuration. The report itself says telemetry cannot establish electrical timing; the accompanying acceptance notes say visible pixel correctness, motor-on/noise behavior, extended soak and production-mode integration remain outside the demonstrated scope. This review did not upload firmware or independently repeat that test.

| Acceptance check | Status in this review |
|---|---|
| OLED boots and visibly shows the expected screen | Not tested on hardware in this review. |
| OLED acknowledges at `0x3C` on the assembled board | Not tested on hardware in this review. |
| MPU6050 reads while OLED updates | Historical motor-locked telemetry capture exists; not repeated in this review. |
| Seven-item menu navigation by short/long press | Not applicable yet; that menu is not implemented. |
| Robot motion with OLED updates, including BLE/control coexistence | Not verified; the production status profile is newly added and motor-on coexistence is not qualified. |
| Return to OLED menu after a mode/test | Not implemented or verified. |
| ST-Link flash and visible OLED confirmation | Not performed in this review. |

## 7. Recommended next steps

1. Keep the default firmware unchanged; flash `production_oled_status_96m` only after confirming the OLED wiring and supply.
2. If adding a menu, define the menu items and button policy (debounce, short/long press, selection, stop, and return-to-menu transitions); do not assume `Display_Menu()` exists.
3. Keep all post-MPU OLED updates in main context through the shared renderer. Never use blocking full-screen updates while MPU6050 owns the same active I2C1 bus.
4. On hardware, verify module voltage and pull-ups, scan/ACK both device addresses, then repeat the motor-locked OLED/MPU A/B capture with the new profile. Only after that, qualify motor-on/noise, prolonged display churn, visible pixels, menu behavior, BLE coexistence, and return-to-menu behavior separately.
