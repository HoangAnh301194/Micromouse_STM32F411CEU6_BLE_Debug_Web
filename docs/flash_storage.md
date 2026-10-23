# Flash persistence safety (STM32F411CEU6)

**Source changes only. No Flash write/erase is invoked from `src/main.c`.**
These changes have CI build validation, but require physical-board save/readback/reset
acceptance before enabling in navigation.

## Memory ownership

| Range | Use | Guard |
| --- | --- | --- |
| `0x08000000..0x0803FFFF` (256 KiB, sectors 0..5) | Firmware | `ld/STM32F411CE_FLASH_256K.ld` plus `board_upload.maximum_size=262144` |
| `0x08040000..0x0805FFFF` (128 KiB, sector 6) | IR and MPU calibration | Flash driver only |
| `0x08060000..0x0807FFFF` (128 KiB, sector 7) | Maze map | Flash driver only |

Linker script adapted from STMicroelectronics STM32CubeF4 template (BSD-3-Clause).
The linker MUST reject any executable image over 256 KiB. Confirm actual ELF
section addresses during CI and ensure your flashing tool does not **mass erase**
the data sectors. Existing data should be backed up before first testing.

## Storage format

Legacy `main` layouts (map 5x5; IR and MPU calibration structs) remain at the
first 1024 bytes of each data sector. A fresh new record is appended into the
next completely erased 1024-byte slot, with a v1 header:

- 32-bit magic `MMF1`, packed version/type, sequence, payload length, CRC32.
- Up to **1000 bytes payload**; CRC32 protects metadata and payload.
- Last word in slot is a commit marker, written **only after readback verifies
  the payload and CRC**. Partial writes are ignored on restart.
- Each 128 KiB data sector provides 127 append-only slots (slot 0 is legacy).
  `FLASH_ERR_FULL` means **no further writes are attempted**; the driver
  never automatically erases a full sector. Safe reclamation needs separate
  backup and maintenance workflow, especially for sector 6 shared records.
- For new 16x16 maps, use `Flash_SaveMapData` / `Flash_LoadMapData`.
  The legacy 5x5 `Flash_SaveMazeChecked` is compatibility only.

## Safety contract

**Before every save:** motor fully stopped, call `Control_Stop()`,
`Motor_LockOutputs()` (requires reboot to rearm), and `IR_Stop()`;
keep a stable supply. The checked Flash APIs fail closed unless the TIM11
control loop is stopped and motor outputs are locked. TIM5 provides the
hardware-microsecond timeout; calls before `SystemTimer_Init()` are rejected.
**Never invoke a save in any ISR.**

32-bit programming clears PG before/after the operation, verifies each word,
checks Flash status errors and uses a 3-second busy timeout. These guards do
not eliminate rare hardware stuck-bus states or a supply collapse during
programming; power-loss testing on real hardware is still required.

CRC calculation lives in `src/flash_record_format.c` and has an independent native C regression (`tools/flash_record_native_test.py`) cross-checked against Python zlib. This is not an on-board memory-write test.\n\nThe compatibility `void Flash_Save*/Load*` functions remain; prefer the
`Flash_*Checked` APIs because they return a precise status.

### Remaining limitations

- Only 127 writes per data sector until maintenance reclamation; no GC yet.
- Legacy data has no CRC; fallback can only validate its old magic byte.
- No automatic map save-on-goal, map restore/startup, or IR/MPU runtime wiring.
- No physical-board power-loss, flash endurance or ST-Link upload-preservation
  acceptance has yet been performed. GitHub Actions **cannot** replace that.
