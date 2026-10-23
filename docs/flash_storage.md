# Flash storage (ported from main)

**State: source integrated, not activated or hardware qualified.**

This port retains the public API names from `main` in `flash_storage.h`:
`Flash_SaveMaze/LoadMaze`, `Flash_SaveIRCalib/LoadIRCalib` and
`Flash_SaveMPUCalib/LoadMPUCalib`. The data definitions live in
`flash_legacy_types.h` instead of importing the removed
`maze_solver.h` and `ir_simple_calib.h` algorithms. Legacy persistent map
data was **5 x 5**, not the 16 x 16 web-console map. Values are NOT yet
connected to the new motion, IR calibration, or MPU6050 runtime.

## Memory map (STM32F411CEU6, 512 KiB internal flash)

| Address | Erase sector | Legacy payload |
| --- | --- | --- |
| `0x08040000` | Sector 6 (128 KiB) | `IR_Simple_Calib_t` |
| `0x08040100` | Same Sector 6 | `MPU_Calib_Flash_t` |
| `0x08060000` | Sector 7 (128 KiB) | `Maze_Persistent_t` |

The port preserves those addresses and legacy magic values (`0xCA`,
`0xEB`, `0xA5`) for compatibility with `main`.
It fixes the source's rounded-up word-writing over-read by padding the
last program word with erased bits (0xFF).

## Safety and limitations

- **No startup erase/write, no live save, no control-loop changes.** Source
  compiles as part of the standard firmware but none of its entrypoints are
  invoked in `src/main.c`.
- The original flash routines perform **blocking** sector erase/program;
  reads/execution from internal flash may stall. Do NOT call saves while
  TIM11 1-kHz control, motor PWM, or other timing-critical work is live.
  Stop the control loop, positively disable motor outputs and preferably
  remove motor VM before controlled Flash experiments.
- Sector 6 is shared: saving either IR or MPU calibration erases the whole
  sector and restores only the other recognized legacy record. Power loss
  during erase/program may destroy **both** records.
- The legacy format validates only one-byte magic flags; it has no CRC,
  version, atomic copy, journal or wear-levelling. This is a **compatibility
  port**, not yet a robust persistent-storage design.
- Before adding writes, verify the exact deployed linker image fits strictly
  below `0x08040000` (sectors 0–5), verify voltage conditions and measure
  worst-case blocking time. Do not reformat sector 6/7 before backing up
  any valuable calibration or maze data previously saved by `main`.
- No board read/write/erase acceptance evidence yet. A build passing does
  not validate persistence or power-loss resilience.

## Follow-up

Add safe, explicit motor-locked maintenance-mode tests (read-only first),
then write/readback/erase with independent backup; only then connect a
versioned, integrity-checked schema to the new calibration and map modules.
