#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

#include "ir_simple_calib.h"
#include "maze_solver.h"

/* STM32F411 Flash Sector 7 starts at 0x08060000 (128KB) — Maze data */
#define FLASH_STORAGE_ADDR 0x08060000

/* STM32F411 Flash Sector 6 starts at 0x08040000 (128KB) — IR + MPU calibration */
#define FLASH_IR_CALIB_ADDR 0x08040000
#define IR_CALIB_MAGIC 0xCA

/* MPU6050 gyro calibration — stored in Sector 6 at offset 256 */
#define FLASH_MPU_CALIB_ADDR (FLASH_IR_CALIB_ADDR + 256)
#define MPU_CALIB_MAGIC 0xEB

typedef struct {
  float gyro_z_offset;  /* Calibrated gyro Z offset (deg/s) */
  uint8_t is_valid;     /* MPU_CALIB_MAGIC if valid */
} MPU_Calib_Flash_t;

/* Save and load Persistent Maze Map to Flash (Sector 7) */
void Flash_SaveMaze(Maze_Persistent_t *pm);
void Flash_LoadMaze(Maze_Persistent_t *pm);

/* Save and load IR Calibration to Flash (Sector 6) */
void Flash_SaveIRCalib(IR_Simple_Calib_t *calib);
void Flash_LoadIRCalib(IR_Simple_Calib_t *calib);

/* Save and load MPU Calibration to Flash (Sector 6, offset 256) */
void Flash_SaveMPUCalib(MPU_Calib_Flash_t *mcalib);
void Flash_LoadMPUCalib(MPU_Calib_Flash_t *mcalib);

#endif /* FLASH_STORAGE_H */

