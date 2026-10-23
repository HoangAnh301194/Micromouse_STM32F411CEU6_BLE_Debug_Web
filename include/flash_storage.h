#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H
#include <stdint.h>
#include "flash_legacy_types.h"

/* F411CEUx 512 KiB flash; Sectors 0..5 = app (linker 256 KiB),
 * 6 = calibration data, 7 = maze data. The first 1024 bytes of
 * each data sector are reserved for the original main-branch layout. */
#define FLASH_IR_CALIB_ADDR  0x08040000UL
#define FLASH_MPU_CALIB_ADDR (FLASH_IR_CALIB_ADDR + 256UL)
#define FLASH_STORAGE_ADDR   0x08060000UL
#define FLASH_DATA_SECTOR_BYTES (128UL * 1024UL)
#define FLASH_RECORD_SLOT_BYTES 1024U
#define FLASH_RECORD_PAYLOAD_MAX (FLASH_RECORD_SLOT_BYTES - 24U)
#define IR_CALIB_MAGIC 0xCAU
#define MPU_CALIB_MAGIC 0xEBU

typedef struct {
  float gyro_z_offset;
  uint8_t is_valid;
} MPU_Calib_Flash_t;

typedef enum {
  FLASH_OK = 0,
  FLASH_ERR_ARG,
  FLASH_ERR_NOT_FOUND,
  FLASH_ERR_UNSAFE_STATE,
  FLASH_ERR_NOT_READY,
  FLASH_ERR_FULL,
  FLASH_ERR_TIMEOUT,
  FLASH_ERR_CONTROLLER,
  FLASH_ERR_VERIFY,
  FLASH_ERR_FORMAT
} Flash_Status_t;


typedef struct {
    uint32_t active_slot;
    uint32_t sequence;
    uint32_t bytes;
    uint32_t free_slots;
} Flash_MapInfo_t;
Flash_Status_t Flash_GetMapInfo(Flash_MapInfo_t *out);

/* Versioned binary map payload, independent of 5x5 legacy layout.
 * The entire payload is verified by CRC32 before returning.
 * Calling Save requires Control_Stop(), Motor_LockOutputs(), then IR_Stop().
 * No automatic sector erase: FLASH_ERR_FULL requires explicit backup/maintenance. */
Flash_Status_t Flash_SaveMapData(const void *data, uint16_t length);
Flash_Status_t Flash_LoadMapData(void *data, uint16_t capacity,
                                uint16_t *actual_length);

/* Compatibility with main's 5x5 persistent maze and calibration structs.
 * New writes use versioned CRC records; loads fall back to legacy bytes only
 * when no new-format record exists for that type. */
Flash_Status_t Flash_SaveMazeChecked(const Maze_Persistent_t *pm);
Flash_Status_t Flash_LoadMazeChecked(Maze_Persistent_t *pm);
Flash_Status_t Flash_SaveIRCalibChecked(const IR_Simple_Calib_t *calib);
Flash_Status_t Flash_LoadIRCalibChecked(IR_Simple_Calib_t *calib);
Flash_Status_t Flash_SaveMPUCalibChecked(const MPU_Calib_Flash_t *mcalib);
Flash_Status_t Flash_LoadMPUCalibChecked(MPU_Calib_Flash_t *mcalib);

/* Old void signatures are kept for source compatibility. Prefer checked
 * APIs for any new integration, and handle every non-OK result. */
void Flash_SaveMaze(Maze_Persistent_t *pm);
void Flash_LoadMaze(Maze_Persistent_t *pm);
void Flash_SaveIRCalib(IR_Simple_Calib_t *calib);
void Flash_LoadIRCalib(IR_Simple_Calib_t *calib);
void Flash_SaveMPUCalib(MPU_Calib_Flash_t *mcalib);
void Flash_LoadMPUCalib(MPU_Calib_Flash_t *mcalib);
#endif
