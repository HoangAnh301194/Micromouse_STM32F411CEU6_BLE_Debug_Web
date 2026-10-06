#include "flash_storage.h"
#include "stm32f4xx.h"
#include "uart.h"
#include <stdio.h>
#include <string.h>

/* Helper to get Flash sector from address if not defined */
#ifndef FLASH_SECTOR_7
#define FLASH_SECTOR_7 7
#endif
#ifndef FLASH_SECTOR_6
#define FLASH_SECTOR_6 6
#endif

/* Flash key values for unlocking */
#define FLASH_KEY1  0x45670123UL
#define FLASH_KEY2  0xCDEF89ABUL

/* Flash program size: x32 (2 << 8) for 3.3V operation */
#define FLASH_PSIZE_WORD  (2UL << 8)

/* ========================================================================== */
/* Low-level Flash helpers (CMSIS register access)                            */
/* ========================================================================== */

static void Flash_Unlock(void) {
  if (FLASH->CR & FLASH_CR_LOCK) {
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
  }
}

static void Flash_Lock(void) {
  FLASH->CR |= FLASH_CR_LOCK;
}

static void Flash_WaitBusy(void) {
  while (FLASH->SR & FLASH_SR_BSY);
}

/**
 * @brief Erase a flash sector (0-7)
 * @return 0 on success, non-zero on error
 */
static int Flash_EraseSector(uint32_t sector) {
  Flash_WaitBusy();

  /* Clear any previous errors */
  FLASH->SR = (FLASH_SR_PGSERR | FLASH_SR_PGPERR | FLASH_SR_PGAERR |
               FLASH_SR_WRPERR | FLASH_SR_SOP | FLASH_SR_EOP);

  /* Set sector erase, sector number, voltage range 3 (x32) */
  FLASH->CR &= ~(FLASH_CR_SNB | FLASH_CR_PSIZE);
  FLASH->CR |= (sector << 3) | FLASH_PSIZE_WORD | FLASH_CR_SER;

  /* Start erase */
  FLASH->CR |= FLASH_CR_STRT;

  Flash_WaitBusy();

  /* Clear SER bit */
  FLASH->CR &= ~FLASH_CR_SER;

  /* Check for errors */
  if (FLASH->SR & (FLASH_SR_PGSERR | FLASH_SR_PGPERR | FLASH_SR_PGAERR |
                   FLASH_SR_WRPERR)) {
    return -1;
  }
  return 0;
}

/**
 * @brief Program a 32-bit word to flash
 * @return 0 on success, non-zero on error
 */
static int Flash_ProgramWord(uint32_t address, uint32_t data) {
  Flash_WaitBusy();

  /* Clear any previous errors */
  FLASH->SR = (FLASH_SR_PGSERR | FLASH_SR_PGPERR | FLASH_SR_PGAERR |
               FLASH_SR_WRPERR | FLASH_SR_SOP | FLASH_SR_EOP);

  /* Set program mode, word size */
  FLASH->CR &= ~FLASH_CR_PSIZE;
  FLASH->CR |= FLASH_PSIZE_WORD | FLASH_CR_PG;

  /* Write data */
  *(volatile uint32_t *)address = data;

  Flash_WaitBusy();

  /* Clear PG bit */
  FLASH->CR &= ~FLASH_CR_PG;

  /* Check for errors */
  if (FLASH->SR & (FLASH_SR_PGSERR | FLASH_SR_PGPERR | FLASH_SR_PGAERR |
                   FLASH_SR_WRPERR)) {
    return -1;
  }
  return 0;
}

/**
 * @brief Write multiple words to flash
 * @return 0 on success, non-zero on error
 */
static int Flash_WriteWords(uint32_t addr, const void *data,
                            uint32_t byte_count) {
  uint32_t sizeWords = (byte_count + 3) / 4;
  const uint32_t *src = (const uint32_t *)data;
  uint32_t i;

  for (i = 0; i < sizeWords; i++) {
    if (Flash_ProgramWord(addr + (i * 4), src[i]) != 0)
      return -1;
  }
  return 0;
}

/* ========================================================================== */
/* MAZE — Sector 7                                                            */
/* ========================================================================== */

void Flash_SaveMaze(Maze_Persistent_t *pm) {
  uint32_t *dataPtr = (uint32_t *)pm;
  uint32_t sizeWords =
      (sizeof(Maze_Persistent_t) + 3) / 4; /* Ceiling division */
  uint32_t i;
  char buf[64];

  /* Ensure magic is set before saving */
  pm->is_valid = MAZE_PERSISTENT_MAGIC;

  UART_SendString("Flash: Saving persistent maze map...\r\n");

  Flash_Unlock();

  /* Erase Sector 7 (128KB on F411) */
  if (Flash_EraseSector(FLASH_SECTOR_7) != 0) {
    UART_SendString("Flash: Erase FAILED\r\n");
    Flash_Lock();
    return;
  }

  /* Write data word by word */
  for (i = 0; i < sizeWords; i++) {
    if (Flash_ProgramWord(FLASH_STORAGE_ADDR + (i * 4), dataPtr[i]) != 0) {
      sprintf(buf, "Flash: Write FAILED at word %lu\r\n", i);
      UART_SendString(buf);
      Flash_Lock();
      return;
    }
  }

  Flash_Lock();
  UART_SendString("Flash: Maze Map Save SUCCESS\r\n");
}

void Flash_LoadMaze(Maze_Persistent_t *pm) {
  Maze_Persistent_t temp;

  /* Read from Flash into temp first to verify magic byte */
  memcpy(&temp, (void *)FLASH_STORAGE_ADDR, sizeof(Maze_Persistent_t));

  if (temp.is_valid == MAZE_PERSISTENT_MAGIC) {
    memcpy(pm, &temp, sizeof(Maze_Persistent_t));
    UART_SendString("Flash: Persistent Maze Map LOADED\r\n");
  } else {
    UART_SendString(
        "Flash: No valid maze map found in Flash (Magic mismatch)\r\n");
  }
}

/* ========================================================================== */
/* SECTOR 6 SHARED HELPER — IR + MPU calib coexist in same sector             */
/* Erasing Sector 6 wipes both, so we must read-backup-erase-write-both.      */
/* ========================================================================== */

static int Flash_EraseSector6(void) {
  return Flash_EraseSector(FLASH_SECTOR_6);
}

/* ========================================================================== */
/* IR CALIBRATION — Sector 6 @ offset 0                                       */
/* ========================================================================== */

void Flash_SaveIRCalib(IR_Simple_Calib_t *calib) {
  MPU_Calib_Flash_t mpu_backup;
  char buf[64];

  /* Backup MPU calib before erasing sector */
  memcpy(&mpu_backup, (void *)FLASH_MPU_CALIB_ADDR, sizeof(MPU_Calib_Flash_t));

  /* Mark as valid before saving */
  calib->is_calibrated = IR_CALIB_MAGIC;

  UART_SendString("Flash: Saving IR calibration...\r\n");

  Flash_Unlock();

  if (Flash_EraseSector6() != 0) {
    UART_SendString("Flash: IR Erase FAILED\r\n");
    Flash_Lock();
    return;
  }

  /* Write IR calib */
  if (Flash_WriteWords(FLASH_IR_CALIB_ADDR, calib,
                       sizeof(IR_Simple_Calib_t)) != 0) {
    UART_SendString("Flash: IR Write FAILED\r\n");
    Flash_Lock();
    return;
  }

  /* Restore MPU calib if it was valid */
  if (mpu_backup.is_valid == MPU_CALIB_MAGIC) {
    Flash_WriteWords(FLASH_MPU_CALIB_ADDR, &mpu_backup,
                     sizeof(MPU_Calib_Flash_t));
  }

  Flash_Lock();

  UART_SendString("Flash: IR Calib Save OK\r\n");
  sprintf(buf, "  L90=%d L45=%d L0=%d\r\n", calib->wall_threshold_left,
          calib->wall_threshold_l45, calib->wall_threshold_front_left);
  UART_SendString(buf);
  sprintf(buf, "  R0=%d R45=%d R90=%d\r\n", calib->wall_threshold_front_right,
          calib->wall_threshold_r45, calib->wall_threshold_right);
  UART_SendString(buf);
  sprintf(buf, "  CenterL90=%d CenterR90=%d Fsum=%lu\r\n", calib->center_l90,
          calib->center_r90, calib->front_fsum_target);
  UART_SendString(buf);
  sprintf(buf, "  CenterL0=%d CenterR0=%d\r\n", calib->center_l0,
          calib->center_r0);
  UART_SendString(buf);
}

void Flash_LoadIRCalib(IR_Simple_Calib_t *calib) {
  IR_Simple_Calib_t temp;
  char buf[64];

  memcpy(&temp, (void *)FLASH_IR_CALIB_ADDR, sizeof(IR_Simple_Calib_t));

  if (temp.is_calibrated == IR_CALIB_MAGIC) {
    memcpy(calib, &temp, sizeof(IR_Simple_Calib_t));
    UART_SendString("Flash: IR Calib LOADED\r\n");
    sprintf(buf, "  L90=%d L45=%d L0=%d\r\n", calib->wall_threshold_left,
            calib->wall_threshold_l45, calib->wall_threshold_front_left);
    UART_SendString(buf);
    sprintf(buf, "  R0=%d R45=%d R90=%d\r\n", calib->wall_threshold_front_right,
            calib->wall_threshold_r45, calib->wall_threshold_right);
    UART_SendString(buf);
    sprintf(buf, "  CenterL90=%d CenterR90=%d Fsum=%lu\r\n", calib->center_l90,
            calib->center_r90, calib->front_fsum_target);
    UART_SendString(buf);
    sprintf(buf, "  CenterL0=%d CenterR0=%d\r\n", calib->center_l0,
            calib->center_r0);
    UART_SendString(buf);
  } else {
    UART_SendString("Flash: No valid IR calib found (using defaults)\r\n");
  }
}

/* ========================================================================== */
/* MPU CALIBRATION — Sector 6 @ offset 256                                    */
/* ========================================================================== */

void Flash_SaveMPUCalib(MPU_Calib_Flash_t *mcalib) {
  IR_Simple_Calib_t ir_backup;
  char buf[64];

  /* Backup IR calib before erasing sector */
  memcpy(&ir_backup, (void *)FLASH_IR_CALIB_ADDR, sizeof(IR_Simple_Calib_t));

  mcalib->is_valid = MPU_CALIB_MAGIC;

  UART_SendString("Flash: Saving MPU calibration...\r\n");

  Flash_Unlock();

  if (Flash_EraseSector6() != 0) {
    UART_SendString("Flash: MPU Erase FAILED\r\n");
    Flash_Lock();
    return;
  }

  /* Restore IR calib if it was valid */
  if (ir_backup.is_calibrated == IR_CALIB_MAGIC) {
    Flash_WriteWords(FLASH_IR_CALIB_ADDR, &ir_backup,
                     sizeof(IR_Simple_Calib_t));
  }

  /* Write MPU calib */
  if (Flash_WriteWords(FLASH_MPU_CALIB_ADDR, mcalib,
                       sizeof(MPU_Calib_Flash_t)) != 0) {
    UART_SendString("Flash: MPU Write FAILED\r\n");
    Flash_Lock();
    return;
  }

  Flash_Lock();

  sprintf(buf, "Flash: MPU Calib Save OK (offset=%.3f)\r\n",
          mcalib->gyro_z_offset);
  UART_SendString(buf);
}

void Flash_LoadMPUCalib(MPU_Calib_Flash_t *mcalib) {
  MPU_Calib_Flash_t temp;
  char buf[64];

  memcpy(&temp, (void *)FLASH_MPU_CALIB_ADDR, sizeof(MPU_Calib_Flash_t));

  if (temp.is_valid == MPU_CALIB_MAGIC) {
    memcpy(mcalib, &temp, sizeof(MPU_Calib_Flash_t));
    sprintf(buf, "Flash: MPU Calib LOADED (offset=%.3f)\r\n",
            mcalib->gyro_z_offset);
    UART_SendString(buf);
  } else {
    mcalib->is_valid = 0;
    mcalib->gyro_z_offset = 0.0f;
    UART_SendString("Flash: No valid MPU calib found\r\n");
  }
}
