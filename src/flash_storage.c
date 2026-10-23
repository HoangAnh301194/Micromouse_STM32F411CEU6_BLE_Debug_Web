#include "flash_storage.h"
#include "flash_record_format.h"
#include "control.h"
#include "motor.h"
#include "stm32f4xx.h"
#include "system_timer.h"
#include "uart.h"
#include <stddef.h>
#include <string.h>

/* Append-only v1 records. One reserved 1 KiB legacy region followed by
 * 127 individually programmable 1 KiB slots per 128 KiB sector.
 * Program the final commit word ONLY after data+CRC have been verified.
 * Never erase a sector as part of a normal Save operation.
 * Note: firmware image MUST be linked below 0x08040000. */
#define REC_MAGIC FLASH_REC_MAGIC
#define REC_VERSION FLASH_REC_VERSION
#define REC_MAP 1U
#define REC_IR 2U
#define REC_MPU 3U
#define REC_COMMIT FLASH_REC_COMMIT
#define REC_HEADER_BYTES 20U
#define REC_COMMIT_OFFSET (FLASH_RECORD_SLOT_BYTES - 4U)
#define REC_SLOT_COUNT (FLASH_DATA_SECTOR_BYTES / FLASH_RECORD_SLOT_BYTES)
#define FLASH_KEY1 0x45670123UL
#define FLASH_KEY2 0xCDEF89ABUL
#define FLASH_PSIZE_WORD (2UL << 8)
#define FLASH_ERR_MASK (FLASH_SR_PGSERR | FLASH_SR_PGPERR | FLASH_SR_PGAERR | FLASH_SR_WRPERR)
#define FLASH_WAIT_US 3000000UL

static uint32_t Flash_SectorBase(uint32_t type)
{
  return (type == REC_MAP) ? FLASH_STORAGE_ADDR : FLASH_IR_CALIB_ADDR;
}

static uint32_t Flash_SlotAddr(uint32_t base, uint32_t slot)
{
  return base + slot * (uint32_t)FLASH_RECORD_SLOT_BYTES;
}

static uint8_t Flash_IsErased(uint32_t address)
{
  uint32_t i;
  for (i = 0; i < FLASH_RECORD_SLOT_BYTES; i += 4U)
    if (*(const volatile uint32_t *)(address + i) != 0xFFFFFFFFUL) return 0U;
  return 1U;
}

static uint8_t Flash_Valid(uint32_t address, uint32_t expected_type)
{
  const Flash_RecordHeader_t *h = (const Flash_RecordHeader_t *)address;
  uint32_t type = h->version_type & 0xFFFFU;
  if (*(const volatile uint32_t *)(address + REC_COMMIT_OFFSET) != REC_COMMIT) return 0U;
  if (h->magic != REC_MAGIC || (h->version_type >> 16U) != REC_VERSION ||
      h->sequence == 0U || h->length == 0U ||
      h->length > FLASH_RECORD_PAYLOAD_MAX) return 0U;
  if (type != REC_MAP && type != REC_IR && type != REC_MPU) return 0U;
  if (expected_type != 0U && type != expected_type) return 0U;
  return h->crc == Flash_RecordCrc32(h, (const void *)(address + REC_HEADER_BYTES));
}

static Flash_Status_t Flash_FindLatest(uint32_t type, uint32_t *location,
                                       uint32_t *highest_sequence)
{
  uint32_t base = Flash_SectorBase(type), i;
  uint32_t best = 0U, seq = 0U;
  for (i = 1U; i < REC_SLOT_COUNT; ++i) {
    uint32_t addr = Flash_SlotAddr(base, i);
    if (Flash_Valid(addr, type)) {
      const Flash_RecordHeader_t *h = (const Flash_RecordHeader_t *)addr;
      if (best == 0U || (int32_t)(h->sequence - seq) > 0) {
        best = addr;
        seq = h->sequence;
      }
    }
  }
  if (location) *location = best;
  if (highest_sequence) *highest_sequence = seq;
  return best ? FLASH_OK : FLASH_ERR_NOT_FOUND;
}

static uint32_t Flash_NextSequence(uint32_t base)
{
  uint32_t i, next = 1U;
  for (i = 1U; i < REC_SLOT_COUNT; ++i) {
    uint32_t addr = Flash_SlotAddr(base, i);
    if (Flash_Valid(addr, 0U)) {
      const Flash_RecordHeader_t *h = (const Flash_RecordHeader_t *)addr;
      if (h->sequence >= next && h->sequence != 0xFFFFFFFFUL)
        next = h->sequence + 1U;
    }
  }
  return next;
}

static Flash_Status_t Flash_WaitReady(void)
{
  uint32_t start;
  if ((RCC->APB1ENR & RCC_APB1ENR_TIM5EN) == 0U ||
      (TIM5->CR1 & TIM_CR1_CEN) == 0U) return FLASH_ERR_NOT_READY;
  start = micros();
  while (FLASH->SR & FLASH_SR_BSY) {
    if ((uint32_t)(micros() - start) >= FLASH_WAIT_US)
      return FLASH_ERR_TIMEOUT;
  }
  return FLASH_OK; /* Preexisting error flags are cleared before programming. */
}

static Flash_Status_t Flash_ProgramWord(uint32_t address, uint32_t word)
{
  Flash_Status_t status = Flash_WaitReady();
  if (status != FLASH_OK) return status;
  /* W1C: clear errors, then enable only 32-bit programming. */
  FLASH->SR = FLASH_ERR_MASK | FLASH_SR_EOP;
  FLASH->CR &= ~(FLASH_CR_PG | FLASH_CR_SER | FLASH_CR_SNB | FLASH_CR_PSIZE);
  FLASH->CR |= FLASH_PSIZE_WORD | FLASH_CR_PG;
  *(volatile uint32_t *)address = word;
  status = Flash_WaitReady();
  FLASH->CR &= ~FLASH_CR_PG; /* CRITICAL: do not leave PG set. */
  if (status != FLASH_OK) return status;
  if (FLASH->SR & FLASH_ERR_MASK) return FLASH_ERR_CONTROLLER;
  return (*(const volatile uint32_t *)address == word) ? FLASH_OK : FLASH_ERR_VERIFY;
}

static Flash_Status_t Flash_WriteBytes(uint32_t address, const void *src, uint32_t length)
{
  const uint8_t *p = (const uint8_t *)src;
  uint32_t i;
  for (i = 0U; i < length; i += 4U) {
    uint32_t word = 0xFFFFFFFFUL;
    uint32_t n = length - i;
    Flash_Status_t status;
    if (n > 4U) n = 4U;
    memcpy(&word, p + i, n);
    status = Flash_ProgramWord(address + i, word);
    if (status != FLASH_OK) return status;
  }
  return FLASH_OK;
}

static Flash_Status_t Flash_Append(uint32_t type, const void *data, uint32_t length)
{
  Flash_RecordHeader_t h;
  uint32_t base, addr = 0U, i;
  Flash_Status_t status;
  if (!data || length == 0U || length > FLASH_RECORD_PAYLOAD_MAX)
    return FLASH_ERR_ARG;
  if (Control_IsRunning() || !Motor_OutputsLocked())
    return FLASH_ERR_UNSAFE_STATE;
  if ((RCC->APB1ENR & RCC_APB1ENR_TIM5EN) == 0U ||
      (TIM5->CR1 & TIM_CR1_CEN) == 0U)
    return FLASH_ERR_NOT_READY;
  base = Flash_SectorBase(type);
  for (i = 1U; i < REC_SLOT_COUNT; ++i) {
    uint32_t candidate = Flash_SlotAddr(base, i);
    if (Flash_IsErased(candidate)) { addr = candidate; break; }
  }
  if (addr == 0U) return FLASH_ERR_FULL;

  h.magic = REC_MAGIC;
  h.version_type = (REC_VERSION << 16U) | type;
  h.sequence = Flash_NextSequence(base);
  h.length = length;
  h.crc = Flash_RecordCrc32(&h, data);

  if (FLASH->CR & FLASH_CR_LOCK) {
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
  }
  if (FLASH->CR & FLASH_CR_LOCK) return FLASH_ERR_CONTROLLER;

  status = Flash_WriteBytes(addr, &h, sizeof(h));
  if (status == FLASH_OK)
    status = Flash_WriteBytes(addr + REC_HEADER_BYTES, data, length);
  if (status == FLASH_OK && memcmp((const void *)(addr + REC_HEADER_BYTES), data, length))
    status = FLASH_ERR_VERIFY;
  if (status == FLASH_OK && h.crc != Flash_RecordCrc32((const Flash_RecordHeader_t *)addr,
                                     (const void *)(addr + REC_HEADER_BYTES)))
    status = FLASH_ERR_VERIFY;

  /* Commit marker is written only after all bytes and CRC read-back pass. */
  if (status == FLASH_OK)
    status = Flash_ProgramWord(addr + REC_COMMIT_OFFSET, REC_COMMIT);
  FLASH->CR &= ~FLASH_CR_PG;
  FLASH->CR |= FLASH_CR_LOCK;
  if (status == FLASH_OK && !Flash_Valid(addr, type))
    status = FLASH_ERR_VERIFY;
  return status;
}

static Flash_Status_t Flash_Load(uint32_t type, void *data,
                                 uint32_t capacity, uint32_t *actual)
{
  uint32_t addr;
  const Flash_RecordHeader_t *h;
  Flash_Status_t status;
  if (!data || capacity == 0U) return FLASH_ERR_ARG;
  status = Flash_FindLatest(type, &addr, 0);
  if (status != FLASH_OK) return status;
  h = (const Flash_RecordHeader_t *)addr;
  if (h->length > capacity) return FLASH_ERR_FORMAT;
  memcpy(data, (const void *)(addr + REC_HEADER_BYTES), h->length);
  if (actual) *actual = h->length;
  return FLASH_OK;
}

Flash_Status_t Flash_SaveMapData(const void *data, uint16_t length)
{
  return Flash_Append(REC_MAP, data, length);
}

Flash_Status_t Flash_LoadMapData(void *data, uint16_t capacity, uint16_t *actual_length)
{
  uint32_t actual = 0U;
  Flash_Status_t status = Flash_Load(REC_MAP, data, capacity, &actual);
  if (status == FLASH_OK && actual_length) *actual_length = (uint16_t)actual;
  return status;
}

Flash_Status_t Flash_SaveMazeChecked(const Maze_Persistent_t *pm)
{
  Maze_Persistent_t copy;
  if (!pm) return FLASH_ERR_ARG;
  copy = *pm;
  copy.is_valid = MAZE_PERSISTENT_MAGIC;
  return Flash_Append(REC_MAP, &copy, sizeof(copy));
}

Flash_Status_t Flash_LoadMazeChecked(Maze_Persistent_t *pm)
{
  Maze_Persistent_t copy;
  uint32_t actual = 0U;
  Flash_Status_t status;
  if (!pm) return FLASH_ERR_ARG;
  status = Flash_Load(REC_MAP, &copy, sizeof(copy), &actual);
  if (status == FLASH_OK)
    return (actual == sizeof(copy) && copy.is_valid == MAZE_PERSISTENT_MAGIC)
        ? (*pm = copy, FLASH_OK) : FLASH_ERR_FORMAT;
  if (status != FLASH_ERR_NOT_FOUND) return status;

  /* Read old main-branch map only if no new-format record exists. The
   * 1 KiB legacy zone is never erased/programmed by Flash_Append. */
  memcpy(&copy, (const void *)FLASH_STORAGE_ADDR, sizeof(copy));
  if (copy.is_valid != MAZE_PERSISTENT_MAGIC) return FLASH_ERR_NOT_FOUND;
  *pm = copy;
  return FLASH_OK; /* Legacy: magic only; no integrity guarantee. */
}

Flash_Status_t Flash_SaveIRCalibChecked(const IR_Simple_Calib_t *calib)
{
  IR_Simple_Calib_t copy;
  if (!calib) return FLASH_ERR_ARG;
  copy = *calib;
  copy.is_calibrated = IR_CALIB_MAGIC;
  return Flash_Append(REC_IR, &copy, sizeof(copy));
}

Flash_Status_t Flash_LoadIRCalibChecked(IR_Simple_Calib_t *calib)
{
  IR_Simple_Calib_t copy;
  uint32_t actual = 0U;
  Flash_Status_t status;
  if (!calib) return FLASH_ERR_ARG;
  status = Flash_Load(REC_IR, &copy, sizeof(copy), &actual);
  if (status == FLASH_OK)
    return (actual == sizeof(copy) && copy.is_calibrated == IR_CALIB_MAGIC)
        ? (*calib = copy, FLASH_OK) : FLASH_ERR_FORMAT;
  if (status != FLASH_ERR_NOT_FOUND) return status;
  memcpy(&copy, (const void *)FLASH_IR_CALIB_ADDR, sizeof(copy));
  if (copy.is_calibrated != IR_CALIB_MAGIC) return FLASH_ERR_NOT_FOUND;
  *calib = copy;
  return FLASH_OK;
}

Flash_Status_t Flash_SaveMPUCalibChecked(const MPU_Calib_Flash_t *mcalib)
{
  MPU_Calib_Flash_t copy;
  if (!mcalib) return FLASH_ERR_ARG;
  copy = *mcalib;
  copy.is_valid = MPU_CALIB_MAGIC;
  return Flash_Append(REC_MPU, &copy, sizeof(copy));
}

Flash_Status_t Flash_LoadMPUCalibChecked(MPU_Calib_Flash_t *mcalib)
{
  MPU_Calib_Flash_t copy;
  uint32_t actual = 0U;
  Flash_Status_t status;
  if (!mcalib) return FLASH_ERR_ARG;
  status = Flash_Load(REC_MPU, &copy, sizeof(copy), &actual);
  if (status == FLASH_OK)
    return (actual == sizeof(copy) && copy.is_valid == MPU_CALIB_MAGIC)
        ? (*mcalib = copy, FLASH_OK) : FLASH_ERR_FORMAT;
  if (status != FLASH_ERR_NOT_FOUND) return status;
  memcpy(&copy, (const void *)FLASH_MPU_CALIB_ADDR, sizeof(copy));
  if (copy.is_valid != MPU_CALIB_MAGIC) return FLASH_ERR_NOT_FOUND;
  *mcalib = copy;
  return FLASH_OK;
}

static void Flash_Report(const char *name, Flash_Status_t status)
{
  UART_Printf("FLASH,%s,status=%u\r\n", name, (unsigned)status);
}

void Flash_SaveMaze(Maze_Persistent_t *pm) {
  if (pm) pm->is_valid = MAZE_PERSISTENT_MAGIC; /* Preserve main API side effect. */
  Flash_Report("SAVE_MAP", Flash_SaveMazeChecked(pm));
}
void Flash_LoadMaze(Maze_Persistent_t *pm) {
  Flash_Report("LOAD_MAP", Flash_LoadMazeChecked(pm));
}
void Flash_SaveIRCalib(IR_Simple_Calib_t *calib) {
  if (calib) calib->is_calibrated = IR_CALIB_MAGIC;
  Flash_Report("SAVE_IR", Flash_SaveIRCalibChecked(calib));
}
void Flash_LoadIRCalib(IR_Simple_Calib_t *calib) {
  Flash_Report("LOAD_IR", Flash_LoadIRCalibChecked(calib));
}
void Flash_SaveMPUCalib(MPU_Calib_Flash_t *mcalib) {
  if (mcalib) mcalib->is_valid = MPU_CALIB_MAGIC;
  Flash_Report("SAVE_MPU", Flash_SaveMPUCalibChecked(mcalib));
}
void Flash_LoadMPUCalib(MPU_Calib_Flash_t *mcalib) {
  Flash_Status_t status = Flash_LoadMPUCalibChecked(mcalib);
  if (mcalib && status != FLASH_OK) {
    mcalib->gyro_z_offset = 0.0f;
    mcalib->is_valid = 0U;
  }
  Flash_Report("LOAD_MPU", status);
}
