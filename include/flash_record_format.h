#ifndef FLASH_RECORD_FORMAT_H
#define FLASH_RECORD_FORMAT_H
#include <stdint.h>

#define FLASH_REC_MAGIC 0x4D4D4631UL /* MMF1 */
#define FLASH_REC_VERSION 1U
#define FLASH_REC_COMMIT 0xA53C69F0UL

typedef struct {
  uint32_t magic;
  uint32_t version_type;
  uint32_t sequence;
  uint32_t length;
  uint32_t crc;
} Flash_RecordHeader_t;

/* CRC32/IEEE reflected, initial/final XOR FFFFFFFF; protects the first
 * 16 bytes of the header (all except crc) followed by payload bytes.
 * Pure software (no FLASH/I2C/timer access), host-testable. */
uint32_t Flash_RecordCrc32(const Flash_RecordHeader_t *h, const void *payload);
#endif
