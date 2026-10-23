#include "flash_record_format.h"
#include <stddef.h>

typedef char Flash_Header_Size_20[(sizeof(Flash_RecordHeader_t) == 20U) ? 1 : -1];

static uint32_t Flash_CrcFeed(uint32_t crc, uint8_t byte)
{
  uint32_t bit;
  crc ^= byte;
  for (bit = 0U; bit < 8U; ++bit)
    crc = (crc >> 1U) ^ ((crc & 1U) ? 0xEDB88320UL : 0U);
  return crc;
}

uint32_t Flash_RecordCrc32(const Flash_RecordHeader_t *h, const void *payload)
{
  uint32_t crc = 0xFFFFFFFFUL;
  const uint8_t *head = (const uint8_t *)h;
  const uint8_t *data = (const uint8_t *)payload;
  uint32_t i;
  for (i = 0U; i < offsetof(Flash_RecordHeader_t, crc); ++i)
    crc = Flash_CrcFeed(crc, head[i]);
  for (i = 0U; i < h->length; ++i)
    crc = Flash_CrcFeed(crc, data[i]);
  return ~crc;
}

uint32_t Flash_DataCrc32(const void *data, uint32_t length) {
    const uint8_t *p = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t i;
    if (!p && length) return 0U;
    for (i = 0U; i < length; ++i) crc = Flash_CrcFeed(crc, p[i]);
    return ~crc;
}
