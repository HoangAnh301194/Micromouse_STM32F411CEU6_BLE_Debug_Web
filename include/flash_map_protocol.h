#ifndef FLASH_MAP_PROTOCOL_H
#define FLASH_MAP_PROTOCOL_H
#include <stdint.h>
#define MAP_DIAG_MAX_SIZE 16U
#define MAP_DIAG_HEADER_SIZE 8U
#define MAP_DIAG_MAX_BLOB (MAP_DIAG_HEADER_SIZE + MAP_DIAG_MAX_SIZE * MAP_DIAG_MAX_SIZE)
/* "MAP1" (4 bytes), width, height, flags=0, reserved=0, then row-major
 * wall nibbles N=1 E=2 S=4 W=8; row 0 is bottom (Y=0). */
uint16_t MapDiag_Length(uint8_t size);
void MapDiag_Make(uint8_t *blob, uint8_t size);
uint8_t MapDiag_Validate(const uint8_t *blob, uint16_t length);
#endif
