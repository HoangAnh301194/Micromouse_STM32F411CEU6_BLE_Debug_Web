#include "flash_map_protocol.h"
#include <string.h>
uint16_t MapDiag_Length(uint8_t n)
{
    return (uint16_t)(MAP_DIAG_HEADER_SIZE + (uint16_t)n*n);
}
void MapDiag_Make(uint8_t *b, uint8_t n)
{
    uint8_t x,y;
    if (!b || n < 2U || n > MAP_DIAG_MAX_SIZE) return;
    memcpy(b,"MAP1",4);
    b[4]=n; b[5]=n; b[6]=0U; b[7]=0U;
    for (y=0U;y<n;y++) for(x=0U;x<n;x++) {
        uint8_t w=0U;
        if(y==n-1U || (3U*x+5U*(y+1U))%11U==0U) w|=1U;
        if(x==n-1U || (7U*(x+1U)+2U*y)%13U==0U) w|=2U;
        if(y==0U || (3U*x+5U*y)%11U==0U) w|=4U;
        if(x==0U || (7U*x+2U*y)%13U==0U) w|=8U;
        b[MAP_DIAG_HEADER_SIZE+(uint16_t)y*n+x]=w;
    }
}
uint8_t MapDiag_Validate(const uint8_t *b, uint16_t len)
{
    uint8_t n,x,y;
    if(!b || len<MAP_DIAG_HEADER_SIZE || memcmp(b,"MAP1",4) ||
       b[6]!=0U || b[7]!=0U) return 0U;
    n=b[4];
    if(n<2U || n>MAP_DIAG_MAX_SIZE || b[5]!=n || len!=MapDiag_Length(n))
        return 0U;
    for(y=0U;y<n;y++) for(x=0U;x<n;x++) {
        uint8_t w=b[MAP_DIAG_HEADER_SIZE+(uint16_t)y*n+x];
        if(w>15U) return 0U;
    }
    return 1U;
}
