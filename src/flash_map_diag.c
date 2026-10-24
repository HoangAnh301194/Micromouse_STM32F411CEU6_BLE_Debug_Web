/* Dedicated opt-in diagnostic: motor locked, no TIM11, MPU or IR startup.
 * Commands via UART1 PA9/PA10; BLE UART2 streams the verified map.
 * Does not erase sectors and never runs automatically in production.
 */
#include "flash_map_diag.h"
#if defined(FLASH_MAP_DIAG)
#include "board.h"
#include "bt_debug.h"
#include "flash_map_protocol.h"
#include "flash_record_format.h"
#include "flash_storage.h"
#include "motor.h"
#include "system_timer.h"
#include "uart.h"
#include <stdio.h>
#include <string.h>

#define FM_CMD_CAP 40U
#define FM_PACE_MS 120UL /* JDY-33: pace 16x16 map over ~2s, no blocking */
#define FM_READY_MS 3500UL
static uint8_t fake_map[MAP_DIAG_MAX_BLOB];
static uint8_t flash_buffer[FLASH_RECORD_PAYLOAD_MAX];
static uint8_t tx_map[MAP_DIAG_MAX_BLOB];
static char command[FM_CMD_CAP];
static uint8_t cmd_len,cmd_bad,tx_active,tx_row;
static uint16_t tx_len;
static uint32_t tx_crc,tx_last,ready_last;
static char tx_source[12];

static void PrintInfo(void)
{
    Flash_MapInfo_t i={0};
    Flash_Status_t st=Flash_GetMapInfo(&i);
    UART_Printf("FM_INFO,status=%u,slot=%lu,seq=%lu,len=%lu,free=%lu,locked=%u,ctrl=OFF\r\n",
                (unsigned)st,(unsigned long)i.active_slot,
                (unsigned long)i.sequence,(unsigned long)i.bytes,
                (unsigned long)i.free_slots,(unsigned)Motor_OutputsLocked());
}

static Flash_Status_t ReadMap(uint8_t *dest,uint16_t *len,uint8_t *legacy)
{
    uint16_t n=0U;
    Flash_Status_t st=Flash_LoadMapData(flash_buffer,sizeof(flash_buffer),&n);
    *legacy=0U;
    if(st==FLASH_OK && MapDiag_Validate(flash_buffer,n)) {
        memcpy(dest,flash_buffer,n);
        *len=n;
        return FLASH_OK;
    }
    if(st==FLASH_OK && n==sizeof(Maze_Persistent_t)) {
        Maze_Persistent_t old;
        uint8_t x,y;
        memcpy(&old,flash_buffer,sizeof(old));
        if(old.is_valid!=MAZE_PERSISTENT_MAGIC) return FLASH_ERR_FORMAT;
        MapDiag_Make(dest,5U);
        for(y=0;y<5U;y++) for(x=0;x<5U;x++)
            dest[MAP_DIAG_HEADER_SIZE+y*5U+x]=old.walls[y][x]&15U;
        *len=MapDiag_Length(5U); *legacy=1U;
        return FLASH_OK;
    }
    if(st!=FLASH_ERR_NOT_FOUND) return st==FLASH_OK?FLASH_ERR_FORMAT:st;
    /* The old main branch stored a bare 5x5 map in sector 7, slot zero. */
    {
        Maze_Persistent_t old;
        uint8_t x,y;
        st=Flash_LoadMazeChecked(&old);
        if(st!=FLASH_OK) return st;
        MapDiag_Make(dest,5U);
        for(y=0;y<5U;y++) for(x=0;x<5U;x++)
            dest[MAP_DIAG_HEADER_SIZE+y*5U+x]=old.walls[y][x]&15U;
        *len=MapDiag_Length(5U); *legacy=1U;
        return FLASH_OK;
    }
}

static void BeginBle(const uint8_t *map,uint16_t len,const char *src)
{
    if(!MapDiag_Validate(map,len)) {
        UART_SendString("FM_ERR,reason=BAD_MAP_PAYLOAD\r\n"); return;
    }
    memcpy(tx_map,map,len);
    tx_len=len; tx_row=0U;
    tx_crc=Flash_DataCrc32(tx_map,tx_len);
    snprintf(tx_source,sizeof(tx_source),"%s",src);
    tx_active=1U;
    tx_last=millis()-FM_PACE_MS;
    BT_Printf("MAPCFG,w=%u,h=%u,fmt=1,len=%u,crc=%08lX,src=%s\r\n",
              (unsigned)tx_map[4],(unsigned)tx_map[5],
              (unsigned)tx_len,(unsigned long)tx_crc,tx_source);
    UART_Printf("FM_CFG,w=%u,h=%u,len=%u,crc=%08lX,src=%s\r\n",
                (unsigned)tx_map[4],(unsigned)tx_map[5],
                (unsigned)tx_len,(unsigned long)tx_crc,tx_source);
}

static void ServiceBle(void)
{
    const char digits[]="0123456789ABCDEF";
    char row[MAP_DIAG_MAX_SIZE+1U];
    uint8_t n=tx_map[4],x;
    if(!tx_active || (uint32_t)(millis()-tx_last)<FM_PACE_MS) return;
    tx_last=millis();
    if(tx_row>=n) {
        BT_Printf("MAPEND,crc=%08lX,rows=%u\r\n",
                  (unsigned long)tx_crc,(unsigned)n);
        UART_Printf("FM_DONE,rows=%u,crc=%08lX,src=%s\r\n",
                    (unsigned)n,(unsigned long)tx_crc,tx_source);
        tx_active=0U; return;
    }
    for(x=0U;x<n;x++)
        row[x]=digits[tx_map[MAP_DIAG_HEADER_SIZE+(uint16_t)tx_row*n+x]&15U];
    row[n]='\0';
    BT_Printf("MAPROW,y=%u,hex=%s\r\n",(unsigned)tx_row,row);
    UART_Printf("FM_ROW,y=%u,hex=%s\r\n",(unsigned)tx_row,row);
    tx_row++;
}

static void DoRead(void)
{
    uint16_t n=0U;
    uint8_t legacy=0U;
    Flash_Status_t st=ReadMap(fake_map,&n,&legacy);
    UART_Printf("FM_READ,status=%u,legacy=%u,len=%u,crc=%08lX\r\n",
                (unsigned)st,(unsigned)legacy,(unsigned)(st==FLASH_OK?n:0U),
                (unsigned long)(st==FLASH_OK?Flash_DataCrc32(fake_map,n):0U));
    PrintInfo();
    if(st==FLASH_OK) BeginBle(fake_map,n,legacy?"LEGACY":"FLASH");
    else BT_Printf("MAPERR,reason=NO_SAVED_MAP,status=%u\r\n",(unsigned)st);
}

static void DoPreview(uint8_t size)
{
    MapDiag_Make(fake_map,size);
    UART_Printf("FM_PREVIEW,size=%u,flash_written=0\r\n",(unsigned)size);
    BeginBle(fake_map,MapDiag_Length(size),"RAM");
}

static void DoSave(uint8_t size)
{
    uint16_t n,loaded=0U;
    uint8_t legacy=0U;
    uint32_t start=millis(),crc;
    Flash_Status_t save,read;
    if(!Motor_OutputsLocked()) {
        UART_SendString("FM_SAVE,status=3,reason=MOTOR_NOT_LOCKED\r\n"); return;
    }
    MapDiag_Make(fake_map,size);
    n=MapDiag_Length(size);
    crc=Flash_DataCrc32(fake_map,n);
    UART_Printf("FM_BEGIN,size=%u,len=%u,crc=%08lX\r\n",
                (unsigned)size,(unsigned)n,(unsigned long)crc);
    save=Flash_SaveMapData(fake_map,n);
    UART_Printf("FM_SAVE,status=%u,elapsed_ms=%lu\r\n",
                (unsigned)save,(unsigned long)(millis()-start));
    if(save!=FLASH_OK) { PrintInfo(); return; }
    read=ReadMap(flash_buffer,&loaded,&legacy);
    if(read!=FLASH_OK || loaded!=n || legacy ||
       memcmp(fake_map,flash_buffer,n)!=0) {
        UART_Printf("FM_VERIFY,status=FAIL,load=%u,expect=%u,actual=%u\r\n",
                    (unsigned)read,(unsigned)n,(unsigned)loaded);
        PrintInfo(); return;
    }
    UART_Printf("FM_VERIFY,status=PASS,size=%u,len=%u,crc=%08lX,elapsed_ms=%lu\r\n",
                (unsigned)size,(unsigned)n,(unsigned long)crc,
                (unsigned long)(millis()-start));
    PrintInfo();
    BeginBle(flash_buffer,n,"FLASH");
}

static void Dispatch(void)
{
    command[cmd_len]='\0';
    if(!strcmp(command,"HELP"))
        UART_SendString("FM_HELP,INFO|READ|PREVIEW 16|PREVIEW 5|SAVE 16 CONFIRM|SAVE 5 CONFIRM; append-only; NO ERASE\r\n");
    else if(!strcmp(command,"INFO")) PrintInfo();
    else if(!strcmp(command,"READ")) DoRead();
    else if(!strcmp(command,"PREVIEW 16")) DoPreview(16U);
    else if(!strcmp(command,"PREVIEW 5")) DoPreview(5U);
    else if(!strcmp(command,"SAVE 16 CONFIRM")) DoSave(16U);
    else if(!strcmp(command,"SAVE 5 CONFIRM")) DoSave(5U);
    else UART_Printf("FM_REJECT,reason=UNKNOWN,len=%u\r\n",(unsigned)cmd_len);
}
static void PollSerial(void)
{
    uint8_t c,budget=32U;
    while(budget-- && UART_ReadByte(&c)) {
        if(c=='\r' || c=='\n') {
            if(cmd_bad) UART_SendString("FM_REJECT,reason=BAD_LINE\r\n");
            else if(cmd_len) Dispatch();
            cmd_len=cmd_bad=0U;
        } else if(!cmd_bad) {
            if(c<32U || c>126U || cmd_len>=FM_CMD_CAP-1U) cmd_bad=1U;
            else command[cmd_len++]=(char)c;
        }
    }
}
int FlashMapDiag_Main(void)
{
    Board_Init();
    SystemTimer_Init();
    UART_Init();
    if(Board_GetClockFault()) {
        Board_StatusLed(1U);
        UART_SendString("FM_BOOT,status=CLOCK_FAULT\r\n");
        while(1) { __WFI(); }
    }
    Motor_Init();
    Motor_LockOutputs(); /* Permanently locked until reset. */
    BT_Init();
    UART_SendString("FM_BOOT,profile=flash_map_diag_96m,locked=1,tim11=OFF,ir=OFF,mpu=OFF,erase=DISABLED\r\n");
    PrintInfo();
    ready_last=millis()-FM_READY_MS;
    while(1) {
        PollSerial();
        if(BT_Available()) {
            char c=BT_ReceiveChar();
            if(c=='?') {
                UART_Printf("FM_BLE_REQ,cmd=READ,ms=%lu\r\n",(unsigned long)millis());
                BT_SendString("MAPACK,cmd=READ\r\n");
                DoRead();
            } else if(c=='V') {
                UART_Printf("FM_BLE_REQ,cmd=PREVIEW16,ms=%lu\r\n",(unsigned long)millis());
                BT_SendString("MAPACK,cmd=PREVIEW16\r\n");
                DoPreview(16U);
            } else if(c=='v') {
                UART_Printf("FM_BLE_REQ,cmd=PREVIEW5,ms=%lu\r\n",(unsigned long)millis());
                BT_SendString("MAPACK,cmd=PREVIEW5\r\n");
                DoPreview(5U);
            }
        }
        ServiceBle();
        /* Lightweight heartbeat lets Web detect board resets and a missing
         * map request, even when JDY-33 was connected after boot. */
        if((uint32_t)(millis()-ready_last)>=FM_READY_MS) {
            ready_last=millis();
            BT_Printf("MAPREADY,ms=%lu,locked=1\r\n",(unsigned long)ready_last);
        }
    }
}
#else
typedef int flash_map_diag_disabled_translation_unit;
#endif
