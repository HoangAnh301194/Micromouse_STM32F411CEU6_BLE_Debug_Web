#ifndef MOCK_INTEGRATION_SWEEP_H
#define MOCK_INTEGRATION_SWEEP_H
#include <stdint.h>
typedef struct { uint8_t calibrated,cal_state; uint32_t timeout_reads; } MPU6050_Handle_t;
typedef struct { uint8_t valid; uint32_t completed_us; } MPU6050_Snapshot_t;
uint8_t MPU6050_GetSnapshot(const MPU6050_Handle_t *, MPU6050_Snapshot_t *);
typedef enum { SWEEP_DISARMED=0,SWEEP_DRIVE=1,SWEEP_COAST=2,SWEEP_DONE=3,SWEEP_FAULT=4 } SweepState;
typedef enum { SWEEP_OK=0,SWEEP_STOP_REQUEST=1,SWEEP_IMU_STALE=2,
SWEEP_TIM11_DEADLINE=3,SWEEP_I2C_TIMEOUT=4,SWEEP_MOTOR_WATCHDOG=5,
SWEEP_SESSION_TIMEOUT=6,SWEEP_CLOCK_FAULT=7,SWEEP_UART_LINK_LOST=9 } SweepFault;
typedef struct { uint8_t state,fault,group,step,armed_once,motor_enabled;
uint16_t duty_permille;uint32_t elapsed_ms,stage_id,deadline_ms; } SweepSnapshot;
void Sweep_Init(void);
uint8_t Sweep_MaxPwmPercent(void);
uint8_t Sweep_StageTotal(void);
uint8_t Sweep_SelectMode(uint8_t motor_on);
uint8_t Sweep_Arm(uint32_t now_ms,const MPU6050_Handle_t *,uint8_t host_authorized);
void Sweep_Heartbeat(uint32_t);
void Sweep_Update(uint32_t,const MPU6050_Handle_t *);
void Sweep_Abort(uint8_t);
void Sweep_ImuFaultFromISR(void);
void Sweep_GetSnapshot(SweepSnapshot *);
uint8_t Sweep_IsActive(void);
#endif
