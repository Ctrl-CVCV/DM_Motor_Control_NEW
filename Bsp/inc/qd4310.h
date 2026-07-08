#ifndef _QD4310__H__
#define _QD4310__H__
#include "main.h"
#include "can_bsp.h"

/* ── QD4310 CAN 协议 ──
   CAN ID : 0x400 + motor_id (标准帧)
   数据   : 3 字节 [cmd, val_low, val_high]
   速度   : [-1000, 1000] rpm → int16_t → (speed/1000)*32767
   电流   : [-10, 10] A     → int16_t → (cur/10)*32767
   角度   : [0, 2π] rad     → uint16_t → (ang/2π)*65535
────────────────────────────── */

#define QD4310_MAX_SPEED     1000.0f
#define QD4310_MIN_SPEED    -1000.0f
#define QD4310_MAX_CURRENT    10.0f
#define QD4310_MIN_CURRENT   -10.0f

typedef enum {
    QD4310_CMD_NOP        = 0x00,
    QD4310_CMD_ENABLE     = 0x01,
    QD4310_CMD_DISABLE    = 0x02,
    QD4310_CMD_CURRENT    = 0x03,
    QD4310_CMD_SPEED      = 0x04,
    QD4310_CMD_ANGLE      = 0x05,
    QD4310_CMD_LOW_SPEED  = 0x06,
    QD4310_CMD_STEP_ANGLE = 0x07
} QD4310_Command_t;

typedef struct {
    uint8_t  id;
    FDCAN_HandleTypeDef *hfdcan;
} QD4310_t;

uint8_t QD4310_SendCommand(QD4310_t *motor, QD4310_Command_t cmd, int16_t value);
uint8_t QD4310_Enable(QD4310_t *motor);
uint8_t QD4310_Disable(QD4310_t *motor);
uint8_t QD4310_SetSpeed(QD4310_t *motor, float speed);
uint8_t QD4310_SetAngle(QD4310_t *motor, float angle);
uint8_t QD4310_SetCurrent(QD4310_t *motor, float current);

#endif
