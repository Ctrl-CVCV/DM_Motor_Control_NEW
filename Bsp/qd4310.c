#include "qd4310.h"

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/*
 * QD4310 CAN 通信格式（参考官方例程 QD4310.c）
 * CAN ID  : 0x400 + motor->id   (标准帧)
 * 数据帧  : 3 字节 [cmd, value_low, value_high]
 */
uint8_t QD4310_SendCommand(QD4310_t *motor, QD4310_Command_t cmd, int16_t value)
{
    uint8_t data[3];
    data[0] = (uint8_t)cmd;
    data[1] = (uint8_t)(value & 0xFF);
    data[2] = (uint8_t)((value >> 8) & 0xFF);

    uint32_t can_id = 0x400 + motor->id;
    return fdcanx_send_data(motor->hfdcan, can_id, data, 3);
}

/* 使能电机（必须先调用） */
uint8_t QD4310_Enable(QD4310_t *motor)
{
    return QD4310_SendCommand(motor, QD4310_CMD_ENABLE, 0);
}

uint8_t QD4310_Disable(QD4310_t *motor)
{
    return QD4310_SendCommand(motor, QD4310_CMD_DISABLE, 0);
}

/* 设置速度，范围 [-1000, 1000] rpm */
uint8_t QD4310_SetSpeed(QD4310_t *motor, float speed)
{
    speed = clampf(speed, QD4310_MIN_SPEED, QD4310_MAX_SPEED);
    int16_t v = (int16_t)(speed / QD4310_MAX_SPEED * 32767);
    return QD4310_SendCommand(motor, QD4310_CMD_SPEED, v);
}

/* 设置角度，范围 [0, 2π] rad */
uint8_t QD4310_SetAngle(QD4310_t *motor, float angle)
{
    angle = clampf(angle, 0.0f, 6.283185307f);
    int16_t v = (int16_t)(angle / 6.283185307f * 65535);
    return QD4310_SendCommand(motor, QD4310_CMD_ANGLE, v);
}

/* 设置电流，范围 [-10, 10] A */
uint8_t QD4310_SetCurrent(QD4310_t *motor, float current)
{
    current = clampf(current, QD4310_MIN_CURRENT, QD4310_MAX_CURRENT);
    int16_t v = (int16_t)(current / QD4310_MAX_CURRENT * 32767);
    return QD4310_SendCommand(motor, QD4310_CMD_CURRENT, v);
}
