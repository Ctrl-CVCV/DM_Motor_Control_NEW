/**
 * @file        qd4310.c
 * @brief       基于HAL FDCAN库的QD4310电机CAN总线控制库(C语言版)
 * @details     参考QGimbal项目中的QD4310 C++实现,改写为C函数接口
 *              发送帧: 3字节 = [cmd][int16 value 小端序]
 *              反馈帧: 8字节状态数据
 */

#include "qd4310.h"
#include "can_bsp.h"
#include <math.h>
#include <stdint.h>

#ifndef INT16_MAX
#define INT16_MAX   32767
#endif
#ifndef INT16_MIN
#define INT16_MIN   (-32768)
#endif
#ifndef UINT16_MAX
#define UINT16_MAX  65535U
#endif

#define QD_PI           3.14159265358979323846f
#define QD_TWO_PI       6.28318530717958647692f

/*
 * QD4310 CAN 通信格式（参考官方例程 qd4310.c）
 * CAN ID  : 0x400 + motor_id   (标准帧)
 * 数据帧  : 3 字节 [cmd, value_low, value_high]
 * 归一化  : 物理量映射到 int16 满量程 [-32767, 32767] / [0, 65535]
 */

/* ===== 限幅辅助函数 ===== */
static float qd_clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static int16_t qd_clamp_int16(int32_t v)
{
    if (v > INT16_MAX) return INT16_MAX;
    if (v < INT16_MIN) return INT16_MIN;
    return (int16_t)v;
}

/* ===== 电机状态表 ===== */
static qd_motor_state_t qd_motor_states[QD_MAX_MOTORS];

/* ===== 内部发送函数 ===== */
/**
 * @brief 发送3字节命令帧
 * @param hfdcan   FDCAN句柄
 * @param motor_id 电机ID
 * @param cmd      命令字
 * @param value    int16数值(小端序填入帧)
 */
static void qd_send_command(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id,
                            uint8_t cmd, int16_t value)
{
    if (motor_id >= QD_MAX_MOTORS) return;

    /* 3字节: [cmd][value_low][value_high] (参考官方例程) */
    uint8_t data[3];
    data[0] = (uint8_t)cmd;
    data[1] = (uint8_t)((uint16_t)value & 0xFF);
    data[2] = (uint8_t)(((uint16_t)value >> 8) & 0xFF);

    uint32_t can_id = QD_TX_ID(motor_id);
    fdcanx_send_data(hfdcan, can_id, data, 3);
}

/* ===== 控制函数实现 ===== */

void qd_enable(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id)
{
    qd_send_command(hfdcan, motor_id, QD_CMD_ENABLE, 0x0000);
}

void qd_disable(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id)
{
    qd_send_command(hfdcan, motor_id, QD_CMD_DISABLE, 0x0000);
}

void qd_nop(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id)
{
    qd_send_command(hfdcan, motor_id, QD_CMD_NOP, 0x0000);
}

void qd_set_current(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, float current)
{
    current = qd_clampf(current, -10.0f, 10.0f);
    /* current / 10 * INT16_MAX */
    int32_t val = (int32_t)(current / 10.0f * (float)INT16_MAX);
    qd_send_command(hfdcan, motor_id, QD_CMD_CURRENT, qd_clamp_int16(val));
}

void qd_set_speed(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, float speed)
{
    speed = qd_clampf(speed, -1000.0f, 1000.0f);
    /* speed / 1000 * INT16_MAX */
    int32_t val = (int32_t)(speed / 1000.0f * (float)INT16_MAX);
    qd_send_command(hfdcan, motor_id, QD_CMD_SPEED, qd_clamp_int16(val));
}

void qd_set_low_speed(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, float speed)
{
    speed = qd_clampf(speed, -1000.0f, 1000.0f);
    int32_t val = (int32_t)(speed / 1000.0f * (float)INT16_MAX);
    qd_send_command(hfdcan, motor_id, QD_CMD_LOW_SPEED, qd_clamp_int16(val));
}

void qd_set_angle(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, float angle)
{
    angle = qd_clampf(angle, 0.0f, QD_TWO_PI);
    /* angle / 2π * UINT16_MAX, 存入int16(0~65535需用uint16,但协议用int16_t传递) */
    uint32_t uval = (uint32_t)(angle / QD_TWO_PI * (float)UINT16_MAX);
    if (uval > UINT16_MAX) uval = UINT16_MAX;
    qd_send_command(hfdcan, motor_id, QD_CMD_ANGLE, (int16_t)(uint16_t)uval);
}

void qd_set_step_angle(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, float step_angle)
{
    step_angle = qd_clampf(step_angle, -QD_TWO_PI, QD_TWO_PI);
    /* step_angle / 2π * INT16_MAX */
    int32_t val = (int32_t)(step_angle / QD_TWO_PI * (float)INT16_MAX);
    qd_send_command(hfdcan, motor_id, QD_CMD_STEP_ANGLE, qd_clamp_int16(val));
}

/* ===== 反馈解析函数 ===== */

void qd_update(uint8_t motor_id, const uint8_t feedback[8])
{
    if (motor_id >= QD_MAX_MOTORS || feedback == NULL) return;

    qd_motor_state_t *st = &qd_motor_states[motor_id];

    /* enabled: feedback[0] bit0 */
    st->enabled = feedback[0] & 0x01;

    /* current: int16 at feedback[2..3] (小端) × 10 / INT16_MAX = A */
    int16_t cur_raw = (int16_t)((uint16_t)feedback[2] | ((uint16_t)feedback[3] << 8));
    st->current = (float)cur_raw * 10.0f / (float)INT16_MAX;

    /* speed: int16 at feedback[4..5] (小端) × 1000 / INT16_MAX = rpm */
    int16_t spd_raw = (int16_t)((uint16_t)feedback[4] | ((uint16_t)feedback[5] << 8));
    st->speed = (float)spd_raw * 1000.0f / (float)INT16_MAX;

    /* angle: uint16 at feedback[6..7] (小端) × 2π / UINT16_MAX = rad */
    uint16_t ang_raw = (uint16_t)feedback[6] | ((uint16_t)feedback[7] << 8);
    st->angle = (float)ang_raw * QD_TWO_PI / (float)UINT16_MAX;
}

/* ===== 状态获取函数 ===== */

qd_motor_state_t *qd_get_state(uint8_t motor_id)
{
    if (motor_id >= QD_MAX_MOTORS) return NULL;
    return &qd_motor_states[motor_id];
}