/**
 * @file        qd4310.h
 * @brief       基于HAL FDCAN库的QD4310电机CAN总线控制库(C语言版)
 * @details     参考QGimbal项目中的QD4310 C++实现,改写为C函数接口
 *              协议: 3字节发送帧(cmd + int16), 8字节反馈帧
 *              发送CAN ID = 0x400 + motor_id
 *              反馈CAN ID = 0x500 + motor_id
 * @note        默认电机ID为0
 */

#ifndef _QD4310__H__
#define _QD4310__H__

#include "main.h"
#include <stdint.h>

/* ===== 配置宏 ===== */

/** @brief QD4310 默认电机ID */
#define QD_DEFAULT_ID       0x00

/** @brief 最大支持的电机数量 (ID: 0 ~ QD_MAX_MOTORS-1) */
#define QD_MAX_MOTORS       8

/* ===== 物理量限幅 (参考官方例程) ===== */
#define QD4310_MAX_SPEED        1000.0f      /* 最大转速 rpm */
#define QD4310_MIN_SPEED        (-1000.0f)   /* 最小转速 rpm */
#define QD4310_MAX_CURRENT      10.0f        /* 最大电流 A */
#define QD4310_MIN_CURRENT      (-10.0f)     /* 最小电流 A */
#define QD4310_MAX_ANGLE_RAD    6.283185307f /* 最大角度 2π rad */
#define QD4310_MIN_ANGLE_RAD    0.0f         /* 最小角度 0 rad */

/* ===== 命令字定义 (参考官方例程) ===== */
#define QD_CMD_NOP          0x00    /* 空操作 */
#define QD_CMD_ENABLE       0x01    /* 使能电机 */
#define QD_CMD_DISABLE      0x02    /* 失能电机 */
#define QD_CMD_CURRENT      0x03    /* 设置电流 */
#define QD_CMD_SPEED        0x04    /* 设置转速 */
#define QD_CMD_ANGLE        0x05    /* 设置绝对角度 */
#define QD_CMD_LOW_SPEED    0x06    /* 设置低速模式转速 */
#define QD_CMD_STEP_ANGLE   0x07    /* 设置步进角度 */

/* ===== CAN ID 计算宏 ===== */
#define QD_TX_ID(motor_id)  (0x400U + (uint32_t)(motor_id))   /* 发送帧ID */
#define QD_RX_ID(motor_id)  (0x500U + (uint32_t)(motor_id))   /* 反馈帧ID */

/* ===== 电机反馈状态结构体 ===== */
typedef struct {
    uint8_t enabled;    /* 使能状态: 0=失能, 1=使能 */
    float    speed;     /* 转速 (rpm) */
    float    angle;     /* 角度 (rad), 范围[0, 2π] */
    float    current;   /* 电流 (A) */
} qd_motor_state_t;

/* ===== 发送控制函数 ===== */

/**
 * @brief  使能电机
 * @param  hfdcan   FDCAN句柄
 * @param  motor_id 电机ID (0 ~ QD_MAX_MOTORS-1)
 */
void qd_enable(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);

/**
 * @brief  失能电机
 */
void qd_disable(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);

/**
 * @brief  空操作(保持电机在线)
 */
void qd_nop(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);

/**
 * @brief  设置电机电流
 * @param  current  目标电流, 范围[-10, 10] A
 */
void qd_set_current(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, float current);

/**
 * @brief  设置电机转速
 * @param  speed  目标转速, 范围[-1000, 1000] rpm
 */
void qd_set_speed(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, float speed);

/**
 * @brief  设置电机低速模式转速
 * @param  speed  目标转速, 范围[-1000, 1000] rpm
 */
void qd_set_low_speed(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, float speed);

/**
 * @brief  设置电机绝对角度
 * @param  angle  目标角度, 范围[0, 2π] rad
 */
void qd_set_angle(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, float angle);

/**
 * @brief  设置电机步进角度
 * @param  step_angle  步进角度, 范围[-2π, 2π] rad
 */
void qd_set_step_angle(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, float step_angle);

/* ===== 反馈解析函数 ===== */

/**
 * @brief  解析电机反馈帧
 * @param  motor_id  电机ID
 * @param  feedback  8字节反馈数据指针
 * @note   反馈帧格式:
 *         feedback[0]    : bit0 = enabled状态
 *         feedback[2..3] : int16 电流 (×10/INT16_MAX = A)
 *         feedback[4..5] : int16 转速 (×1000/INT16_MAX = rpm)
 *         feedback[6..7] : uint16 角度 (×2π/UINT16_MAX = rad)
 */
void qd_update(uint8_t motor_id, const uint8_t feedback[8]);

/* ===== 状态获取函数 ===== */

/**
 * @brief  获取电机状态指针
 * @return 电机状态结构体指针, 若ID无效则返回NULL
 */
qd_motor_state_t *qd_get_state(uint8_t motor_id);

#endif /* _QD4310__H__ */