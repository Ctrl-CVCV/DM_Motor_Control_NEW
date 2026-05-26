#ifndef _JC4310__H__
#define _JC4310__H__
#include "main.h" // 包含 hfdcan2 定义
#include <stdio.h>
#include "can_bsp.h" // 包含 fdcanx_send_data 函数声明

void jc_calibrate_motor(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);
void jc_set_speed_rpm_x100(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, int32_t speed_rpm);
void jc_enter_closed_loop(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);
void jc_set_control_mode(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, uint8_t mode);
void jc_set_abs_position_x100(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, int32_t position_x100);
void jc_set_rel_position_x100(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, int32_t delta_position_x100);
void jc_set_abs_angle_x100(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, int32_t angle_x100);
#endif // !_JC4310__H__