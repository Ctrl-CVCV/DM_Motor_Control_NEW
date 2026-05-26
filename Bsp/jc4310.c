#include "jc4310.h"
void jc_calibrate_motor(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id)
{
    if (motor_id == 0 || motor_id > 127) return;
    uint8_t can_data[8] = {0x2B, 0x00, 0xA1, 0x00, 0x00, 0x01, 0x00, 0x00};
    uint32_t can_id = 0x600 + motor_id;
    fdcanx_send_data(hfdcan, can_id, can_data, 8);
}
// 设置电机速度（单位：rpm），此处为乘以100后的整数值（例：100.00 rpm -> 10000）
void jc_set_speed_rpm_x100(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, int32_t speed_rpm)
{
    // 1. 参数安全检查（可选但推荐）
    if (motor_id == 0 || motor_id > 127) {
        return; // 无效 ID
    }

    // 2. 参数已是 ×100 的值，直接使用
    int32_t speed_scaled = speed_rpm;

    // 3. 构造 CAN 数据帧（8 字节）
    uint8_t can_data[8];

    can_data[0] = 0x23;           // 命令字：设置速度
    can_data[1] = 0x00;           // 寄存器高字节（0x0021）
    can_data[2] = 0x21;           // 寄存器低字节
    can_data[3] = 0x00;           // 保留/空字节

    // 4. 填充 32 位速度值（大端序：MSB first）
    can_data[4] = (uint8_t)((speed_scaled >> 24) & 0xFF); // Byte 1 (MSB)
    can_data[5] = (uint8_t)((speed_scaled >> 16) & 0xFF); // Byte 2
    can_data[6] = (uint8_t)((speed_scaled >>  8) & 0xFF); // Byte 3
    can_data[7] = (uint8_t)( speed_scaled        & 0xFF); // Byte 4 (LSB)

    // 5. 计算标准 CAN ID（0x600 + motor_id）
    uint32_t can_id = 0x600 + motor_id;

    // 6. 发送 CAN 帧（使用传入的 FDCAN 句柄）
    fdcanx_send_data(hfdcan, can_id, can_data, 8);
}
void jc_enter_closed_loop(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id)
{
    // 参数检查
    if (motor_id == 0 || motor_id > 127) {
        return; // 无效 ID
    }

    // 构造 CAN 数据帧（8 字节）
    uint8_t can_data[8];

    can_data[0] = 0x2B;           // 命令字：进入闭环
    can_data[1] = 0x00;           // 寄存器高字节（0x00A2）
    can_data[2] = 0xA2;           // 寄存器低字节
    can_data[3] = 0x00;           // null

    can_data[4] = 0x00;           // 数据高字节（0x0001）
    can_data[5] = 0x01;           // 数据低字节（开启闭环）
    can_data[6] = 0x00;           // null
    can_data[7] = 0x00;           // null

    // 计算 CAN ID
    uint32_t can_id = 0x600 + motor_id;

    // 发送 CAN 帧
    fdcanx_send_data(hfdcan, can_id, can_data, 8);
}
void jc_set_control_mode(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, uint8_t mode)
{
    // 参数检查
    if (motor_id == 0 || motor_id > 127) {
        return; // 无效 ID
    }

    // 构造 CAN 数据帧（8 字节）
    uint8_t can_data[8];

    can_data[0] = 0x2B;           // 命令字：切换控制模式
    can_data[1] = 0x00;           // 寄存器高字节（0x0060）
    can_data[2] = 0x60;           // 寄存器低字节
    can_data[3] = 0x00;           // null

    can_data[4] = 0x00;           // 控制模式高字节
    can_data[5] = mode;           // 控制模式低字节（如 0x01=速度，0x02=位置梯形，0x04 = 位置直通）
    can_data[6] = 0x00;           // null
    can_data[7] = 0x00;           // null

    // 计算 CAN ID
    uint32_t can_id = 0x600 + motor_id;

    // 发送 CAN 帧
    fdcanx_send_data(hfdcan, can_id, can_data, 8);
}
/*控制模式说明
速度模式（mode = 0x01）：在此模式下，电机控制器将根据接收到的速度指令（如 jc_set_speed_rpm 函数）调整电机的转速。适用于需要恒定速度运行的场景，如风扇或输送带。
位置梯形模式（mode = 0x02）：在此模式下，电机控制器将根据接收到的位置指令（如 jc_set_abs_position_x100 或 jc_set_rel_position_x100 函数）移动电机到指定位置。控制器会自动计算加速和减速，以实现平滑的运动。适用于需要精确位置控制的场景，如机械臂或数控机床。

*/
// 设置绝对位置（单位：0.01deg，例：360deg -> 36000）
void jc_set_abs_position_x100(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, int32_t position_x100)
{
    // 参数检查
    if (motor_id == 0 || motor_id > 127) {
        return; // 无效 ID
    }

    // 构造 CAN 数据帧（8 字节）
    uint8_t can_data[8];

    can_data[0] = 0x23;           // 命令字：写 32 位数据
    can_data[1] = 0x00;           // 寄存器高字节（0x0023）
    can_data[2] = 0x23;           // 寄存器低字节（绝对位置）
    can_data[3] = 0x00;           // null

    // 填充 32 位位置值（大端序：MSB first）
    can_data[4] = (uint8_t)((position_x100 >> 24) & 0xFF); // Byte 1 (MSB)
    can_data[5] = (uint8_t)((position_x100 >> 16) & 0xFF); // Byte 2
    can_data[6] = (uint8_t)((position_x100 >>  8) & 0xFF); // Byte 3
    can_data[7] = (uint8_t)( position_x100        & 0xFF); // Byte 4 (LSB)

    // 计算 CAN ID
    uint32_t can_id = 0x600 + motor_id;

    // 发送 CAN 帧
    fdcanx_send_data(hfdcan, can_id, can_data, 8);
}

// 设置相对位置（单位：0.01deg，例：+360deg -> +36000，-180.45deg -> -18045）
void jc_set_rel_position_x100(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, int32_t delta_position_x100)
{
    // 参数检查
    if (motor_id == 0 || motor_id > 127) {
        return; // 无效 ID
    }

    // 构造 CAN 数据帧（8 字节）
    uint8_t can_data[8];

    can_data[0] = 0x23;           // 命令字：写 32 位数据
    can_data[1] = 0x00;           // 寄存器高字节（0x0025）
    can_data[2] = 0x25;           // 寄存器低字节（相对位置）
    can_data[3] = 0x00;           // null

    // 填充 32 位位置增量（大端序：MSB first）
    can_data[4] = (uint8_t)((delta_position_x100 >> 24) & 0xFF); // Byte 1 (MSB)
    can_data[5] = (uint8_t)((delta_position_x100 >> 16) & 0xFF); // Byte 2
    can_data[6] = (uint8_t)((delta_position_x100 >>  8) & 0xFF); // Byte 3
    can_data[7] = (uint8_t)( delta_position_x100        & 0xFF); // Byte 4 (LSB)

    // 计算 CAN ID
    uint32_t can_id = 0x600 + motor_id;

    // 发送 CAN 帧
    fdcanx_send_data(hfdcan, can_id, can_data, 8);
}

// 兼容旧接口名：含义等同于设置绝对位置
void jc_set_abs_angle_x100(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, int32_t angle_x100)
{
    jc_set_abs_position_x100(hfdcan, motor_id, angle_x100);
}