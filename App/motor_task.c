#include "motor_task.h"
#include "jc4310.h"
#include "can_bsp.h"

#define MOTOR1_ID               0x01
#define MOTOR2_ID               0x01    // 与工作项目一致，两个电机各自在不同CAN总线上，ID都是1

#define JC_MODE_SPEED           0x01

void StartMotorTask(void const *argument)
{
    osDelay(5000);

    jc_enter_closed_loop(&hfdcan1, MOTOR1_ID);
    osDelay(50);
    jc_enter_closed_loop(&hfdcan2, MOTOR2_ID);
    osDelay(50);
    jc_set_control_mode(&hfdcan1, MOTOR1_ID, JC_MODE_SPEED);
    osDelay(50);
    jc_set_control_mode(&hfdcan2, MOTOR2_ID, JC_MODE_SPEED);
    osDelay(50);
    //jc_set_abs_angle_x100(&hfdcan2, MOTOR2_ID, 19500);
    osDelay(50);

    for (;;)
    {
        //jc_set_speed_rpm_x100(&hfdcan1, MOTOR1_ID, 20000);   // 100.00 rpm
        jc_set_speed_rpm_x100(&hfdcan1, MOTOR2_ID, 5000);
        osDelay(10);
    }
}
