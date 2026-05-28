#include "imu_task.h"
#include "bmi088driver.h"
#include "MahonyAHRS.h"
#include "tim.h"
#include "vofa.h"
#include "vision_task.h"
#include "jc4310.h"
#include "can_bsp.h"
#include "pid.h"
#include <math.h>

#define DES_TEMP    40.0f
#define KP          100.f
#define KI          50.f
#define KD          10.f
#define MAX_OUT     500

#define MOTOR1_ID               0x01
#define MOTOR2_ID               0x01
#define JC_MODE_SPEED           0x01
#define JC_MODE_POSITION        0x02

float gyro[3] = {0.0f};
float acc[3] = {0.0f};
static float temp = 0.0f;

float imuQuat[4] = {0.0f};
float imuAngle[3] = {0.0f};

float out = 0;
float err = 0;
float err_l = 0;
float err_ll = 0;

void AHRS_init(float quat[4])
{
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;

}

void AHRS_update(float quat[4], float gyro[3], float accel[3])
{
    MahonyAHRSupdateIMU(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
}

void GetAngle(float q[4], float *yaw, float *pitch, float *roll)
{
    *yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f);
    *pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]));
    *roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f);
}

#define PI_F 3.14159265358979323846f

static float normalize_angle_rad(float ang)
{
    while (ang > PI_F)  ang -= 2.0f * PI_F;
    while (ang < -PI_F) ang += 2.0f * PI_F;
    return ang;
}

static float angle_diff_rad(float target, float current)
{
    return normalize_angle_rad(target - current);
}

/* USER CODE BEGIN Header_ImuTask_Entry */
/**
* @brief Function implementing the ImuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ImuTask_Entry */
void ImuTask_Entry(void const * argument)
{
    /* USER CODE BEGIN ImuTask_Entry */
    osDelay(10);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    while(BMI088_init())
    {
        osDelay(100);
    }

    AHRS_init(imuQuat);

    /* Wait for system to fully boot, then init motors */
    osDelay(4000);

    jc_enter_closed_loop(&hfdcan1, MOTOR1_ID);
    osDelay(50);
    jc_enter_closed_loop(&hfdcan2, MOTOR2_ID);
    osDelay(50);
    jc_set_control_mode(&hfdcan1, MOTOR1_ID, JC_MODE_SPEED);
    osDelay(50);
    jc_set_control_mode(&hfdcan2, MOTOR2_ID, JC_MODE_POSITION);
    osDelay(50);
    jc_set_abs_angle_x100(&hfdcan2, MOTOR2_ID, 27000);
    osDelay(50);
    jc_set_abs_angle_x100(&hfdcan1, MOTOR1_ID, 19500);
    osDelay(50);

    /* Let IMU readings settle, then lock current yaw as target */
    osDelay(100);
    float yaw_target = imuAngle[INS_YAW_ADDRESS_OFFSET];

    PID_Controller yaw_pid;
    PID_Init(&yaw_pid, 500.0f, 250.0f, 100.0f, 5000.0f);
    yaw_pid.integral_limit = 200.0f;

    /* Infinite loop */
    for(;;)
    {
        BMI088_read(gyro, acc, &temp);

        AHRS_update(imuQuat, gyro, acc);
        GetAngle(imuQuat, imuAngle + INS_YAW_ADDRESS_OFFSET, imuAngle + INS_PITCH_ADDRESS_OFFSET, imuAngle + INS_ROLL_ADDRESS_OFFSET);

        err_ll = err_l;
        err_l = err;
        err = DES_TEMP - temp;
        out = KP*err + KI*(err + err_l + err_ll) + KD*(err - err_l);
        if (out > MAX_OUT) out = MAX_OUT;
        if (out < 0) out = 0.f;
        htim3.Instance->CCR4 = (uint16_t)out;

        /*vofa_send_data(0, imuAngle[0]);
        vofa_send_data(1, imuAngle[1]);
        vofa_send_data(2, imuAngle[2]);
        vofa_sendframetail();*/

        /* Use vision target_yaw when active, otherwise hold initial yaw */
        float yaw_setpoint = vision_output.active ? vision_output.target_yaw : yaw_target;
        float yaw_error = angle_diff_rad(yaw_setpoint, imuAngle[INS_YAW_ADDRESS_OFFSET]);
        PID_Update(&yaw_pid, yaw_error, 0.0f, 0.001f);
        jc_set_speed_rpm_x100(&hfdcan1, MOTOR1_ID, (int32_t)(yaw_pid.out * 100));
        osDelay(1);
    }
    /* USER CODE END ImuTask_Entry */
}
