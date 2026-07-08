#include "imu_task.h"
#include "bmi088driver.h"
#include "MahonyAHRS.h"
#include "tim.h"
#include "vofa.h"
#include "qd4310.h"
#include "can_bsp.h"
#include "pid.h"
#include "usbd_cdc_if.h"
#include <math.h>
#include <stdio.h>

#define DES_TEMP    40.0f
#define KP          100.f
#define KI          50.f
#define KD          10.f
#define MAX_OUT     500

/* ── YAW 角度闭环 PID 参数 (参考 QGimbal, 适配 PID_Update dt 缩放) ──
   QGimbal 位置式 PID: KP=5.0, KI=0.1, KD=110.0 (无 dt 缩放, 1kHz)
   PID_Update 积分 *=dt, 微分 /=dt. dt≈0.001:
     KI 需 ×1000 → 100.0;  KD 需 ÷1000 → 0.11 */
#define YAW_KP              500.0f     /* 比例 (QGimbal: 5.0) */
#define YAW_KI              20.0f   /* 积分 (QGimbal KI=0.1, 已补偿 dt) */
#define YAW_KD              0.11f    /* 微分 (QGimbal KD=110.0, 已补偿 dt) */
#define YAW_INTEGRAL_LIMIT  1.0f     /* 积分限幅, 等效 QGimbal ±1.8rad */
#define YAW_DEAD_ZONE       0.0f     /* 死区 */
#define TARGET_YAW          0.0f     /* 目标偏航角 (度) */
#define YAW_SPEED_MAX       1000.0f  /* 输出限幅 */

float gyro[3] = {0.0f};
float acc[3] = {0.0f};
static float temp = 0.0f;

float imuQuat[4] = {0.0f};
float imuAngle[3] = {0.0f};

float out = 0;
float err = 0;
float err_l = 0;
float err_ll = 0;

volatile uint32_t can_tx_cnt = 0;   /* CAN发送计数器(调试用) */
volatile uint32_t can_err_cnt = 0;  /* CAN发送失败计数 */

/* ── YAW 角度闭环控制变量 ── */
static PID_Controller yaw_pid;              /* 偏航角 → 差速 PID */
static float          target_yaw = TARGET_YAW;
static uint8_t        yaw_ctrl_enabled = 1; /* 偏航闭环使能, 0=禁用 1=使能 */

/* 角度误差环绕: 将误差限幅到 [-180, 180] */
static float angle_wrap_180(float error)
{
    while (error >  180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}

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
    osDelay(500);  /* 等USB枚举完成 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    CDC_Transmit_HS((uint8_t*)"BOOT\r\n", 6);

    while(BMI088_init())
    {
        osDelay(100);
    }

    AHRS_init(imuQuat);
    CDC_Transmit_HS((uint8_t*)"IMU_OK\r\n", 8);

    /* ── Motor startup: CAN1 + CAN2, ID=0 (QD4310协议) ── */
    QD4310_t motor1 = {.id = 0x00, .hfdcan = &hfdcan1};
    QD4310_t motor2 = {.id = 0x00, .hfdcan = &hfdcan2};
    osDelay(3000);

    if (QD4310_Enable(&motor1) == 0)
        can_tx_cnt++;
    else
        can_err_cnt++;
    osDelay(10);
    if (QD4310_Enable(&motor2) == 0)
        can_tx_cnt++;
    else
        can_err_cnt++;
    osDelay(10);

    /* ── YAW 角度闭环 PID 初始化 (参考 GIMBAL 参数) ── */
    PID_Init(&yaw_pid, YAW_KP, YAW_KI, YAW_KD, YAW_SPEED_MAX);
    yaw_pid.integral_limit = YAW_INTEGRAL_LIMIT;
    yaw_pid.dead_zone      = YAW_DEAD_ZONE;
    PID_Reset(&yaw_pid);

    float yaw_out = 0.0f;
    uint32_t imu_tick = 0;
    uint32_t cdc_err = 0;
    char cdc_buf[128];

    /* Infinite loop */
    for(;;)
    {
        BMI088_read(gyro, acc, &temp);

        AHRS_update(imuQuat, gyro, acc);
        GetAngle(imuQuat, imuAngle + INS_YAW_ADDRESS_OFFSET, imuAngle + INS_PITCH_ADDRESS_OFFSET, imuAngle + INS_ROLL_ADDRESS_OFFSET);

        /* CDC send IMU data every 10ms */
        if (++imu_tick >= 10) {
            imu_tick = 0;
            int len = snprintf(cdc_buf, sizeof(cdc_buf),
                "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.1f,%.2f\r\n",
                gyro[0], gyro[1], gyro[2],
                acc[0], acc[1], acc[2],
                imuAngle[0], imuAngle[1], imuAngle[2],
                temp,
                yaw_out);
            if (CDC_Transmit_HS((uint8_t*)cdc_buf, len) != USBD_OK)
                cdc_err++;
        }

        /* Temperature PWM control */
        err_ll = err_l;
        err_l = err;
        err = DES_TEMP - temp;
        out = KP*err + KI*(err + err_l + err_ll) + KD*(err - err_l);
        if (out > MAX_OUT) out = MAX_OUT;
        if (out < 0) out = 0.f;
        htim3.Instance->CCR4 = (uint16_t)out;

        /* ── YAW 角度闭环: 角度环绕 + PID 结构体 → 左右轮差速 ── */
        if (yaw_ctrl_enabled) {
            float yaw_current = imuAngle[INS_YAW_ADDRESS_OFFSET];
            float yaw_error = angle_wrap_180(target_yaw - yaw_current);

            /* 将 setpoint 包裹到当前角度附近, 保证 PID_Update 内部误差正确 */
            float wrapped_sp = yaw_current + yaw_error;

            PID_Update(&yaw_pid, wrapped_sp, yaw_current, 0.001f);
            yaw_out = yaw_pid.out;
        } else {
            PID_Reset(&yaw_pid);
            yaw_out = 0.0f;
        }

        /* 速度指令发送: 左轮 +yaw_out, 右轮 -yaw_out (原地旋转) */
        if (QD4310_SetSpeed(&motor1,  yaw_out) == 0)
            can_tx_cnt++;
        else
            can_err_cnt++;
        if (QD4310_SetSpeed(&motor2, -yaw_out) == 0)
            can_tx_cnt++;
        else
            can_err_cnt++;

        osDelay(1);
    }
    /* USER CODE END ImuTask_Entry */
}
