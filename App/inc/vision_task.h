#ifndef __VISION_TASK_H__
#define __VISION_TASK_H__

#include "cmsis_os.h"
#include "pid.h"

#define VISION_FRAME_HEADER1    0xAA
#define VISION_FRAME_HEADER2    0xFF
#define VISION_FRAME_TAIL1      0xFF
#define VISION_FRAME_TAIL2      0xAA
#define VISION_FRAME_LEN        13

#define VISION_MODE_STOP        0x00
#define VISION_MODE_RUN         0x01

#define CDC_RX_BUF_SIZE         1024

/* Vision PID parameters */
#define VISION_LPF_ALPHA        0.30f
#define X_DEAD_ZONE             50.0f
#define Y_DEAD_ZONE             80.0f
#define X_CENTER_HOLD_CNT       3
#define VISION_LOST_CYCLES      100
#define OUTER_YAW_GAIN           0.0050f
#define OUTER_PITCH_GAIN         0.0100f
#define YAW_LARGE_ERR_THRESH     100.0f   /* delta_x_lpf量程0~1000，100=10%偏离 */
#define YAW_LARGE_ERR_SCALE      0.5f     /* 大误差时转速缩放比 */

typedef struct {
    uint8_t buffer[CDC_RX_BUF_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} CDC_RxRingBuffer;

typedef struct {
    uint8_t mode;
    float delta_x;
    float delta_y;
} VisionData;

typedef struct {
    float target_yaw;
    float target_pitch;
    uint8_t active;
} VisionOutput;

extern VisionOutput vision_output;

void CDC_RxPush(uint8_t byte);
uint8_t CDC_RxPop(uint8_t *byte);
void StartVisionTask(void const * argument);

#endif
