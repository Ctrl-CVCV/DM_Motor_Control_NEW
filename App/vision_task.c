#include "vision_task.h"
#include "imu_task.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <math.h>

#define PI_F 3.14159265358979323846f

CDC_RxRingBuffer cdc_rx_buf = {0};
VisionOutput vision_output = {0};

static PID_Controller x_pid;
static PID_Controller y_pid;

static float normalize_angle_rad(float ang)
{
    while (ang > PI_F)  ang -= 2.0f * PI_F;
    while (ang < -PI_F) ang += 2.0f * PI_F;
    return ang;
}

void CDC_RxPush(uint8_t byte)
{
    uint16_t next = (cdc_rx_buf.head + 1) % CDC_RX_BUF_SIZE;
    if (next != cdc_rx_buf.tail) {
        cdc_rx_buf.buffer[cdc_rx_buf.head] = byte;
        cdc_rx_buf.head = next;
    }
}

uint8_t CDC_RxPop(uint8_t *byte)
{
    if (cdc_rx_buf.head == cdc_rx_buf.tail) return 0;
    *byte = cdc_rx_buf.buffer[cdc_rx_buf.tail];
    cdc_rx_buf.tail = (cdc_rx_buf.tail + 1) % CDC_RX_BUF_SIZE;
    return 1;
}

static uint8_t parse_frame(VisionData *data)
{
    static uint8_t state = 0;
    static uint8_t buf[VISION_FRAME_LEN];
    static uint8_t idx = 0;
    uint8_t byte;

    while (CDC_RxPop(&byte)) {
        switch (state) {
        case 0:
            if (byte == VISION_FRAME_HEADER1) { buf[0] = byte; idx = 1; state = 1; }
            break;
        case 1:
            if (byte == VISION_FRAME_HEADER2) { buf[1] = byte; idx = 2; state = 2; }
            else state = 0;
            break;
        default:
            buf[idx++] = byte;
            if (idx >= VISION_FRAME_LEN) {
                state = 0;
                if (buf[11] == VISION_FRAME_TAIL1 && buf[12] == VISION_FRAME_TAIL2) {
                    data->mode = buf[2];
                    memcpy(&data->delta_x, &buf[3], 4);
                    memcpy(&data->delta_y, &buf[7], 4);
                    return 1;
                }
            }
            break;
        }
    }
    return 0;
}

void StartVisionTask(void const * argument)
{
    osDelay(4000);

    VisionData vis_data;
    float delta_x_lpf = 0.0f;
    float delta_y_lpf = 0.0f;
    float target_yaw = 0.0f;
    uint8_t vision_active = 0;
    uint8_t x_centered_cnt = 0;
    uint32_t vision_lost_cnt = 0;
    uint8_t target_yaw_inited = 0;

    PID_Init(&x_pid, 11000.0f, 0.000f, 0.000f, 5000000.0f);
    PID_Init(&y_pid, 0.00000040f, 0.00000010f, 0.00000020f, 500.0f);

    for (;;)
    {
        /* ── 1. Check for new vision frame, update LPF ── */
        uint8_t new_frame = parse_frame(&vis_data);

        if (new_frame) {
            uint8_t ack[] = "RECEIVE_OK\r\n";
            CDC_Transmit_HS(ack, sizeof(ack) - 1);

            if (vis_data.mode == VISION_MODE_RUN) {
                if (!vision_active) {
                    delta_x_lpf = vis_data.delta_x;
                    delta_y_lpf = vis_data.delta_y;
                    PID_Reset(&x_pid);
                    PID_Reset(&y_pid);
                } else {
                    delta_x_lpf = delta_x_lpf * (1.0f - VISION_LPF_ALPHA) + vis_data.delta_x * VISION_LPF_ALPHA;
                    delta_y_lpf = delta_y_lpf * (1.0f - VISION_LPF_ALPHA) + vis_data.delta_y * VISION_LPF_ALPHA;
                }

                /* X dead zone with center-hold */
                if (fabsf(delta_x_lpf) < X_DEAD_ZONE) {
                    x_centered_cnt++;
                    if (x_centered_cnt >= X_CENTER_HOLD_CNT) {
                        delta_x_lpf = 0.0f;
                        PID_Reset(&x_pid);
                    }
                } else {
                    x_centered_cnt = 0;
                }

                /* Y dead zone */
                if (fabsf(delta_y_lpf) < Y_DEAD_ZONE) {
                    delta_y_lpf = 0.0f;
                }

                vision_active = 1;
                vision_lost_cnt = 0;
            } else if (vis_data.mode == VISION_MODE_STOP) {
                vision_active = 0;
                PID_Reset(&x_pid);
                PID_Reset(&y_pid);
                vision_lost_cnt = VISION_LOST_CYCLES + 1;
            }
        }

        /* ── 2. Vision lost timeout ── */
        if (!new_frame) {
            vision_lost_cnt++;
            if (vision_lost_cnt > VISION_LOST_CYCLES && vision_active) {
                vision_active = 0;
                PID_Reset(&x_pid);
                PID_Reset(&y_pid);
            }
        }

        /* ── 3. Run outer PID every cycle (400Hz) ── */
        if (vision_active) {
            /* Init target_yaw from current IMU yaw on first activation */
            if (!target_yaw_inited) {
                target_yaw = imuAngle[INS_YAW_ADDRESS_OFFSET];
                target_yaw_inited = 1;
            }

            PID_Update(&x_pid, 0.0f, delta_x_lpf, 0.0025f);
            PID_Update(&y_pid, 0.0f, -delta_y_lpf, 0.0025f);

            if (x_centered_cnt >= X_CENTER_HOLD_CNT) {
                target_yaw = imuAngle[INS_YAW_ADDRESS_OFFSET];
            } else {
                float yaw_rate = x_pid.out * OUTER_YAW_GAIN;
                target_yaw = normalize_angle_rad(target_yaw + yaw_rate * 0.0025f);
            }

            vision_output.target_yaw = target_yaw;
            vision_output.pitch_cmd = y_pid.out;
            vision_output.active = 1;
        } else {
            vision_output.active = 0;
            target_yaw_inited = 0;
        }

        osDelay(2);
    }
}
