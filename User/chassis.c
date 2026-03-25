#include "chassis.h"
#include "bsp_can.h"
#include "math.h"

chassis_motor_fb_t chassis_fb[4] = {0};
chassis_motor_fb_t grip_rot_fb[2] = {0};

// 3508 电机 CAN ID 映射
// 左前=ID1, 右前=ID2, 左后=ID3, 右后=ID4
// 全部在 0x200 帧内：data[0-1]=ID1, data[2-3]=ID2, data[4-5]=ID3, data[6-7]=ID4
// 反馈 ID: 0x201, 0x202, 0x203, 0x204

static void chassis_parse_fb(chassis_motor_fb_t *fb, uint8_t *data)
{
    fb->angle         = (uint16_t)(data[0] << 8 | data[1]);
    fb->speed_rpm     = (int16_t)(data[2] << 8 | data[3]);
    fb->torque_current= (int16_t)(data[4] << 8 | data[5]);
    fb->temperature   = data[6];
}

void chassis_can2_callback(void)
{
    uint16_t rec_id;
    uint8_t rx_data[8] = {0};
    canx_receive(&hcan2, &rec_id, rx_data);

    switch (rec_id)
    {
        case 0x201: chassis_parse_fb(&chassis_fb[CHASSIS_LF], rx_data); break; // 左前 ID=1
        case 0x202: chassis_parse_fb(&chassis_fb[CHASSIS_RF], rx_data); break; // 右前 ID=2
        case 0x203: chassis_parse_fb(&chassis_fb[CHASSIS_RR], rx_data); break; // 右后 ID=3
        case 0x204: chassis_parse_fb(&chassis_fb[CHASSIS_LR], rx_data); break; // 左后 ID=4
        case 0x205: chassis_parse_fb(&grip_rot_fb[GRIP_ROT_A], rx_data); break; // M2006 A ID=5
        case 0x206: chassis_parse_fb(&grip_rot_fb[GRIP_ROT_B], rx_data); break; // M2006 B ID=6
    }
}

/**
 * @brief  发送四个 3508 电机电流指令
 *         0x200: ID1(左前)=data[0-1], ID2(右前)=data[2-3], ID3(右后)=data[4-5], ID4(左后)=data[6-7]
 */
void chassis_set_current(int16_t lf, int16_t rf, int16_t lr, int16_t rr)
{
    uint8_t data[8];
    data[0] = (uint8_t)(lf >> 8);   // ID1 左前
    data[1] = (uint8_t)(lf);
    data[2] = (uint8_t)(rf >> 8);   // ID2 右前
    data[3] = (uint8_t)(rf);
    data[4] = (uint8_t)(rr >> 8);   // ID3 右后
    data[5] = (uint8_t)(rr);
    data[6] = (uint8_t)(lr >> 8);   // ID4 左后
    data[7] = (uint8_t)(lr);

    canx_send_data(&hcan2, 0x200, data, 8);
}

/**
 * @brief  发送 M2006 夹爪旋转电机电流（-10000~10000）
 *         0x1FF: ID5=data[0-1], ID6=data[2-3]
 */
void grip_rot_set_current(int16_t mot_a, int16_t mot_b)
{
    uint8_t data[8] = {0};
    data[0] = (uint8_t)(mot_a >> 8);  // ID5
    data[1] = (uint8_t)(mot_a);
    data[2] = (uint8_t)(mot_b >> 8);  // ID6
    data[3] = (uint8_t)(mot_b);

    canx_send_data(&hcan2, 0x1FF, data, 8);
}

/**
 * @brief  麦克纳姆轮全向运动解算
 *
 *   LF(1)\  /RF(2)
 *          \/
 *          /\
 *   LR(4)/  \RR(3)
 */
void chassis_mecanum_calc(float vx, float vy, float wz, float speed_max)
{
    float wheel[4];
    wheel[CHASSIS_LF] = vx - vy - wz;
    wheel[CHASSIS_RF] = vx + vy + wz;
    wheel[CHASSIS_LR] = vx + vy - wz;
    wheel[CHASSIS_RR] = vx - vy + wz;

    float max_val = 0;
    for (int i = 0; i < 4; i++) {
        if (fabs(wheel[i]) > max_val) max_val = fabs(wheel[i]);
    }
    if (max_val > speed_max) {
        float scale = speed_max / max_val;
        for (int i = 0; i < 4; i++) wheel[i] *= scale;
    }

    chassis_set_current((int16_t)wheel[CHASSIS_LF],
                        (int16_t)wheel[CHASSIS_RF],
                        (int16_t)wheel[CHASSIS_LR],
                        (int16_t)wheel[CHASSIS_RR]);
}
