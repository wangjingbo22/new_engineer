#include "chassis.h"
#include "bsp_can.h"
#include "math.h"

chassis_motor_fb_t chassis_fb[4] = {0};

// 3508 电机 CAN ID 映射
// ID 3→0x203, ID 4→0x204, ID 5→0x205, ID 6→0x206
// C620 电调: ID 1-4 用 0x200 发送, ID 5-8 用 0x1FF 发送
// 你的电机: LF=5, RF=6, LR=3, RR=4
// 其中 ID 3,4 用 0x200 发送（data[4-7]）, ID 5,6 用 0x1FF 发送（data[0-3]）

/**
 * @brief  解析 3508 电机反馈帧
 */
static void chassis_parse_fb(chassis_motor_fb_t *fb, uint8_t *data)
{
    fb->angle         = (uint16_t)(data[0] << 8 | data[1]);
    fb->speed_rpm     = (int16_t)(data[2] << 8 | data[3]);
    fb->torque_current= (int16_t)(data[4] << 8 | data[5]);
    fb->temperature   = data[6];
}

/**
 * @brief  CAN2 接收回调 —— 解析 3508 反馈
 *         反馈 CAN ID: 0x201+电机ID, 即 ID3→0x203, ID4→0x204, ID5→0x205, ID6→0x206
 */
void chassis_can2_callback(void)
{
    uint16_t rec_id;
    uint8_t rx_data[8] = {0};
    canx_receive(&hcan2, &rec_id, rx_data);

    switch (rec_id)
    {
        case 0x205: chassis_parse_fb(&chassis_fb[CHASSIS_LF], rx_data); break; // 左前 ID=5
        case 0x206: chassis_parse_fb(&chassis_fb[CHASSIS_RF], rx_data); break; // 右前 ID=6
        case 0x203: chassis_parse_fb(&chassis_fb[CHASSIS_LR], rx_data); break; // 左后 ID=3
        case 0x204: chassis_parse_fb(&chassis_fb[CHASSIS_RR], rx_data); break; // 右后 ID=4
    }
}

/**
 * @brief  发送四个 3508 电机电流指令
 *         ID 5,6 → CAN ID 0x1FF, data[0-3] = ID5电流, data[4-7] = ID6电流
 *         ID 3,4 → CAN ID 0x200, data[4-7] = ID3电流(byte4-5), ID4电流(byte6-7)
 *         注意: 0x200 帧中 data[0-3] 是 ID1,ID2（未使用，填0）
 */
void chassis_set_current(int16_t lf, int16_t rf, int16_t lr, int16_t rr)
{
    uint8_t data_1ff[8] = {0};
    uint8_t data_200[8] = {0};

    // 0x1FF: ID5(左前) = data[0-1], ID6(右前) = data[2-3], ID7 = data[4-5], ID8 = data[6-7]
    data_1ff[0] = (uint8_t)(lf >> 8);
    data_1ff[1] = (uint8_t)(lf);
    data_1ff[2] = (uint8_t)(rf >> 8);
    data_1ff[3] = (uint8_t)(rf);

    // 0x200: ID1 = data[0-1], ID2 = data[2-3], ID3(左后) = data[4-5], ID4(右后) = data[6-7]
    data_200[4] = (uint8_t)(lr >> 8);
    data_200[5] = (uint8_t)(lr);
    data_200[6] = (uint8_t)(rr >> 8);
    data_200[7] = (uint8_t)(rr);

    canx_send_data(&hcan2, 0x1FF, data_1ff, 8);
    canx_send_data(&hcan2, 0x200, data_200, 8);
}

/**
 * @brief  麦克纳姆轮全向运动解算
 * @param  vx: 前后速度（正=前进）
 * @param  vy: 左右速度（正=右平移）
 * @param  wz: 旋转速度（正=逆时针）
 * @param  speed_max: 最大电流限幅（建议 3000~8000）
 *
 * 麦轮安装方式（X 型）:
 *   LF(5)\  /RF(6)
 *          \/
 *          /\
 *   LR(3)/  \RR(4)
 *
 * 解算公式:
 *   LF = vx - vy - wz
 *   RF = vx + vy + wz
 *   LR = vx + vy - wz
 *   RR = vx - vy + wz
 */
void chassis_mecanum_calc(float vx, float vy, float wz, float speed_max)
{
    float wheel[4];
    wheel[CHASSIS_LF] = vx - vy - wz;   // 左前
    wheel[CHASSIS_RF] = vx + vy + wz;   // 右前
    wheel[CHASSIS_LR] = vx + vy - wz;   // 左后
    wheel[CHASSIS_RR] = vx - vy + wz;   // 右后

    // 找最大值做等比限幅，保持运动方向不变
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
