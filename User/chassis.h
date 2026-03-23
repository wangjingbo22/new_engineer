#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "main.h"
#include "can.h"

// 麦轮底盘电机 ID（大疆 3508 + C620 电调）
// C620 电调接收 ID: 0x1FF 控制 ID 5-8, 0x200 控制 ID 1-4
// 你的 ID: 左前5 右前6 左后3 右后4
#define CHASSIS_LF  0   // 左前 ID=5
#define CHASSIS_RF  1   // 右前 ID=6
#define CHASSIS_LR  2   // 左后 ID=3
#define CHASSIS_RR  3   // 右后 ID=4

// 3508 电机反馈数据
typedef struct {
    int16_t  speed_rpm;    // 转速 rpm
    int16_t  torque_current; // 实际转矩电流
    uint16_t angle;        // 机械角度 0-8191
    uint8_t  temperature;  // 温度
} chassis_motor_fb_t;

extern chassis_motor_fb_t chassis_fb[4];

// 初始化（注册 CAN2 回调）
void chassis_init(void);

// 发送四个电机电流指令（-16384~16384）
void chassis_set_current(int16_t lf, int16_t rf, int16_t lr, int16_t rr);

// 麦轮运动解算：vx=前后, vy=左右平移, wz=旋转，speed_max=限速
void chassis_mecanum_calc(float vx, float vy, float wz, float speed_max);

// CAN2 接收回调（在 bsp_can 的 can2_rx_callback 中调用）
void chassis_can2_callback(void);

#endif /* __CHASSIS_H__ */
