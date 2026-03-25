#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "main.h"
#include "can.h"

// 麦轮底盘电机 ID（大疆 3508 + C620 电调）
// 全部用 0x200 帧控制 ID 1-4
// 左前=1, 右前=2, 右后=3, 左后=4
#define CHASSIS_LF  0   // 左前 ID=1
#define CHASSIS_RF  1   // 右前 ID=2
#define CHASSIS_RR  2   // 右后 ID=3
#define CHASSIS_LR  3   // 左后 ID=4

// 3508 电机反馈数据
typedef struct {
    int16_t  speed_rpm;    // 转速 rpm
    int16_t  torque_current; // 实际转矩电流
    uint16_t angle;        // 机械角度 0-8191
    uint8_t  temperature;  // 温度
} chassis_motor_fb_t;

extern chassis_motor_fb_t chassis_fb[4];

// === M2006 夹爪旋转电机 (ID=5,6, C610 电调) ===
#define GRIP_ROT_A  0   // 旋转电机A ID=5
#define GRIP_ROT_B  1   // 旋转电机B ID=6

extern chassis_motor_fb_t grip_rot_fb[2];

// 发送底盘四个电机电流（-16384~16384）
void chassis_set_current(int16_t lf, int16_t rf, int16_t lr, int16_t rr);

// 发送M2006旋转电机电流（-10000~10000）
void grip_rot_set_current(int16_t mot_a, int16_t mot_b);

// 麦轮运动解算
void chassis_mecanum_calc(float vx, float vy, float wz, float speed_max);

// CAN2 接收回调
void chassis_can2_callback(void);

#endif /* __CHASSIS_H__ */
