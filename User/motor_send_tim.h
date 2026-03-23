#ifndef __MOTOR_SEND_TIM_H__
#define __MOTOR_SEND_TIM_H__

#include "main.h"

// ISR 与主任务之间的共享指令结构
typedef struct {
    float pos;
    float vel;
    float kp;
    float kd;
    float tor;
} motor_cmd_t;

// 4 个电机的指令缓冲（主任务写，ISR读）
extern volatile motor_cmd_t motor_cmd[4];

// TIM2 初始化（250μs 周期中断）
void motor_send_tim_init(void);

// 启动定时器发送
void motor_send_tim_start(void);

// ISR 回调（在 TIM2_IRQHandler 中调用）
void motor_send_tim_isr(void);

#endif /* __MOTOR_SEND_TIM_H__ */
