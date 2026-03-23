#include "motor_send_tim.h"
#include "dm_motor_ctrl.h"
#include "dm_motor_drv.h"
#include "can.h"
#include "tim.h"

// ---- 共享数据 ----
volatile motor_cmd_t motor_cmd[4] = {0};

// ---- 私有变量 ----
static volatile uint8_t send_idx = 0;
static const uint8_t motor_map[4] = { Motor1, Motor2, Motor3, Motor4 };

/**
 * @brief  复用 CubeMX 已初始化的 TIM8，重配为 250μs 周期中断
 *         TIM8 在 APB2，定时器时钟 = 168MHz
 *         Prescaler = 167 → 1MHz (1μs/tick)
 *         Period = 249 → 250μs
 */
void motor_send_tim_init(void)
{
    // TIM8 时钟和 NVIC 已由 CubeMX MspInit 配好（优先级7），只需改周期参数
    __HAL_TIM_SET_PRESCALER(&htim8, 167);   // 168MHz / 168 = 1MHz
    __HAL_TIM_SET_AUTORELOAD(&htim8, 249);  // 250 ticks = 250μs
    __HAL_TIM_SET_COUNTER(&htim8, 0);
}

/**
 * @brief  启动定时器中断
 */
void motor_send_tim_start(void)
{
    HAL_TIM_Base_Start_IT(&htim8);
}

/**
 * @brief  250μs 中断回调 —— 每次只发 1 帧，轮转 4 个电机
 */
void motor_send_tim_isr(void)
{
    __HAL_TIM_CLEAR_IT(&htim8, TIM_IT_UPDATE);

    uint8_t idx = send_idx;
    uint8_t mi  = motor_map[idx];

    float pos = motor_cmd[idx].pos;
    float vel = motor_cmd[idx].vel;

    if (motor[mi].ctrl.mode == psi_mode) {
        float cur = motor_cmd[idx].tor;
        psi_ctrl(&hcan1, motor[mi].id, pos, vel, cur);
    } else {
        float kp  = motor_cmd[idx].kp;
        float kd  = motor_cmd[idx].kd;
        float tor = motor_cmd[idx].tor;
        mit_ctrl(&hcan1, &motor[mi], motor[mi].id, pos, vel, kp, kd, tor);
    }

    send_idx = (idx + 1) & 0x03;
}
