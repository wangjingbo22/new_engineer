#include "motor_send_tim.h"
#include "dm_motor_ctrl.h"
#include "dm_motor_drv.h"
#include "can.h"
#include "stm32f4xx_hal.h"

// ---- 共享数据 ----
volatile motor_cmd_t motor_cmd[4] = {0};

// ---- 私有变量 ----
static TIM_HandleTypeDef htim2;
static volatile uint8_t send_idx = 0;

// Motor 数组下标映射：0→Motor1, 1→Motor2, 2→Motor3, 3→Motor4
static const uint8_t motor_map[4] = { Motor1, Motor2, Motor3, Motor4 };

/**
 * @brief  初始化 TIM2 为 250μs 周期中断
 *         APB1 timer clock = 84MHz (168/2, x2 because prescaler>1)
 *         Prescaler = 83 → 1MHz (1μs/tick)
 *         Period = 249 → 250μs
 */
void motor_send_tim_init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 83;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 249;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim2);

    // 优先级 7：低于 CAN RX (6)，不会抢占反馈接收
    HAL_NVIC_SetPriority(TIM2_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/**
 * @brief  启动定时器中断
 */
void motor_send_tim_start(void)
{
    HAL_TIM_Base_Start_IT(&htim2);
}

/**
 * @brief  250μs 中断回调 —— 每次只发 1 帧，轮转 4 个电机
 *         每个电机 1kHz 刷新，邮箱永远有空位，零阻塞
 */
void motor_send_tim_isr(void)
{
    // 清中断标志
    __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);

    uint8_t idx = send_idx;
    uint8_t mi  = motor_map[idx];

    // 读取主任务写入的目标值（volatile 读，单次浮点读在 CM4 上是原子的）
    float pos = motor_cmd[idx].pos;
    float vel = motor_cmd[idx].vel;
    float kp  = motor_cmd[idx].kp;
    float kd  = motor_cmd[idx].kd;
    float tor = motor_cmd[idx].tor;

    mit_ctrl(&hcan1, &motor[mi], motor[mi].id, pos, vel, kp, kd, tor);

    send_idx = (idx + 1) & 0x03;
}
