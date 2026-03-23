/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dm_motor_ctrl.h"
#include "dm_motor_drv.h"
#include "can.h"
#include "bsp_can.h"
#include "remote.h"
#include "motor_send_tim.h"
#include "chassis.h"
#include "math.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for Init_TaskHandle */
osThreadId_t Init_TaskHandleHandle;
const osThreadAttr_t Init_TaskHandle_attributes = {
  .name = "Init_TaskHandle",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for Comm_TaskHandle */
osThreadId_t Comm_TaskHandleHandle;
const osThreadAttr_t Comm_TaskHandle_attributes = {
  .name = "Comm_TaskHandle",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for Ins_TaskHandler */
osThreadId_t Ins_TaskHandlerHandle;
const osThreadAttr_t Ins_TaskHandler_attributes = {
  .name = "Ins_TaskHandler",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for WatchDog_TaskHa */
osThreadId_t WatchDog_TaskHaHandle;
const osThreadAttr_t WatchDog_TaskHa_attributes = {
  .name = "WatchDog_TaskHa",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Shoot_TaskHandl */
osThreadId_t Shoot_TaskHandlHandle;
const osThreadAttr_t Shoot_TaskHandl_attributes = {
  .name = "Shoot_TaskHandl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for Remote_TaskHand */
osThreadId_t Remote_TaskHandHandle;
const osThreadAttr_t Remote_TaskHand_attributes = {
  .name = "Remote_TaskHand",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for Gimbal_TaskHand */
osThreadId_t Gimbal_TaskHandHandle;
const osThreadAttr_t Gimbal_TaskHand_attributes = {
  .name = "Gimbal_TaskHand",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for Referee_TaskHa */
osThreadId_t Referee_TaskHaHandle;
const osThreadAttr_t Referee_TaskHa_attributes = {
  .name = "Referee_TaskHa",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for SoftTimer */
osTimerId_t SoftTimerHandle;
const osTimerAttr_t SoftTimer_attributes = {
  .name = "SoftTimer"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Init_Task(void *argument);
void Comm_Task(void *argument);
void Ins_Task(void *argument);
void WatchDog_Task(void *argument);
void Shoot_Task(void *argument);
void Remote_Task(void *argument);
void Gimbal_Task(void *argument);
void Referee_Task(void *argument);
void SoftTimerCallback(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of SoftTimer */
  SoftTimerHandle = osTimerNew(SoftTimerCallback, osTimerPeriodic, NULL, &SoftTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Init_TaskHandle */
  Init_TaskHandleHandle = osThreadNew(Init_Task, NULL, &Init_TaskHandle_attributes);

  /* creation of Comm_TaskHandle */
  Comm_TaskHandleHandle = osThreadNew(Comm_Task, NULL, &Comm_TaskHandle_attributes);

  /* creation of Ins_TaskHandler */
  Ins_TaskHandlerHandle = osThreadNew(Ins_Task, NULL, &Ins_TaskHandler_attributes);

  /* creation of WatchDog_TaskHa */
  WatchDog_TaskHaHandle = osThreadNew(WatchDog_Task, NULL, &WatchDog_TaskHa_attributes);

  /* creation of Shoot_TaskHandl */
  Shoot_TaskHandlHandle = osThreadNew(Shoot_Task, NULL, &Shoot_TaskHandl_attributes);

  /* creation of Remote_TaskHand */
  Remote_TaskHandHandle = osThreadNew(Remote_Task, NULL, &Remote_TaskHand_attributes);

  /* creation of Gimbal_TaskHand */
  Gimbal_TaskHandHandle = osThreadNew(Gimbal_Task, NULL, &Gimbal_TaskHand_attributes);

  /* creation of Referee_TaskHa */
  Referee_TaskHaHandle = osThreadNew(Referee_Task, NULL, &Referee_TaskHa_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Init_Task */
/**
  * @brief  Function implementing the Init_TaskHandle thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Init_Task */
__weak void Init_Task(void *argument)
{
  /* USER CODE BEGIN Init_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Init_Task */
}

/* USER CODE BEGIN Header_Comm_Task */
/**
* @brief Function implementing the Comm_TaskHandle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Comm_Task */
__weak void Comm_Task(void *argument)
{
  /* USER CODE BEGIN Comm_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Comm_Task */
}

/* USER CODE BEGIN Header_Ins_Task */
/**
* @brief Function implementing the Ins_TaskHandler thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Ins_Task */
__weak void Ins_Task(void *argument)
{
  /* USER CODE BEGIN Ins_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Ins_Task */
}

/* USER CODE BEGIN Header_WatchDog_Task */
/**
* @brief Function implementing the WatchDog_TaskHa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_WatchDog_Task */
__weak void WatchDog_Task(void *argument)
{
  /* USER CODE BEGIN WatchDog_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END WatchDog_Task */
}

/* USER CODE BEGIN Header_Shoot_Task */
/**
* @brief Function implementing the Shoot_TaskHandl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Shoot_Task */
__weak void Shoot_Task(void *argument)
{
  /* USER CODE BEGIN Shoot_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Shoot_Task */
}

/* USER CODE BEGIN Header_Remote_Task */
/**
* @brief Function implementing the Remote_TaskHand thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Remote_Task */
__weak void Remote_Task(void *argument)
{
  /* USER CODE BEGIN Remote_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Remote_Task */
}

/* USER CODE BEGIN Header_Gimbal_Task */
/**
* @brief Function implementing the Gimbal_TaskHand thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Gimbal_Task */
__weak void Gimbal_Task(void *argument)
{
  /* USER CODE BEGIN Gimbal_Task */
  // Initialize CAN and Filter
  bsp_can_init();
  rc_init();
  osDelay(100);

  // Initialize Motors struct
  dm_motor_init();
  osDelay(100);

  // Enable Motors: 先清错再使能，间隔拉大防丢帧，失败重试
  for(int retry = 0; retry < 3; retry++)
  {
      for(int i = 0; i < num; i++)
      {
          if (motor[i].id == 0x01 || motor[i].id == 0x04 || motor[i].id == 0x07 || motor[i].id == 0x0A)
          {
              dm_motor_clear_err(&hcan1, &motor[i]);
              osDelay(50);
              dm_motor_enable(&hcan1, &motor[i]);
              osDelay(50);
          }
      }
      osDelay(100);
  }

  // === Motor 1/2/3 MIT PD 参数 ===
  // Kp 降低减少振荡，Kd 拉满增加阻尼，重力靠积分补偿兜底
  float kp_m1 = 150.0f;    // 底部yaw DM4340（无重力，柔一点更稳）
  float kd_m1 = 5.0f;
  float kp_m2 = 200.0f;    // 大臂 8009P
  float kd_m2 = 5.0f;
  float kp_m3 = 150.0f;    // 小臂 8009P
  float kd_m3 = 5.0f;

  // === 夹爪参数 (Motor 4, MIT 模式) ===
  #define GRIPPER_HOLD    0
  #define GRIPPER_CLAMP   1
  #define GRIPPER_RELEASE 2
  float gripper_clamp_vel   =  2.0f;
  float gripper_release_vel = -2.0f;
  float gripper_kd          =  1.0f;
  float gripper_hold_kp     = 50.0f;
  float gripper_hold_kd     =  2.0f;
  float gripper_tor_limit   =  3.0f;

  // 等待电机回传第一帧真实位置
  bool init_flag = false;
  uint32_t timeout_cnt = 0;
  while (!init_flag && timeout_cnt < 200) {
      init_flag = true;
      for (int i = 0; i < num; i++) {
          if (motor[i].id == 0x01 || motor[i].id == 0x04 || motor[i].id == 0x07 || motor[i].id == 0x0A) {
              if (motor[i].para.state == 0) {
                  init_flag = false;
              }
          }
      }
      osDelay(5);
      timeout_cnt++;
  }
  osDelay(100);

  // PSI 目标位置从当前编码器位置启动（仅此一次读编码器，之后只做增量）
  float pos_ref_m1 = motor[Motor1].para.pos;
  float pos_ref_m2 = motor[Motor2].para.pos;
  float pos_ref_m3 = motor[Motor3].para.pos;
  float gripper_hold_pos = motor[Motor4].para.pos;

  // TIM8 已在 main.c 初始化，写入初始 MIT 保持指令后启动
  motor_cmd[0] = (motor_cmd_t){ pos_ref_m1, 0, kp_m1, kd_m1, 0 };
  motor_cmd[1] = (motor_cmd_t){ pos_ref_m2, 0, kp_m2, kd_m2, 0 };
  motor_cmd[2] = (motor_cmd_t){ pos_ref_m3, 0, kp_m3, kd_m3, 0 };
  motor_cmd[3] = (motor_cmd_t){ gripper_hold_pos, 0, gripper_hold_kp, gripper_hold_kd, 0 };
  motor_send_tim_start();

  osDelay(500);

  float int_m2 = 0.0f;
  float int_m3 = 0.0f;

  uint8_t last_s0 = 0;
  uint8_t gripper_state = GRIPPER_HOLD;
  bool gripper_overcurrent = false;

  for(;;)
  {
    // --- 1. 快照遥控器数据（一次性拷贝，防止 DMA 中途更新导致撕裂）---
    uint8_t s0  = rc.s[0];
    uint8_t s1  = rc.s[1];
    int16_t ch0 = rc.ch[0];
    int16_t ch1 = rc.ch[1];
    int16_t ch2 = rc.ch[2];
    int16_t ch3 = rc.ch[3];

    // 遥控器未连接时（s0=0）全部停
    if (s0 == 0) {
        chassis_set_current(0, 0, 0, 0);
        last_s0 = s0;
        osDelay(4);
        continue;
    }

    // --- 2. 获取夹爪状态 ---
    float pos_m4 = motor[Motor4].para.pos;
    float tor_m4 = motor[Motor4].para.tor;

    float target_vel_m1 = 0.0f;
    float target_vel_m2 = 0.0f;
    float target_vel_m3 = 0.0f;

    // --- 3. 切档同步 ---
    if (s0 != last_s0) {
        pos_ref_m1 = motor[Motor1].para.pos;
        pos_ref_m2 = motor[Motor2].para.pos;
        pos_ref_m3 = motor[Motor3].para.pos;
    }

    // --- 4. 模式分发（精确匹配，不用 else 兜底）---
    if (s0 == 1)
    {
      // ===== S[0]=1: 大臂/小臂 =====
      float v2 = (float)ch3;
      float v3 = (float)ch1;
      if (fabs(v2) < 20.0f) v2 = 0.0f;
      if (fabs(v3) < 20.0f) v3 = 0.0f;

      target_vel_m2 = v2 * 0.005f;
      target_vel_m3 = v3 * 0.005f;
    }
    else if (s0 == 2)
    {
      // ===== S[0]=2: 底座Yaw + 夹爪 =====
      float v1 = (float)ch0;
      if (fabs(v1) < 20.0f) v1 = 0.0f;
      target_vel_m1 = v1 * 0.008f;

      // 夹爪状态机
      uint8_t desired_grip = GRIPPER_HOLD;
      if (s1 == 1)      desired_grip = GRIPPER_CLAMP;
      else if (s1 == 2) desired_grip = GRIPPER_RELEASE;
      else               desired_grip = GRIPPER_HOLD;

      if (gripper_overcurrent) {
          if (desired_grip == GRIPPER_HOLD) gripper_overcurrent = false;
          desired_grip = GRIPPER_HOLD;
      }
      if (desired_grip != gripper_state) {
          if (desired_grip == GRIPPER_HOLD) gripper_hold_pos = pos_m4;
          gripper_state = desired_grip;
      }
      if ((gripper_state == GRIPPER_CLAMP || gripper_state == GRIPPER_RELEASE) &&
          fabs(tor_m4) > gripper_tor_limit) {
          gripper_hold_pos = pos_m4;
          gripper_state = GRIPPER_HOLD;
          gripper_overcurrent = true;
      }
    }
    else if (s0 == 3)
    {
      // ===== S[0]=3: 麦轮底盘 =====
      if (gripper_state != GRIPPER_HOLD) {
          gripper_hold_pos = pos_m4;
          gripper_state = GRIPPER_HOLD;
      }

      float vx_in = (float)ch1;   // 前后
      float vy_in = (float)ch0;   // 左右平移
      float wz_in = (float)ch2;   // 旋转
      if (fabs(vx_in) < 20.0f) vx_in = 0.0f;
      if (fabs(vy_in) < 20.0f) vy_in = 0.0f;
      if (fabs(wz_in) < 20.0f) wz_in = 0.0f;

      chassis_mecanum_calc(vx_in * 10.0f, vy_in * 10.0f, -wz_in * 8.0f, 5000.0f);
    }

    // 非底盘模式 → 停车
    if (s0 != 3) {
        chassis_set_current(0, 0, 0, 0);
    }

    // --- 3. 摇杆直接积分到位置（推就动，松就停，零惯性）---
    pos_ref_m1 += target_vel_m1 * 0.004f;
    pos_ref_m2 += target_vel_m2 * 0.004f;
    pos_ref_m3 += target_vel_m3 * 0.004f;

    // --- 4. 重力补偿 (Motor 2/3) ---
    float pos_m2 = motor[Motor2].para.pos;
    float pos_m3 = motor[Motor3].para.pos;
    float err_m2 = pos_ref_m2 - pos_m2;
    float err_m3 = pos_ref_m3 - pos_m3;
    float gi_m2 = (fabs(target_vel_m2) < 0.01f) ? 2.0f : 0.5f;
    float gi_m3 = (fabs(target_vel_m3) < 0.01f) ? 1.0f : 0.2f;
    int_m2 += err_m2 * gi_m2;
    int_m3 += err_m3 * gi_m3;
    if(int_m2 >  25.0f) int_m2 =  25.0f;
    if(int_m2 < -25.0f) int_m2 = -25.0f;
    if(int_m3 >  20.0f) int_m3 =  20.0f;
    if(int_m3 < -20.0f) int_m3 = -20.0f;

    // --- 5. 写入共享指令（TIM8 ISR 每250μs读取并发送）---
    motor_cmd[0] = (motor_cmd_t){ pos_ref_m1, 0, kp_m1, kd_m1, 0.0f };
    motor_cmd[1] = (motor_cmd_t){ pos_ref_m2, 0, kp_m2, kd_m2, int_m2 };
    motor_cmd[2] = (motor_cmd_t){ pos_ref_m3, 0, kp_m3, kd_m3, int_m3 };

    if (gripper_state == GRIPPER_CLAMP) {
        motor_cmd[3] = (motor_cmd_t){ 0, gripper_clamp_vel, 0, gripper_kd, 0 };
    } else if (gripper_state == GRIPPER_RELEASE) {
        motor_cmd[3] = (motor_cmd_t){ 0, gripper_release_vel, 0, gripper_kd, 0 };
    } else {
        motor_cmd[3] = (motor_cmd_t){ gripper_hold_pos, 0, gripper_hold_kp, gripper_hold_kd, 0 };
    }

    last_s0 = s0;
    osDelay(4);
  }
  /* USER CODE END Gimbal_Task */
}

/* USER CODE BEGIN Header_Referee_Task */
/**
* @brief Function implementing the Referee_TaskHa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Referee_Task */
__weak void Referee_Task(void *argument)
{
  /* USER CODE BEGIN Referee_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Referee_Task */
}

/* SoftTimerCallback function */
__weak void SoftTimerCallback(void *argument)
{
  /* USER CODE BEGIN SoftTimerCallback */

  /* USER CODE END SoftTimerCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

