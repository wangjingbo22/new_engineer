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
  .stack_size = 128 * 4,
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

  // === PSI 模式参数 (Motor 1/2/3) ===
  float psi_max_vel_m1 = 3.0f;    // yaw 最大速度 rad/s
  float psi_max_cur_m1 = 3.0f;    // yaw 最大电流 A
  float psi_max_vel_m2 = 2.0f;    // 大臂 最大速度
  float psi_max_cur_m2 = 5.0f;    // 大臂 最大电流
  float psi_max_vel_m3 = 2.0f;    // 小臂 最大速度
  float psi_max_cur_m3 = 4.0f;    // 小臂 最大电流

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

  // TIM8 已在 main.c 初始化，写入初始保持指令后启动
  motor_cmd[0] = (motor_cmd_t){ pos_ref_m1, psi_max_vel_m1, 0, 0, psi_max_cur_m1 };
  motor_cmd[1] = (motor_cmd_t){ pos_ref_m2, psi_max_vel_m2, 0, 0, psi_max_cur_m2 };
  motor_cmd[2] = (motor_cmd_t){ pos_ref_m3, psi_max_vel_m3, 0, 0, psi_max_cur_m3 };
  motor_cmd[3] = (motor_cmd_t){ gripper_hold_pos, 0, gripper_hold_kp, gripper_hold_kd, 0 };
  motor_send_tim_start();

  osDelay(500);

  float smooth_vel_m1 = 0.0f;
  float smooth_vel_m2 = 0.0f;
  float smooth_vel_m3 = 0.0f;

  uint8_t last_s0 = 0;
  uint8_t gripper_state = GRIPPER_HOLD;
  bool gripper_overcurrent = false;

  for(;;)
  {
    // --- 1. 获取夹爪状态 ---
    float pos_m4 = motor[Motor4].para.pos;
    float tor_m4 = motor[Motor4].para.tor;

    float target_vel_m1 = 0.0f;
    float target_vel_m2 = 0.0f;
    float target_vel_m3 = 0.0f;

    // --- 2. 模式分发 ---
    uint8_t s0 = rc.s[0];
    if (s0 != last_s0) {
        pos_ref_m1 = motor[Motor1].para.pos;
        pos_ref_m2 = motor[Motor2].para.pos;
        pos_ref_m3 = motor[Motor3].para.pos;
        smooth_vel_m1 = 0.0f;
        smooth_vel_m2 = 0.0f;
        smooth_vel_m3 = 0.0f;
    }

    if (s0 == 1)
    {
      // ===== S[0]=1: 大臂/小臂 =====
      float ch3_val = (float)rc.ch[3];
      float ch1_val = (float)rc.ch[1];
      if (fabs(ch3_val) < 20.0f) ch3_val = 0.0f;
      if (fabs(ch1_val) < 20.0f) ch1_val = 0.0f;

      target_vel_m2 = ch3_val * 0.005f;
      target_vel_m3 = ch1_val * 0.005f;
    }
    else if (s0 == 2)
    {
      // ===== S[0]=2: 底座Yaw + 夹爪 =====
      float ch0_val = (float)rc.ch[0];
      if (fabs(ch0_val) < 20.0f) ch0_val = 0.0f;
      target_vel_m1 = ch0_val * 0.008f;

      // 夹爪状态机
      uint8_t desired_grip = GRIPPER_HOLD;
      if (rc.s[1] == 1)      desired_grip = GRIPPER_CLAMP;
      else if (rc.s[1] == 2) desired_grip = GRIPPER_RELEASE;
      else                    desired_grip = GRIPPER_HOLD;

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
    else
    {
      // ===== S[0]=3: 底盘（暂未实现）=====
      if (gripper_state != GRIPPER_HOLD) {
          gripper_hold_pos = pos_m4;
          gripper_state = GRIPPER_HOLD;
      }
    }

    // --- 3. 速度平滑 + 位置积分 ---
    smooth_vel_m1 += 0.04f * (target_vel_m1 - smooth_vel_m1);
    smooth_vel_m2 += 0.04f * (target_vel_m2 - smooth_vel_m2);
    smooth_vel_m3 += 0.04f * (target_vel_m3 - smooth_vel_m3);

    float vel_out_m1 = (fabs(smooth_vel_m1) < 0.01f) ? 0.0f : smooth_vel_m1;
    float vel_out_m2 = (fabs(smooth_vel_m2) < 0.01f) ? 0.0f : smooth_vel_m2;
    float vel_out_m3 = (fabs(smooth_vel_m3) < 0.01f) ? 0.0f : smooth_vel_m3;

    pos_ref_m1 += vel_out_m1 * 0.004f;
    pos_ref_m2 += vel_out_m2 * 0.004f;
    pos_ref_m3 += vel_out_m3 * 0.004f;

    // --- 4. 写入共享指令（TIM8 ISR 每250μs读取并发送）---
    motor_cmd[0] = (motor_cmd_t){ pos_ref_m1, psi_max_vel_m1, 0, 0, psi_max_cur_m1 };
    motor_cmd[1] = (motor_cmd_t){ pos_ref_m2, psi_max_vel_m2, 0, 0, psi_max_cur_m2 };
    motor_cmd[2] = (motor_cmd_t){ pos_ref_m3, psi_max_vel_m3, 0, 0, psi_max_cur_m3 };

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

