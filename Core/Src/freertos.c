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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SoftTimer */
osTimerId_t SoftTimerHandle;
const osTimerAttr_t SoftTimer_attributes = {
  .name = "SoftTimer"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
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

  // === Motor 1/2/3 PD 参数 ===
  float kp_m1 = 300.0f;    // 底部yaw DM4340
  float kd_m1 = 4.0f;
  float kp_m2 = 450.0f;    // 大臂 8009P
  float kd_m2 = 5.0f;
  float kp_m3 = 350.0f;    // 小臂 8009P
  float kd_m3 = 4.0f;

  // === 夹爪参数 (DM4310) ===
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

  // 读取初始位置
  float pos_ref_m1 = motor[Motor1].para.pos;
  float pos_ref_m2 = motor[Motor2].para.pos;
  float pos_ref_m3 = motor[Motor3].para.pos;
  float gripper_hold_pos = motor[Motor4].para.pos;

  // 初始化 TIM2 250μs 定时发送，并写入初始保持指令
  motor_cmd[0] = (motor_cmd_t){ pos_ref_m1, 0, kp_m1, kd_m1, 0 };
  motor_cmd[1] = (motor_cmd_t){ pos_ref_m2, 0, kp_m2, kd_m2, 0 };
  motor_cmd[2] = (motor_cmd_t){ pos_ref_m3, 0, kp_m3, kd_m3, 0 };
  motor_cmd[3] = (motor_cmd_t){ gripper_hold_pos, 0, gripper_hold_kp, gripper_hold_kd, 0 };
  motor_send_tim_init();
  motor_send_tim_start();

  osDelay(500);

  float smooth_vel_m1 = 0.0f;
  float smooth_vel_m2 = 0.0f;
  float smooth_vel_m3 = 0.0f;
  float int_m2 = 0.0f;
  float int_m3 = 0.0f;

  uint8_t last_s0 = 0;
  uint8_t gripper_state = GRIPPER_HOLD;
  bool gripper_overcurrent = false;

  // 主任务 ~250Hz：只算目标值，CAN 发送由 TIM2 ISR 均匀完成
  for(;;)
  {
    // --- 1. 获取电机当前状态 ---
    float pos_m1 = motor[Motor1].para.pos;
    float pos_m2 = motor[Motor2].para.pos;
    float pos_m3 = motor[Motor3].para.pos;
    float pos_m4 = motor[Motor4].para.pos;
    float tor_m4 = motor[Motor4].para.tor;

    float target_vel_m1 = 0.0f;
    float target_vel_m2 = 0.0f;
    float target_vel_m3 = 0.0f;

    // --- 2. 模式分发 ---
    if (rc.s[0] == 1)
    {
      if (last_s0 != 1) {
          pos_ref_m1 = pos_m1;
          pos_ref_m2 = pos_m2;
          pos_ref_m3 = pos_m3;
          smooth_vel_m1 = 0.0f;
          smooth_vel_m2 = 0.0f;
          smooth_vel_m3 = 0.0f;
      }
      float ch3_val = (float)rc.ch[3];
      float ch1_val = (float)rc.ch[1];
      if (fabs(ch3_val) < 20.0f) ch3_val = 0.0f;
      if (fabs(ch1_val) < 20.0f) ch1_val = 0.0f;

      target_vel_m2 = ch3_val * 0.005f;
      target_vel_m3 = ch1_val * 0.005f;
    }
    else if (rc.s[0] == 2)
    {
      if (last_s0 != 2) {
          pos_ref_m1 = pos_m1;
          pos_ref_m2 = pos_m2;
          pos_ref_m3 = pos_m3;
          smooth_vel_m1 = 0.0f;
          smooth_vel_m2 = 0.0f;
          smooth_vel_m3 = 0.0f;
      }
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
      if (last_s0 != 3) {
          pos_ref_m1 = pos_m1;
          pos_ref_m2 = pos_m2;
          pos_ref_m3 = pos_m3;
          smooth_vel_m1 = 0.0f;
          smooth_vel_m2 = 0.0f;
          smooth_vel_m3 = 0.0f;
      }
      if (gripper_state != GRIPPER_HOLD) {
          gripper_hold_pos = pos_m4;
          gripper_state = GRIPPER_HOLD;
      }
    }

    // --- 3. 速度平滑 + 位置积分 (4ms 周期) ---
    smooth_vel_m1 += 0.04f * (target_vel_m1 - smooth_vel_m1);
    smooth_vel_m2 += 0.04f * (target_vel_m2 - smooth_vel_m2);
    smooth_vel_m3 += 0.04f * (target_vel_m3 - smooth_vel_m3);

    float vel_out_m1 = (fabs(smooth_vel_m1) < 0.01f) ? 0.0f : smooth_vel_m1;
    float vel_out_m2 = (fabs(smooth_vel_m2) < 0.01f) ? 0.0f : smooth_vel_m2;
    float vel_out_m3 = (fabs(smooth_vel_m3) < 0.01f) ? 0.0f : smooth_vel_m3;

    pos_ref_m1 += vel_out_m1 * 0.004f;
    pos_ref_m2 += vel_out_m2 * 0.004f;
    pos_ref_m3 += vel_out_m3 * 0.004f;

    // --- 4. 重力补偿 (Motor 2/3, 增益x2补偿降频) ---
    float err_m2 = pos_ref_m2 - pos_m2;
    float err_m3 = pos_ref_m3 - pos_m3;
    float gi_m2 = (fabs(vel_out_m2) < 0.02f) ? 1.0f : 0.2f;
    float gi_m3 = (fabs(vel_out_m3) < 0.02f) ? 1.0f : 0.2f;
    int_m2 += err_m2 * gi_m2;
    int_m3 += err_m3 * gi_m3;
    if(int_m2 >  25.0f) int_m2 =  25.0f;
    if(int_m2 < -25.0f) int_m2 = -25.0f;
    if(int_m3 >  20.0f) int_m3 =  20.0f;
    if(int_m3 < -20.0f) int_m3 = -20.0f;

    // --- 5. 写入共享指令缓冲（TIM2 ISR 每250μs 读取并发送）---
    motor_cmd[0] = (motor_cmd_t){ pos_ref_m1, vel_out_m1, kp_m1, kd_m1, 0.0f };
    motor_cmd[1] = (motor_cmd_t){ pos_ref_m2, vel_out_m2, kp_m2, kd_m2, int_m2 };
    motor_cmd[2] = (motor_cmd_t){ pos_ref_m3, vel_out_m3, kp_m3, kd_m3, int_m3 };

    if (gripper_state == GRIPPER_CLAMP) {
        motor_cmd[3] = (motor_cmd_t){ 0, gripper_clamp_vel, 0, gripper_kd, 0 };
    } else if (gripper_state == GRIPPER_RELEASE) {
        motor_cmd[3] = (motor_cmd_t){ 0, gripper_release_vel, 0, gripper_kd, 0 };
    } else {
        motor_cmd[3] = (motor_cmd_t){ gripper_hold_pos, 0, gripper_hold_kp, gripper_hold_kd, 0 };
    }

    last_s0 = rc.s[0];
    osDelay(4); // 250Hz 主任务，ISR 1kHz/电机
  }
  /* USER CODE END StartDefaultTask */
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

