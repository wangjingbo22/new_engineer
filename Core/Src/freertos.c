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
  float gripper_clamp_vel   =  2.0f;   // 夹紧速度 rad/s
  float gripper_release_vel = -2.0f;   // 松开速度 rad/s
  float gripper_kd          =  1.0f;   // 夹爪运动阻尼
  float gripper_hold_kp     = 50.0f;   // 夹爪保持刚度
  float gripper_hold_kd     =  2.0f;   // 夹爪保持阻尼
  float gripper_tor_limit   =  3.0f;   // 过流保护阈值 Nm

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

  osDelay(500);

  float smooth_vel_m1 = 0.0f;
  float smooth_vel_m2 = 0.0f;
  float smooth_vel_m3 = 0.0f;
  float int_m2 = 0.0f;
  float int_m3 = 0.0f;

  uint8_t last_s0 = 0;
  uint8_t gripper_state = GRIPPER_HOLD;
  bool gripper_overcurrent = false;

  /* Infinite loop */
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
      // ===== S[0]=1: 大臂/小臂 位置积分控制 =====
      if (last_s0 != 1) {
          pos_ref_m1 = pos_m1;  // 全量捕获，防止yaw漂移
          pos_ref_m2 = pos_m2;
          pos_ref_m3 = pos_m3;
          smooth_vel_m1 = 0.0f; // 清残余速度，防止切档后继续滑动
          smooth_vel_m2 = 0.0f;
          smooth_vel_m3 = 0.0f;
      }
      float ch3_val = (float)rc.ch[3];
      float ch1_val = (float)rc.ch[1];
      if (fabs(ch3_val) < 20.0f) ch3_val = 0.0f;
      if (fabs(ch1_val) < 20.0f) ch1_val = 0.0f;

      target_vel_m2 = ch3_val * 0.005f;  // 缩小4倍，更精细
      target_vel_m3 = ch1_val * 0.005f;
    }
    else if (rc.s[0] == 2)
    {
      // ===== S[0]=2: 底座Yaw + 夹爪 =====
      if (last_s0 != 2) {
          pos_ref_m1 = pos_m1;
          pos_ref_m2 = pos_m2;  // 全量捕获
          pos_ref_m3 = pos_m3;
          smooth_vel_m1 = 0.0f;
          smooth_vel_m2 = 0.0f;
          smooth_vel_m3 = 0.0f;
      }
      float ch0_val = (float)rc.ch[0];
      if (fabs(ch0_val) < 20.0f) ch0_val = 0.0f;
      target_vel_m1 = ch0_val * 0.008f;  // yaw 稍快一点

      // --- 夹爪状态机 (S[1]控制) ---
      uint8_t desired_grip = GRIPPER_HOLD;
      if (rc.s[1] == 1)      desired_grip = GRIPPER_CLAMP;
      else if (rc.s[1] == 2) desired_grip = GRIPPER_RELEASE;
      else                    desired_grip = GRIPPER_HOLD;

      // 过流锁定：触发后强制保持，直到用户回到S[1]=3
      if (gripper_overcurrent) {
          if (desired_grip == GRIPPER_HOLD) {
              gripper_overcurrent = false;
          }
          desired_grip = GRIPPER_HOLD;
      }

      // 状态切换时捕获保持位置
      if (desired_grip != gripper_state) {
          if (desired_grip == GRIPPER_HOLD) {
              gripper_hold_pos = pos_m4;
          }
          gripper_state = desired_grip;
      }

      // 过流检测
      if ((gripper_state == GRIPPER_CLAMP || gripper_state == GRIPPER_RELEASE) &&
          fabs(tor_m4) > gripper_tor_limit) {
          gripper_hold_pos = pos_m4;
          gripper_state = GRIPPER_HOLD;
          gripper_overcurrent = true;
      }
    }
    else
    {
      // ===== S[0]=3: 底盘（暂未实现），所有关节保持 =====
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

    // --- 3. Motor 1/2/3 速度平滑 + 位置积分 ---
    smooth_vel_m1 += 0.02f * (target_vel_m1 - smooth_vel_m1);
    smooth_vel_m2 += 0.02f * (target_vel_m2 - smooth_vel_m2);
    smooth_vel_m3 += 0.02f * (target_vel_m3 - smooth_vel_m3);

    float vel_out_m1 = (fabs(smooth_vel_m1) < 0.01f) ? 0.0f : smooth_vel_m1;
    float vel_out_m2 = (fabs(smooth_vel_m2) < 0.01f) ? 0.0f : smooth_vel_m2;
    float vel_out_m3 = (fabs(smooth_vel_m3) < 0.01f) ? 0.0f : smooth_vel_m3;

    pos_ref_m1 += vel_out_m1 * 0.002f;
    pos_ref_m2 += vel_out_m2 * 0.002f;
    pos_ref_m3 += vel_out_m3 * 0.002f;

    // --- 4. 重力补偿 (Motor 2/3) ---
    // 全程生效：停止时快速积分，运动时慢速积分保持抗重力底线
    float err_m2 = pos_ref_m2 - pos_m2;
    float err_m3 = pos_ref_m3 - pos_m3;
    float gi_m2 = (fabs(vel_out_m2) < 0.02f) ? 0.5f : 0.1f;
    float gi_m3 = (fabs(vel_out_m3) < 0.02f) ? 0.5f : 0.1f;
    int_m2 += err_m2 * gi_m2;
    int_m3 += err_m3 * gi_m3;
    if(int_m2 >  25.0f) int_m2 =  25.0f;
    if(int_m2 < -25.0f) int_m2 = -25.0f;
    if(int_m3 >  20.0f) int_m3 =  20.0f;
    if(int_m3 < -20.0f) int_m3 = -20.0f;

    // --- 5. 指令下发 ---
    mit_ctrl(&hcan1, &motor[Motor1], motor[Motor1].id,
             pos_ref_m1, vel_out_m1, kp_m1, kd_m1, 0.0f);
    mit_ctrl(&hcan1, &motor[Motor2], motor[Motor2].id,
             pos_ref_m2, vel_out_m2, kp_m2, kd_m2, int_m2);
    mit_ctrl(&hcan1, &motor[Motor3], motor[Motor3].id,
             pos_ref_m3, vel_out_m3, kp_m3, kd_m3, int_m3);

    if (gripper_state == GRIPPER_CLAMP) {
        mit_ctrl(&hcan1, &motor[Motor4], motor[Motor4].id,
                 0.0f, gripper_clamp_vel, 0.0f, gripper_kd, 0.0f);
    } else if (gripper_state == GRIPPER_RELEASE) {
        mit_ctrl(&hcan1, &motor[Motor4], motor[Motor4].id,
                 0.0f, gripper_release_vel, 0.0f, gripper_kd, 0.0f);
    } else {
        mit_ctrl(&hcan1, &motor[Motor4], motor[Motor4].id,
                 gripper_hold_pos, 0.0f, gripper_hold_kp, gripper_hold_kd, 0.0f);
    }

    last_s0 = rc.s[0];
    osDelay(2); // 500Hz
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

