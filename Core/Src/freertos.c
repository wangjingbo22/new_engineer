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

  // Enable Motors (ID 2, 3 and 4 mapped to array index Motor2, Motor3 and Motor4)
  for(int i = 0; i < num; i++)
  {
      if (motor[i].id == 1 || motor[i].id == 2 || motor[i].id == 3 || motor[i].id == 4)
      {
          dm_motor_enable(&hcan1, &motor[i]);
          osDelay(5);
      }
  }

  // Initial Control Parameters
  // 此处根据不同关节负载解耦 Kp/Kd
  float kp_m1 = 300.0f;    // 底部yaw轴 DM4340
  float kd_m1 = 4.0f;

  float kp_m2 = 450.0f;    // 提高刚度，配合积分项强力锁死
  float kd_m2 = 5.0f;

  float kp_m3 = 350.0f;    // 提高小臂刚度
  float kd_m3 = 4.0f;

  float kp_m4 = 100.0f;     // 末端夹爪 4310
  float kd_m4 = 3.0f;

  // 上电后给电机一点时间回传第一帧真实位置，防止默认0.0导致直接暴走归零
    bool init_flag = false;
    uint32_t timeout_cnt = 0;
    while (!init_flag && timeout_cnt < 200) {
        init_flag = true;
        for (int i = 0; i < num; i++) {
            if (motor[i].id == 1 || motor[i].id == 2 || motor[i].id == 3 || motor[i].id == 4) {
                if (motor[i].para.state == 0) { // 等待所有生效电机收到至少一次反馈
                    init_flag = false;
                }
            }
        }
        osDelay(5);
        timeout_cnt++;
    }
    osDelay(100);
  float pos_ref_m1 = 0.0f;
  float pos_ref_m2 = 0.0f;
  float pos_ref_m3 = 0.0f;
  float pos_ref_m4 = 0.0f;

  for(int i = 0; i < num; i++)
  {
    if (motor[i].id == 1) pos_ref_m1 = motor[i].para.pos;
    if (motor[i].id == 2) pos_ref_m2 = motor[i].para.pos;
    if (motor[i].id == 3) pos_ref_m3 = motor[i].para.pos;
    if (motor[i].id == 4) pos_ref_m4 = motor[i].para.pos;
  }

   osDelay(500);

    float smooth_vel_m1 = 0.0f;
    float smooth_vel_m2 = 0.0f;
    float smooth_vel_m3 = 0.0f;
    float smooth_vel_m4 = 0.0f;

    float int_m2 = 0.0f;
    float int_m3 = 0.0f;

    uint8_t last_s0 = 0;
    uint8_t last_s1 = 0;
    /* Infinite loop */
    for(;;)
    {
    // --- 1. 获取电机当前角度 ---
    float pos_m1 = 0.0f;
    float pos_m2 = 0.0f;
    float pos_m3 = 0.0f;
    float pos_m4 = 0.0f;
    for(int i = 0; i < num; i++)
    {
      if (motor[i].id == 1) pos_m1 = motor[i].para.pos;
      if (motor[i].id == 2) pos_m2 = motor[i].para.pos;
      if (motor[i].id == 3) pos_m3 = motor[i].para.pos;
      if (motor[i].id == 4) pos_m4 = motor[i].para.pos;
    }

    float target_vel_m1 = 0.0f;
    float target_vel_m2 = 0.0f;
    float target_vel_m3 = 0.0f;
    float target_vel_m4 = 0.0f;

    if (rc.s[0] == 1 && rc.s[1] == 1)
      {
        // 档位切换时平滑接管，同时清零重力积分补偿
        if (!(last_s0 == 1 && last_s1 == 1)) {
            for(int i = 0; i < num; i++) {
                if (motor[i].id == 2) { pos_ref_m2 = motor[i].para.pos; int_m2 = 0.0f; }
                if (motor[i].id == 3) { pos_ref_m3 = motor[i].para.pos; int_m3 = 0.0f; }
            }
        }
        
      float ch3_val = (float)rc.ch[3];
      float ch1_val = (float)rc.ch[1];
      if (fabs(ch3_val) < 20.0f) ch3_val = 0.0f;
      if (fabs(ch1_val) < 20.0f) ch1_val = 0.0f;

      target_vel_m2 = ch3_val * 0.02f; 
      target_vel_m3 = ch1_val * 0.02f; 
    }
    else if (rc.s[0] == 2 && rc.s[1] == 2)
      {
        if (!(last_s0 == 2 && last_s1 == 2)) {
            for(int i = 0; i < num; i++) {
                if (motor[i].id == 1) pos_ref_m1 = motor[i].para.pos;
                if (motor[i].id == 4) pos_ref_m4 = motor[i].para.pos;
            }
        }

      float ch0_val = (float)rc.ch[0];
      float ch3_val = (float)rc.ch[3];
      if (fabs(ch0_val) < 20.0f) ch0_val = 0.0f;
      if (fabs(ch3_val) < 20.0f) ch3_val = 0.0f;

      target_vel_m1 = ch0_val * 0.02f;  // 底部yaw
      target_vel_m4 = ch3_val * 0.05f;  // 夹爪
    }

    // --- 一阶低通滤波：彻底打平 14ms 的遥控器信号阶跃 ---
    // 0.02f 的系数能在 100ms 内平滑完成响应，让运动彻底告别卡顿
    smooth_vel_m1 += 0.02f * (target_vel_m1 - smooth_vel_m1);
    smooth_vel_m2 += 0.02f * (target_vel_m2 - smooth_vel_m2);
    smooth_vel_m3 += 0.02f * (target_vel_m3 - smooth_vel_m3);
    smooth_vel_m4 += 0.02f * (target_vel_m4 - smooth_vel_m4);

    // 完全归中时严格清0，防止微小蠕动
    float vel_out_m1 = (fabs(smooth_vel_m1) < 0.01f) ? 0.0f : smooth_vel_m1;
    float vel_out_m2 = (fabs(smooth_vel_m2) < 0.01f) ? 0.0f : smooth_vel_m2;
    float vel_out_m3 = (fabs(smooth_vel_m3) < 0.01f) ? 0.0f : smooth_vel_m3;
    float vel_out_m4 = (fabs(smooth_vel_m4) < 0.01f) ? 0.0f : smooth_vel_m4;

    // 更新位置 (每 2ms = 0.002s 执行一次)
    pos_ref_m1 += vel_out_m1 * 0.002f;
    pos_ref_m2 += vel_out_m2 * 0.002f;
    pos_ref_m3 += vel_out_m3 * 0.002f;
    pos_ref_m4 += vel_out_m4 * 0.002f;

    // --- 锁止积分补偿：仅在停止期间进行防重力抗下坠 ---
    float err_m2 = pos_ref_m2 - pos_m2;
    float err_m3 = pos_ref_m3 - pos_m3;

    // 为了防止移动期间误差过大导致积分器风饱和，仅当速度为 0 时引入重力补偿
    if (fabs(vel_out_m2) < 0.02f) {
        int_m2 += err_m2 * 0.08f;
    }
    if (fabs(vel_out_m3) < 0.02f) {
        int_m3 += err_m3 * 0.08f;
    }

    // 限制抗重力积分上限
    if(int_m2 > 12.0f) int_m2 = 12.0f;
    if(int_m2 < -12.0f) int_m2 = -12.0f;
    if(int_m3 > 10.0f) int_m3 = 10.0f;
    if(int_m3 < -10.0f) int_m3 = -10.0f;

    float tor_ff_m2 = int_m2;
    float tor_ff_m3 = int_m3;

    // --- 2. 指令下发 ---
    for(int i = 0; i < num; i++)
    {
      if (motor[i].id == 1)
      {
        mit_ctrl(&hcan1, &motor[i], motor[i].id, pos_ref_m1, vel_out_m1, kp_m1, kd_m1, 0.0f);
      }
      else if (motor[i].id == 2)
      {
        mit_ctrl(&hcan1, &motor[i], motor[i].id, pos_ref_m2, vel_out_m2, kp_m2, kd_m2, tor_ff_m2);
      }
      else if (motor[i].id == 3)
      {
        mit_ctrl(&hcan1, &motor[i], motor[i].id, pos_ref_m3, vel_out_m3, kp_m3, kd_m3, tor_ff_m3);
      }
      else if (motor[i].id == 4)
      {
        mit_ctrl(&hcan1, &motor[i], motor[i].id, pos_ref_m4, vel_out_m4, kp_m4, kd_m4, 0.0f);
      }
    }

    last_s0 = rc.s[0];
    last_s1 = rc.s[1];
    osDelay(2); // 500Hz control loop
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

