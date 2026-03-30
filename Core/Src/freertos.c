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
#include "tim.h"
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
volatile float g_servo_pulse = 1500.0f;  // 调试用全局变量，Watch窗口可实时查看
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

  // ====================== 初始化 ======================
  bsp_can_init();
  rc_init();

  // 舵机：立即启动PWM并发送中位，防止AF引脚低电平导致舵机冲限位
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 710);

  osDelay(100);
  dm_motor_init();
  osDelay(100);

  for (int retry = 0; retry < 3; retry++) {
      for (int i = 0; i < num; i++) {
          if (motor[i].id == 0x01 || motor[i].id == 0x04 ||
              motor[i].id == 0x07 || motor[i].id == 0x0A) {
              dm_motor_clear_err(&hcan1, &motor[i]);
              osDelay(50);
              dm_motor_enable(&hcan1, &motor[i]);
              osDelay(50);
          }
      }
      osDelay(100);
  }

  for (int w = 0; w < 200; w++) {
      bool ok = true;
      for (int i = 0; i < num; i++) {
          if (motor[i].id == 0x01 || motor[i].id == 0x04 ||
              motor[i].id == 0x07 || motor[i].id == 0x0A) {
              if (motor[i].para.state == 0) ok = false;
          }
      }
      if (ok) break;
      osDelay(5);
  }
  osDelay(200);

  // ====================== 参数（const 防误改）======================
  const float kp_m1 = 150.0f, kd_m1 = 5.0f;
  const float kp_m2 = 200.0f, kd_m2 = 5.0f;
  const float kp_m3 = 150.0f, kd_m3 = 5.0f;

  #define GRIPPER_HOLD    0
  #define GRIPPER_CLAMP   1                     
  #define GRIPPER_RELEASE 2
  const float gripper_clamp_vel   =  2.0f;
  const float gripper_release_vel = -2.0f;
  const float gripper_kd          =  1.0f;
  const float gripper_hold_kp     = 50.0f;
  const float gripper_hold_kd     =  2.0f;
  const float gripper_tor_limit   =  3.0f;

  const int16_t RC_DEADZONE = 30;     // 死区加大，过滤零漂和噪声
  const float POS_LIMIT     = 12.0f;  // 位置安全限幅（PMAX=12.5）

  // ====================== 状态 ======================
  float pos_ref_m1 = motor[Motor1].para.pos;
  float pos_ref_m2 = motor[Motor2].para.pos;
  float pos_ref_m3 = motor[Motor3].para.pos;
  float gripper_hold_pos = motor[Motor4].para.pos;
  float int_m2 = 0.0f, int_m3 = 0.0f;

  uint8_t last_s0 = 0;
  uint8_t gripper_state = GRIPPER_HOLD;
  bool gripper_overcurrent = false;

  #define SERVO_MIN     550    //低头视角
  #define SERVO_MAX     800    // 抬头视角
  #define SERVO_CENTER  710    // 上电基准位置
  #define SERVO_OFFSET  111     // 约10度对应脉宽偏移（us），待实测后修正
  #define SERVO_STEP    2.0f    // 每帧步进（us），控制切换速度
  float servo_pulse  = SERVO_CENTER;
  float servo_target = SERVO_CENTER;

  // RC 滤波（上一帧有效值）
  int16_t ch_prev[4] = {0, 0, 0, 0};

  // ====================== 主循环 500Hz 直接发送 ======================
  for (;;)
  {
    // -------- 1. RC 采集 + 校验 + 滤波 --------
    uint8_t s0 = rc.s[0];
    uint8_t s1 = rc.s[1];
    int16_t ch_raw[4] = { rc.ch[0], rc.ch[1], rc.ch[2], rc.ch[3] };

    // 校验：开关值必须 1/2/3，通道值必须 [-660, 660]
    bool rc_ok = (s0 >= 1 && s0 <= 3);
    for (int i = 0; i < 4; i++) {
        if (ch_raw[i] < -660 || ch_raw[i] > 660) rc_ok = false;
    }
    if (!rc_ok) {
        chassis_set_current(0, 0, 0, 0);
        grip_rot_set_current(0, 0);
        // DM电机不发新指令→电机维持上一帧状态
        last_s0 = 0;
        osDelay(2);
        continue;
    }

    // 滤波（取当前帧和上一帧平均）+ 死区
    int16_t ch[4];
    for (int i = 0; i < 4; i++) {
        int16_t val = (ch_raw[i] + ch_prev[i]) / 2;
        ch[i] = (val > -RC_DEADZONE && val < RC_DEADZONE) ? 0 : val;
        ch_prev[i] = ch_raw[i];
    }

    // -------- 2. 切档同步 --------
    if (s0 != last_s0 && last_s0 != 0) {
        pos_ref_m1 = motor[Motor1].para.pos;
        pos_ref_m2 = motor[Motor2].para.pos;
        pos_ref_m3 = motor[Motor3].para.pos;
        // 不清零 int_m2/int_m3，保留已积累的重力补偿，防止切档下坠
    }

    // -------- 3. 模式分发 --------
    float vel_m1 = 0, vel_m2 = 0, vel_m3 = 0;

    if (s0 == 1) {
        vel_m2 = -(float)ch[3] * 0.005f;
        vel_m3 = -(float)ch[1] * 0.005f;

        // S[1] 三档控制图传舵机俯仰角度
        // 调试阶段：s1=1持续增加脉宽，s1=2持续减小，找到合适位置后记录g_servo_pulse的值
        if      (s1 == 1) servo_target += SERVO_STEP;   // 持续向一侧移动
        else if (s1 == 2) servo_target -= SERVO_STEP;   // 持续向另一侧移动
        // s1=3：保持当前位置不动

        if (servo_target > SERVO_MAX) servo_target = SERVO_MAX;
        if (servo_target < SERVO_MIN) servo_target = SERVO_MIN;

        servo_pulse = servo_target;
        g_servo_pulse = servo_pulse;  // 同步全局变量，供Watch窗口实时查看
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)servo_pulse);
    }
    else if (s0 == 2) {
        vel_m1 = -(float)ch[0] * 0.008f;

        uint8_t desired = GRIPPER_HOLD;
        if (s1 == 1) desired = GRIPPER_CLAMP;
        else if (s1 == 2) desired = GRIPPER_RELEASE;

        if (gripper_overcurrent) {
            if (desired == GRIPPER_HOLD) gripper_overcurrent = false;
            desired = GRIPPER_HOLD;
        }
        if (desired != gripper_state) {
            if (desired == GRIPPER_HOLD) gripper_hold_pos = motor[Motor4].para.pos;
            gripper_state = desired;
        }
        if ((gripper_state == GRIPPER_CLAMP || gripper_state == GRIPPER_RELEASE) &&
            fabs(motor[Motor4].para.tor) > gripper_tor_limit) {
            gripper_hold_pos = motor[Motor4].para.pos;
            gripper_state = GRIPPER_HOLD;
            gripper_overcurrent = true;
        }
    }
    else if (s0 == 3) {
        if (gripper_state != GRIPPER_HOLD) {
            gripper_hold_pos = motor[Motor4].para.pos;
            gripper_state = GRIPPER_HOLD;
        }
        chassis_mecanum_calc(
            (float)ch[2] * 10.0f,    // vx 前后
            (float)ch[1] * 10.0f,    // vy 平移
            (float)ch[0] * 8.0f,     // wz 旋转
            5000.0f);
    }

    if (s0 != 3) chassis_set_current(0, 0, 0, 0);

    // -------- 4. 位置积分 + 限幅 --------
    pos_ref_m1 += vel_m1 * 0.002f;
    pos_ref_m2 += vel_m2 * 0.002f;
    pos_ref_m3 += vel_m3 * 0.002f;

    if (pos_ref_m1 >  POS_LIMIT) pos_ref_m1 =  POS_LIMIT;
    if (pos_ref_m1 < -POS_LIMIT) pos_ref_m1 = -POS_LIMIT;
    if (pos_ref_m2 >  POS_LIMIT) pos_ref_m2 =  POS_LIMIT;
    if (pos_ref_m2 < -POS_LIMIT) pos_ref_m2 = -POS_LIMIT;
    if (pos_ref_m3 >  POS_LIMIT) pos_ref_m3 =  POS_LIMIT;
    if (pos_ref_m3 < -POS_LIMIT) pos_ref_m3 = -POS_LIMIT;

    // -------- 5. 重力补偿 --------
    float err2 = pos_ref_m2 - motor[Motor2].para.pos;
    float err3 = pos_ref_m3 - motor[Motor3].para.pos;
    int_m2 += err2 * ((fabs(vel_m2) < 0.01f) ? 2.0f : 0.5f);
    int_m3 += err3 * ((fabs(vel_m3) < 0.01f) ? 2.0f : 0.5f);
    if (int_m2 >  25.0f) int_m2 =  25.0f;
    if (int_m2 < -25.0f) int_m2 = -25.0f;
    if (int_m3 >  20.0f) int_m3 =  20.0f;
    if (int_m3 < -20.0f) int_m3 = -20.0f;

    // -------- 6. DM 电机直接发送（无 ISR，零数据竞争）--------
    mit_ctrl(&hcan1, &motor[Motor1], motor[Motor1].id,
             pos_ref_m1, 0, kp_m1, kd_m1, 0);
    mit_ctrl(&hcan1, &motor[Motor2], motor[Motor2].id,
             pos_ref_m2, 0, kp_m2, kd_m2, int_m2);
    mit_ctrl(&hcan1, &motor[Motor3], motor[Motor3].id,
             pos_ref_m3, 0, kp_m3, kd_m3, int_m3);

    if (gripper_state == GRIPPER_CLAMP)
        mit_ctrl(&hcan1, &motor[Motor4], motor[Motor4].id,
                 0, gripper_clamp_vel, 0, gripper_kd, 0);
    else if (gripper_state == GRIPPER_RELEASE)
        mit_ctrl(&hcan1, &motor[Motor4], motor[Motor4].id,
                 0, gripper_release_vel, 0, gripper_kd, 0);
    else
        mit_ctrl(&hcan1, &motor[Motor4], motor[Motor4].id,
                 gripper_hold_pos, 0, gripper_hold_kp, gripper_hold_kd, 0);

    // -------- 7. M2006 夹爪旋转(ch3) / 上下(ch2) --------
    // 叠加控制：A = rot + ud，B = rot - ud
    if (s0 != 2) {
        grip_rot_set_current(0, 0);
    } else {
        float rot_cur = (float)ch[3] * 2.0f;   // 旋转
        float ud_cur  = (float)ch[2] * 0.0f ;  // 上下

        float cur_A = rot_cur + ud_cur;
        float cur_B = rot_cur - ud_cur;
        if (cur_A >  8000.0f) cur_A =  8000.0f;
        if (cur_A < -8000.0f) cur_A = -8000.0f;
        if (cur_B >  8000.0f) cur_B =  8000.0f;
        if (cur_B < -8000.0f) cur_B = -8000.0f;

        grip_rot_set_current((int16_t)cur_A, (int16_t)cur_B);
    }

    last_s0 = s0;
    osDelay(2);
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

