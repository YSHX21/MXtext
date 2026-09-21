/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor.c
  * @brief   TB6612FNG 直流电机驱动实现
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "motor.h"
#include "tim.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* Private variables ---------------------------------------------------------*/

static MotorDir_t s_dir         = MOTOR_DIR_FORWARD;  /*!< 当前转向 */
static uint16_t   s_target_duty = 0;                  /*!< 目标占空比 % */
static uint16_t   s_duty        = 0;                  /*!< 当前占空比 % */
static uint8_t    s_enabled     = 0;                  /*!< STBY 状态 */
static uint8_t    s_brake       = 0;                  /*!< 刹车标志 */

/* Private function prototypes -----------------------------------------------*/

static void Motor_ApplyDir(void);
static void Motor_ApplyDuty(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  把当前转向写到 AIN1 / AIN2
  * @note   TB6612 真值表：IN1=IN2 时为刹车/停止，一高一低才有输出
  */
static void Motor_ApplyDir(void)
{
  if (s_dir == MOTOR_DIR_FORWARD)
  {
    HAL_GPIO_WritePin(AN1_GPIO_Port, AN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AN2_GPIO_Port, AN2_Pin, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(AN1_GPIO_Port, AN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AN2_GPIO_Port, AN2_Pin, GPIO_PIN_SET);
  }
}

/** @brief 把当前占空比写到 TIM1_CH1 的比较寄存器 */
static void Motor_ApplyDuty(void)
{
  uint32_t ccr = ((uint32_t)s_duty * MOTOR_PWM_PERIOD) / MOTOR_DUTY_MAX;

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr);
}

/* Exported functions --------------------------------------------------------*/

void Motor_Init(void)
{
  /* ① 方向脚与 STBY 全部拉低：驱动器关闭，输出高阻，电机绝对不动 */
  HAL_GPIO_WritePin(AN1_GPIO_Port, AN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(AN2_GPIO_Port, AN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);

  /* ② PWM 以 0% 占空比启动，避免上电瞬间电机猛冲 */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  s_dir         = MOTOR_DIR_FORWARD;
  s_target_duty = 0;
  s_duty        = 0;
  s_brake       = 0;
  Motor_ApplyDir();

  /* ③ CCR1 已经是 0，此时再使能驱动器 */
  Motor_Enable();
}

void Motor_Enable(void)
{
  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
  s_enabled = 1;
}

void Motor_Disable(void)
{
  /* 先清占空比再关 STBY，防止残余命令在下次使能时突然启动 */
  s_target_duty = 0;
  s_duty        = 0;
  Motor_ApplyDuty();

  HAL_GPIO_WritePin(AN1_GPIO_Port, AN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(AN2_GPIO_Port, AN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);

  s_enabled = 0;
  s_brake   = 0;
}

uint8_t Motor_IsEnabled(void)
{
  return s_enabled;
}

void Motor_SetDir(MotorDir_t dir)
{
  s_dir = dir;

  if (s_brake == 0u)
  {
    Motor_ApplyDir();
  }
}

MotorDir_t Motor_GetDir(void)
{
  return s_dir;
}

void Motor_SetDuty(uint16_t percent)
{
  if (percent > MOTOR_DUTY_MAX)
  {
    percent = MOTOR_DUTY_MAX;
  }

  s_target_duty = percent;

  if (percent > 0u)
  {
    /* 关键：只要给了速度指令就重新写一次方向脚。
       Motor_Disable() / Motor_Coast() 会把 AIN1、AIN2 都置 0，
       Motor_Brake() 会把它们都置 1 —— 这两情况下 TB6612 输出都是关闭的。
       若这里不恢复，即使 STBY=1、PWM 也有占空比，电机照样不转。 */
    s_brake = 0;
    Motor_ApplyDir();

    /* 有速度指令时自动使能驱动器，省去手动发 E */
    Motor_Enable();
  }
}

uint16_t Motor_GetTargetDuty(void)
{
  return s_target_duty;
}

uint16_t Motor_GetDuty(void)
{
  return s_duty;
}

void Motor_Brake(void)
{
  s_target_duty = 0;
  s_duty        = 0;
  Motor_ApplyDuty();

  HAL_GPIO_WritePin(AN1_GPIO_Port, AN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(AN2_GPIO_Port, AN2_Pin, GPIO_PIN_SET);

  s_brake = 1;
}

void Motor_Coast(void)
{
  s_target_duty = 0;
  s_duty        = 0;
  Motor_ApplyDuty();

  HAL_GPIO_WritePin(AN1_GPIO_Port, AN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(AN2_GPIO_Port, AN2_Pin, GPIO_PIN_RESET);

  s_brake = 0;
}

void Motor_Loop(void)
{
  if (s_duty < s_target_duty)
  {
    /* 升速：按步长缓慢爬升（软启动，抑制堵转电流冲击） */
    uint16_t next = (uint16_t)(s_duty + MOTOR_RAMP_STEP);

    s_duty = (next > s_target_duty) ? s_target_duty : next;
    Motor_ApplyDuty();
  }
  else if (s_duty > s_target_duty)
  {
    /* 降速/停止：立即生效，不拖泥带水 */
    s_duty = s_target_duty;
    Motor_ApplyDuty();
  }
  else
  {
    /* 已到位，无需动作 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
