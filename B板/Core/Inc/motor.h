/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor.h
  * @brief   TB6612FNG 直流电机驱动（JGA25-370）
  *
  * 接线：
  *   PA8  TIM1_CH1 -> PWMA   10 kHz PWM，占空比 = 速度指令
  *   PB0           -> AIN1   方向位 1
  *   PB1           -> AIN2   方向位 2
  *   PB5           -> STBY   驱动器总使能，低电平整体关闭
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_H__
#define __MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/

/** @brief 电机转向 */
typedef enum
{
  MOTOR_DIR_FORWARD = 0,   /*!< 正转：AIN1=1, AIN2=0 */
  MOTOR_DIR_REVERSE = 1    /*!< 反转：AIN1=0, AIN2=1 */
} MotorDir_t;

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/** @brief TIM1 ARR+1 = 7200，PWM 频率 = 72 MHz / 7200 = 10 kHz */
#define MOTOR_PWM_PERIOD        7200u

/** @brief 软启动步进：每 10 ms 上升 2%，0% → 100% 约 500 ms */
#define MOTOR_RAMP_STEP         2u

/** @brief 占空比上限（百分比） */
#define MOTOR_DUTY_MAX          100u

/* USER CODE END EC */

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  上电安全初始化：先 PWM 0%、方向脚拉低，最后使能驱动器
  * @note   必须按硬件接线总表第三节的上电顺序调用：
  *         GPIO 初始化(STBY=0) -> PWM CCR1=0 -> STBY=1 -> CCR1 从 0 缓慢爬升
  */
void       Motor_Init(void);

/** @brief 拉高 STBY，使能驱动器 */
void       Motor_Enable(void);

/** @brief 拉低 STBY，驱动器整体关闭（电机自由停车） */
void       Motor_Disable(void);

/** @brief 驱动器当前是否使能 */
uint8_t    Motor_IsEnabled(void);

/** @brief 设置转向 */
void       Motor_SetDir(MotorDir_t dir);

/** @brief 读取当前转向 */
MotorDir_t Motor_GetDir(void);

/**
  * @brief  设置目标占空比（速度指令）
  * @param  percent 0 ~ 100，超出自动截断
  * @note   占空比 > 0 时自动使能驱动器；实际 CCR 由 Motor_Loop() 软启动爬升
  */
void       Motor_SetDuty(uint16_t percent);

/** @brief 目标占空比（%） */
uint16_t   Motor_GetTargetDuty(void);

/** @brief 实际占空比（%，软启动过程中的当前值） */
uint16_t   Motor_GetDuty(void);

/** @brief 刹车：占空比清零 + AIN1=AIN2=1（电机绕组短路制动） */
void       Motor_Brake(void);

/** @brief 自由停车：占空比清零 + AIN1=AIN2=0 */
void       Motor_Coast(void);

/**
  * @brief  周期任务，在 10 ms 定时中断中调用
  * @note   负责把 CCR 从当前值按 MOTOR_RAMP_STEP 平滑爬升到目标值
  */
void       Motor_Loop(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H__ */
