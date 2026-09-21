/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "can.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "encoder.h"
#include "traction.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*==================================================================
 * 驱动板 B（曳引驱动）——工程说明
 *------------------------------------------------------------------
 * 分工：A 板 = 主控（按键/调度/门控/称重/显示）
 *       B 板 = 本工程，只负责升降，楼层权威在 B 板（编码器在这边）
 *
 * 硬件连接：
 *   PA8   TIM1_CH1 -> TB6612 PWMA   10kHz PWM
 *   PB0            -> TB6612 AIN1   方向位 1
 *   PB1            -> TB6612 AIN2   方向位 2
 *   PB5            -> TB6612 STBY   驱动器总使能
 *   PA0/PA1        -> 编码器 A/B 相 TIM2 四倍频
 *   PB8 / PB9      -> CAN1 RX/TX    125kbps（需 AFIO 重映射，见 can.c）
 *
 * 系统节拍：TIM3 = 10ms 中断
 *   中断里做  M法测速 + PWM软启动爬升 + 曳引状态机/位置闭环
 *   主循环做  CAN 发送（中断里绝不做会产生等待的操作）
 *
 * 串口已按要求全部移除，调试靠 CAN：
 *   CAN_LOOPBACK_TEST=1 + BENCH_AUTO_GOTO=1 → 单板不开 A 板也能跑通整条闭环
 *==================================================================*/

/* USER CODE END 0 */

/**
  * @brief  10 ms 节拍（TIM3 更新中断）
  * @note   整个实时链路都挂在这个中断里：
  *         M 法测速 → PWM 软启动爬升 → 曳引状态机/位置闭环
  *         顺序不能换：占空比和方向的更新要用到本拍刚算出的速度/位置
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    Encoder_Update();   /* 10 ms：编码器测速 + 累计计数 */
    Motor_Loop();       /* 10 ms：PWM 软启动爬升 */
    Traction_Tick();    /* 10 ms：曳引状态机 + 位置闭环 + 置上报标志 */
    Traction_DbgSync(); /* 刷新调试观测点 g_dbg，不参与控制 */
  }
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */

  /* ① 电机驱动：内部顺序是 PWM 0% -> 拉高 STBY，杜绝上电瞬间猛冲 */
  Motor_Init();

  /* ② 编码器接口（TIM2，四倍频） */
  Encoder_Init();

  /* ③ 曳引状态机：上电默认认作停在 1F，位置清零 */
  Traction_Init();

  /* ④ CAN：滤波器 + 启动 + 接收中断
        ❗必须在 Traction_Init() 之后，否则中断进来的命令会被覆盖 */
  Can_Init();

  /* ⑤ 10ms 节拍：测速 + PWM 软启动 + 曳引位置闭环 */
  HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* CAN 发送全部集中在这里：中断只置"需要上报"标志，
       由主循环真正发到总线上，避免中断里等待发送完成 */
    Traction_Poll();

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
