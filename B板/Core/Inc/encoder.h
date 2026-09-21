/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    encoder.h
  * @brief   TIM2 正交编码器测速（JGA25-370，11 PPR，减速比 377:1）
  *
  * 接线：PA0  TIM2_CH1 -> 编码器 A 相
  *       PA1  TIM2_CH2 -> 编码器 B 相
  *       TIM2 工作在 Encoder_Interface / TI12 模式（A、B 两相均计数，四倍频）
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ENCODER_H__
#define __ENCODER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/** @brief 编码器线数（每转脉冲数） */
#define ENCODER_PPR             11u

/** @brief TIM2 TI12 模式双相双边沿计数，四倍频 */
#define ENCODER_MULTIPLIER      4u

/** @brief 减速比 377:1 */
#define GEAR_RATIO              377u

/** @brief 电机轴每转计数 = 11 × 4 = 44 */
#define COUNTS_PER_MOTOR_REV    (ENCODER_PPR * ENCODER_MULTIPLIER)

/** @brief 输出轴每转计数 = 44 × 377 = 16588 */
#define COUNTS_PER_OUTPUT_REV   (COUNTS_PER_MOTOR_REV * GEAR_RATIO)

/**
  * @brief 测速周期（ms），与 TIM3 中断周期一致
  * @note  M 法测速：统计固定时间内的脉冲数
  */
#define ENCODER_SAMPLE_MS       10u

/**
  * @brief 自检判定有效所需的最小每圈计数
  * @note  低于此值认为没收到真实脉冲（理论 2x = 8294，取 1000 足够宽松）
  */
#define ENCODER_CAL_MIN_PER_REV 1000

/* USER CODE END EC */

/* Exported types ------------------------------------------------------------*/

/** @brief 编码器自检（标定）结果 */
typedef struct
{
  int32_t  total;      /*!< 标定期间累计计数；符号 = 顺时针转动时的计数方向 */
  int32_t  per_rev;    /*!< 实测输出轴每圈计数（取绝对值） */
  uint16_t revs;       /*!< 标定时声明的转动圈数 */
  uint16_t flips;      /*!< 采样中方向反转的次数，用于判断信号是否被噪声干扰 */
} EncoderCal_t;

/* Exported functions prototypes ---------------------------------------------*/

/** @brief 启动 TIM2 编码器接口并把计数清零 */
void    Encoder_Init(void);

/** @brief 累计计数（带符号，正转递增/反转递减取决于 A/B 相序） */
int32_t Encoder_Read(void);

/** @brief 清零累计计数与测速状态 */
void    Encoder_Reset(void);

/**
  * @brief  周期任务，在 TIM3 的 10 ms 中断中调用
  * @note   M 法测速，结果可通过 Encoder_GetRpm() 读取
  */
void    Encoder_Update(void);

/** @brief 输出轴转速（rpm），带符号 */
int32_t Encoder_GetRpm(void);

/** @brief 上一个采样周期（10 ms）内的脉冲增量 */
int32_t Encoder_GetDelta(void);

/* ---------------- 自检 / 标定 ---------------- */

/**
  * @brief  开始自检：计数清零，并开始统计方向反转次数
  * @param  revs 接下来将要转动的圈数，0 视为 1
  */
void    Encoder_CalStart(uint16_t revs);

/** @brief 结束自检并结算结果 */
void    Encoder_CalStop(void);

/** @brief 自检是否进行中 */
uint8_t Encoder_CalIsRunning(void);

/** @brief 读取自检结果（未结束时返回上次结果） */
const EncoderCal_t *Encoder_CalGet(void);

/**
  * @brief  根据实测每圈计数判定倍频
  * @param  per_rev      实测输出轴每圈计数
  * @param  err_permille 输出相对理论值的误差，单位为千分比（可为 NULL）
  * @return 4 = 四倍频正常；2 = 只有一相在工作；0 = 脉冲太少，无法判定
  */
uint8_t Encoder_JudgeMultiplier(int32_t per_rev, int32_t *err_permille);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H__ */
