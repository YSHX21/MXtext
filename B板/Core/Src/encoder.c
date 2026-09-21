/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    encoder.c
  * @brief   TIM2 正交编码器测速实现（M 法）
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "encoder.h"
#include "tim.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* Private variables ---------------------------------------------------------*/

static int32_t s_total = 0;   /*!< 累计计数 */
static int16_t s_last  = 0;   /*!< 上次读到的原始计数（16 位） */
static int32_t s_delta = 0;   /*!< 本周期脉冲增量 */
static int32_t s_rpm   = 0;   /*!< 输出轴转速 rpm */

/* ---- 自检 / 标定 ---- */
static uint8_t      s_cal_running   = 0u;   /*!< 自检进行中 */
static EncoderCal_t s_cal;                  /*!< 自检结果 */
static uint16_t     s_cal_revs      = 1u;   /*!< 自检声明的圈数 */
static uint16_t     s_cal_flips     = 0u;   /*!< 方向反转次数 */
static int8_t       s_cal_last_sign = 0;    /*!< 上次非零增量的方向 */

/* Exported functions --------------------------------------------------------*/

void Encoder_Init(void)
{
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  __HAL_TIM_SET_COUNTER(&htim2, 0);

  s_last  = 0;
  s_total = 0;
  s_delta = 0;
  s_rpm   = 0;
}

int32_t Encoder_Read(void)
{
  return s_total;
}

void Encoder_Reset(void)
{
  /* 必须关中断：若在"计数器清零"和"s_last 清零"之间被 TIM3 的 10 ms 中断
     打断，Encoder_Update 会用旧的 s_last 减出一次假的大增量 */
  uint32_t primask = __get_PRIMASK();

  __disable_irq();

  __HAL_TIM_SET_COUNTER(&htim2, 0);

  s_last  = 0;
  s_total = 0;
  s_delta = 0;
  s_rpm   = 0;

  if (primask == 0u)
  {
    __enable_irq();
  }
}

void Encoder_Update(void)
{
  int16_t now   = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
  int16_t delta = (int16_t)(now - s_last);   /* 16 位自然回绕，直接相减即可 */

  s_last   = now;
  s_delta  = delta;
  s_total += delta;

  /* M 法测速：10 ms 采一次
     rpm = delta / COUNTS_PER_OUTPUT_REV * (1000 / ENCODER_SAMPLE_MS) * 60
         = delta * 6000 / COUNTS_PER_OUTPUT_REV                                */
  s_rpm = (int32_t)(((int32_t)delta * 6000) / (int32_t)COUNTS_PER_OUTPUT_REV);

  /* 自检期间统计方向反转次数：匀速手转时应该接近 0，被 EMI 干扰会很大 */
  if (s_cal_running != 0u)
  {
    int8_t sign = (delta > 0) ? 1 : ((delta < 0) ? -1 : 0);

    if (sign != 0)
    {
      if ((s_cal_last_sign != 0) && (sign != s_cal_last_sign))
      {
        s_cal_flips++;
      }
      s_cal_last_sign = sign;
    }
  }
}

int32_t Encoder_GetRpm(void)
{
  return s_rpm;
}

int32_t Encoder_GetDelta(void)
{
  return s_delta;
}

/* ------------------------------ 自检 / 标定 ------------------------------ */

void Encoder_CalStart(uint16_t revs)
{
  if (revs == 0u)
  {
    revs = 1u;
  }

  Encoder_Reset();          /* 计数、转速全部清零，标定从 0 开始累计 */

  s_cal_revs      = revs;
  s_cal_flips     = 0u;
  s_cal_last_sign = 0;
  s_cal_running   = 1u;
}

void Encoder_CalStop(void)
{
  int32_t total;

  s_cal_running = 0u;

  total = s_total;          /* 标定期间从 0 累计，这里直接就是总脉冲 */

  s_cal.total   = total;
  s_cal.revs    = s_cal_revs;
  s_cal.flips   = s_cal_flips;
  s_cal.per_rev = ((total >= 0) ? total : -total) / (int32_t)s_cal_revs;
}

uint8_t Encoder_CalIsRunning(void)
{
  return s_cal_running;
}

const EncoderCal_t *Encoder_CalGet(void)
{
  return &s_cal;
}

uint8_t Encoder_JudgeMultiplier(int32_t per_rev, int32_t *err_permille)
{
  int32_t theory4 = (int32_t)COUNTS_PER_OUTPUT_REV;
  int32_t theory2 = theory4 / 2;
  int32_t err4    = ((per_rev - theory4) * 1000) / theory4;
  int32_t err2    = ((per_rev - theory2) * 1000) / theory2;
  int32_t abs4    = (err4 >= 0) ? err4 : -err4;
  int32_t abs2    = (err2 >= 0) ? err2 : -err2;

  if (per_rev < (int32_t)ENCODER_CAL_MIN_PER_REV)
  {
    if (err_permille != NULL)
    {
      *err_permille = 0;
    }
    return 0u;
  }

  if (abs4 <= abs2)
  {
    if (err_permille != NULL)
    {
      *err_permille = err4;
    }
    return 4u;
  }

  if (err_permille != NULL)
  {
    *err_permille = err2;
  }
  return 2u;
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
