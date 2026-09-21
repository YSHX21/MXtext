/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   CAN1 初始化与收发（驱动板 B ↔ 主控板 A）
  *
  * 引脚：PB8 -> CAN_RX   PB9 -> CAN_TX
  *       ❗这是 CAN1 的重映射引脚，必须调用 __HAL_AFIO_REMAP_CAN1_2()
  *         （默认引脚是 PA11/PA12，见 can.c 的 HAL_CAN_MspInit）
  * 波特率：125 kbps（位时序必须与 A 板完全一致，见 can.c 的注释）
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "can_proto.h"
/* USER CODE END Includes */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/*==================================================================
 * 单板自测开关（与 A 板 app.h 的 CAN_LOOPBACK_TEST 含义一致）
 *   1 = 回环模式：总线上没有第二个节点时用。CAN 控制器自发自收，
 *       不会因为收不到 ACK 而一直重发失败。调试 B 板时打开。
 *   0 = 正常模式：A/B 两块板都接上，且总线两端各接一个 120Ω 终端电阻。
 *==================================================================*/
#ifndef CAN_LOOPBACK_TEST
#define CAN_LOOPBACK_TEST   0
#endif

/** @brief 位时序：PCLK1=36MHz / 48 = 750kHz，1TQ=1.333us，(1+2+3)TQ=8us → 125kbps */
#define CAN_PRESCALER      48U

/* USER CODE END EC */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan;

/* Exported functions prototypes ---------------------------------------------*/
void MX_CAN_Init(void);

/* USER CODE BEGIN Prototypes */

/**
  * @brief  配置接收滤波器、启动 CAN、使能中断（按 CAN_LOOPBACK_TEST 决定模式）
  * @note   必须在 MX_CAN_Init() 之后调用
  */
void    Can_Init(void);

/**
  * @brief  发送一帧（非阻塞式：邮箱满/无 ACK 时最多等 CAN_TX_TIMEOUT_MS 就放弃）
  * @retval 1=已发出  0=失败（调用方可据此重发，绝不在这里死等）
  */
uint8_t Can_Send(uint8_t type, uint8_t src, uint8_t dst, const uint8_t *data, uint8_t len);

/**
  * @brief  上报运行状态  MSG_MOTION_STA (0x121)
  * @param  floor 当前楼层  state 见 MOTOR_STA_xxx  dir 见 MOTOR_DIR_xxx
  */
uint8_t Can_SendMotionSta(uint8_t floor, uint8_t state, uint8_t dir);

/** @brief 上报故障码  MSG_EMERGENCY (0x021) */
uint8_t Can_SendEmergency(uint8_t code);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */
