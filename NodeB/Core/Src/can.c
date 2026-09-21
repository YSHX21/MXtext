/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   驱动板 B 的 CAN1 初始化与收发
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
#include "can.h"

/* USER CODE BEGIN 0 */
#include "traction.h"

/** @brief 单帧最大等待时间(ms)：收不到 ACK 就放弃，绝不死等 */
#define CAN_TX_TIMEOUT_MS   5U

/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  /* 与 A 板(MXtext/Core/Src/can.c)完全一致的位时序：
     PCLK1=36MHz /48 = 750kHz；1TQ=1.333us；(1+2+3)TQ=8us → 125kbps
     ❗两块板只要有一个参数不同就直接通不了 */
  hcan.Init.Prescaler = CAN_PRESCALER;
  hcan.Init.Mode = CAN_MODE_NORMAL;     /*注：Can_Init() 里按 CAN_LOOPBACK_TEST 可能改成回环 */
  hcan.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;   /*按 ID 优先级发，与 A 板一致*/
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN CAN1_MspInit 1 */
    /* ❗PB8/PB9 是 CAN1 的"重映射"引脚(不是默认引脚)：
       STM32F1 的 CAN1 默认落在 PA11/PA12，必须显式打开 AFIO 重映射，
       否则引脚配好了也没用 —— CAN 控制器仍然挂在 PA11/PA12 上，
       一根帧都收发不出去，而且不会有任何报错，极难查。
       REMAP2 = CAN_RX→PB8 / CAN_TX→PB9（见 stm32f103xb.h 的
       AFIO_MAPR_CAN_REMAP_REMAP2 注释）。
       AFIO 时钟已在 HAL_MspInit() 里使能。 */
    __HAL_AFIO_REMAP_CAN1_2();
  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void Can_Init(void)
{
  CAN_FilterTypeDef filter;

#if CAN_LOOPBACK_TEST
  /* 单板自测：切回环模式(自发自收)。
     正常模式下如果总线上没有第二个节点给 ACK 应答，报文会一直重发失败。 */
  HAL_CAN_Stop(&hcan);
  hcan.Init.Mode = CAN_MODE_LOOPBACK;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
#endif

  /* 滤波器全放行(与 A 板同一套配置)，目标节点在软件里判。
     本板只认 dst = NODE_DRIVER 或广播(0)。 */
  filter.FilterActivation     = ENABLE;
  filter.FilterMode           = CAN_FILTERMODE_IDMASK;
  filter.FilterScale          = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh         = 0x0000;
  filter.FilterIdLow          = 0x0000;
  filter.FilterMaskIdHigh     = 0x0000;
  filter.FilterMaskIdLow      = 0x0000;
  filter.FilterBank           = 0;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    Error_Handler();
  }

  /* 使能接收中断，否则 FIFO 里来报文了也没有任何回调 */
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }

  /* ❗CubeMX 没有为 CAN 生成 NVIC 配置(.ioc 里刻意没勾，避免重新生成时
     和 stm32f1xx_it.c 里的中断处理函数重复定义)，这里手动开。
     A 板也是在 CAN_Init() 里手开的，保持一致。 */
  HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
}

uint8_t Can_Send(uint8_t type, uint8_t src, uint8_t dst, const uint8_t *data, uint8_t len)
{
  CAN_TxHeaderTypeDef txHeader;
  uint32_t            mailbox;
  uint32_t            t0;

  if (data == NULL)
  {
    return 0U;
  }
  if (len > 8U)
  {
    len = 8U;
  }

  txHeader.StdId = CAN_MAKE_ID(type, src, dst);
  txHeader.IDE   = CAN_ID_STD;
  txHeader.RTR   = CAN_RTR_DATA;
  txHeader.DLC   = len;
  txHeader.TransmitGlobalTime = DISABLE;

  /* 放入发送邮箱。失败(三个邮箱都满/总线关闭)直接返回，不在这里死等 */
  if (HAL_CAN_AddTxMessage(&hcan, &txHeader, (uint8_t *)data, &mailbox) != HAL_OK)
  {
    return 0U;
  }

  /* 等这一帧真正发出去。总线上没有第二个节点给 ACK 时这里必然超时，
     所以单板调试要把 CAN_LOOPBACK_TEST 打开，否则每帧都白等 5ms。 */
  t0 = HAL_GetTick();
  while (HAL_CAN_IsTxMessagePending(&hcan, mailbox) != 0U)
  {
    if ((HAL_GetTick() - t0) >= CAN_TX_TIMEOUT_MS)
    {
      return 0U;
    }
  }

  return 1U;
}

uint8_t Can_SendMotionSta(uint8_t floor, uint8_t state, uint8_t dir)
{
  /* [0]=当前楼层 [1]=运行状态 [2]=方向 */
  uint8_t tx[3];

  tx[0] = floor;
  tx[1] = state;
  tx[2] = dir;

  return Can_Send(MSG_MOTION_STA, NODE_DRIVER, NODE_MAIN, tx, 3U);
}

uint8_t Can_SendEmergency(uint8_t code)
{
  /* [0]=故障码 */
  uint8_t tx[1];

  tx[0] = code;

  return Can_Send(MSG_EMERGENCY, NODE_DRIVER, NODE_MAIN, tx, 1U);
}

/**
  * @brief  接收中断回调：只做"取出报文 + 交给曳引状态机"，中断里不发报文
  * @note   真正的 CAN 发送全部放在主循环(Traction_Poll)，避免在中断里
  *         做会产生等待的操作
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_ptr)
{
  CAN_RxHeaderTypeDef rxHeader;
  CanFrame_t          frame;
  uint8_t             dst;

  if (hcan_ptr->Instance != CAN1)
  {
    return;
  }

  if (HAL_CAN_GetRxMessage(hcan_ptr, CAN_RX_FIFO0, &rxHeader, frame.data) != HAL_OK)
  {
    return;
  }

  /* 只处理标准数据帧 */
  if (rxHeader.IDE != CAN_ID_STD)
  {
    return;
  }
  if (rxHeader.RTR != CAN_RTR_DATA)
  {
    return;
  }

  frame.id  = rxHeader.StdId;
  frame.len = (uint8_t)rxHeader.DLC;

  /* A→B 才处理：发给本节点或广播。
     这条判断也顺便把回环模式下自己发出去的帧(MOTION_STA/EMERGENCY
     的 dst 是 NODE_MAIN)挡掉了，不会自己吃自己的状态帧。 */
  dst = CAN_GET_DST(frame.id);
  if ((dst != (uint8_t)NODE_DRIVER) && (dst != 0U))
  {
    return;
  }

  Traction_OnCanFrame(&frame);
}

/* USER CODE END 1 */
