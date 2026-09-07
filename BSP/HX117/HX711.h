#ifndef __HX711_H
#define __HX711_H

#include "stm32f1xx_hal.h"

/*================ HX711 引脚定义(已在 CubeMX 的 MX_GPIO_Init 中初始化) ============*/
/* sck_Pin=PA4 输出、DT_Pin=PA5 输入上拉 */
#define HX711_SCK_PORT    GPIOA
#define HX711_SCK_PIN     GPIO_PIN_4
#define HX711_DT_PORT     GPIOA
#define HX711_DT_PIN      GPIO_PIN_5

#define HX711_SCK_H()     HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_SET)
#define HX711_SCK_L()     HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_RESET)
#define HX711_DT_READ()   HAL_GPIO_ReadPin(HX711_DT_PORT, HX711_DT_PIN)

/* @brief 上电唤醒/初始化：SCK 保持低电平，让 HX711 退出掉电模式
 *        注意：CubeMX 里 sck_Pin 默认输出高电平，SCK持续高>60us会让芯片掉电，
 *              所以必须调用一次本函数拉低SCK，否则永远读不到数据 */
void HX711_Init(void);

/* @brief 检测一次转换是否完成：DOUT 被拉低表示数据就绪 */
uint8_t HX711_IsReady(void);

/* @brief 读取一次24位原始码(通道A、增益128)。
 *        阻塞等待DOUT就绪；调用前可先用HX711_IsReady确认。
 *        返回24位无符号原始值(0~0xFFFFFF，压力增大时读数增大需看接线) */
unsigned long HX711_GetData(void);

#endif /* __HX711_H */
