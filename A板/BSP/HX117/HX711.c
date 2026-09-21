#include "HX711.h"

/* @brief 退出掉电并等待第一次转换完成 */
void HX711_Init(void)
{
	/*SCK 保持低电平 >60us：若之前SCK一直为高，芯片处于掉电模式，拉低后自动上电复位*/
	HX711_SCK_L();
	HAL_Delay(2);

	/*上电复位后约需数百ms完成第一次转换(10Hz)，等DOUT变低表示数据就绪*/
	for(uint16_t i = 0; i < 1000U; i++)
	{
		if(HX711_IsReady())
			break;
		HAL_Delay(1);
	}
}

/* @brief 数据就绪判定 */
uint8_t HX711_IsReady(void)
{
	return (HX711_DT_READ() == GPIO_PIN_RESET) ? 1U : 0U;
}

/* @brief 读取一次24位原始数据(通道A、增益128)
 *  时序：等待DOUT拉低(转换完成)→24个SCK脉冲读出数据位(MSB在前)→第25脉冲设定增益
 *  返回24位无符号原始码(不翻转符号，便于按原始值做两点线性标定) */
unsigned long HX711_GetData(void)
{
	unsigned long Count = 0;
	uint8_t i;

	/*等待本次转换完成：DOUT 由高变低(调用前请用 HX711_IsReady 轮询，避免长时间忙等)*/
	while(HX711_DT_READ() != GPIO_PIN_RESET);

	for(i = 0; i < 24; i++)
	{
		HX711_SCK_H();
		Count <<= 1;
		HX711_SCK_L();
		if(HX711_DT_READ() != GPIO_PIN_RESET)
			Count |= 1UL;
	}

	/*第25个脉冲：选择通道A、增益128，结束本次读数*/
	HX711_SCK_H();
	HX711_SCK_L();

	return Count;
}
