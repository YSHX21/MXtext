#include "stm32f1xx_hal.h"
#include "xuanzhun.h"
TIM_HandleTypeDef htim3;
void xuanzhun_SetAngle(float Angle)
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,Angle/180*2000+500);
}
