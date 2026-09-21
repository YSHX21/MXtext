/*
 * @Author: yao 3547780037@qq.com
 * @Date: 2026-08-22 13:39:44
 * @LastEditors: yao 3547780037@qq.com
 * @LastEditTime: 2026-08-22 14:10:58
 * @FilePath: \MDK-ARMd:\CudeMxTest\MXtext\BSP\bottun\button4_4.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include "stm32f1xx_hal.h"
#include "button4_4.h"

struct IO_PORT
{                                            
        GPIO_TypeDef *GPIO_x;                 
        unsigned short GPIO_PIN;
};
static struct IO_PORT KEY_OUT[4] = {
        {BUTTON_ROW1_GPIO_PORT, BUTTON_ROW1_GPIO_PIN},
				{BUTTON_ROW2_GPIO_PORT, BUTTON_ROW2_GPIO_PIN},
        {BUTTON_ROW3_GPIO_PORT, BUTTON_ROW3_GPIO_PIN}, 
				{BUTTON_ROW4_GPIO_PORT, BUTTON_ROW4_GPIO_PIN}
};
static struct IO_PORT KEY_IN[4] = {
        {BUTTON_COL1_GPIO_PORT, BUTTON_COL1_GPIO_PIN}, 
				{BUTTON_COL2_GPIO_PORT, BUTTON_COL2_GPIO_PIN},
        {BUTTON_COL3_GPIO_PORT, BUTTON_COL3_GPIO_PIN}, 
				{BUTTON_COL4_GPIO_PORT, BUTTON_COL4_GPIO_PIN}
};
unsigned char key[4][4];

void Button4_4_Init(void)
{

    unsigned char i;
	for(i = 0; i < 4; i++)
	{
		HAL_GPIO_WritePin(KEY_OUT[i].GPIO_x,KEY_OUT[i].GPIO_PIN,GPIO_PIN_SET);
		HAL_Delay(5);
	}
}
uint8_t Button4_4_Scan(void)
{
        unsigned char i, j;
        for(i = 0; i < 4; i++)            
        {
			
          
			HAL_GPIO_WritePin(KEY_OUT[i].GPIO_x,KEY_OUT[i].GPIO_PIN,GPIO_PIN_RESET);
			HAL_Delay(5);
   
          for(j = 0; j < 4; j++)            
          {
			   
                  HAL_Delay(5);
			  
						      if(HAL_GPIO_ReadPin(KEY_IN[j].GPIO_x, KEY_IN[j].GPIO_PIN) == 0)
                   {
                                key[i][j] = 1;
                   }else{
                                key[i][j] = 0;
                   }
          }
          HAL_GPIO_WritePin(KEY_OUT[i].GPIO_x, KEY_OUT[i].GPIO_PIN,GPIO_PIN_SET);
        }
        if(key[0][0]==1)return 1;
        else if(key[0][1]==1)return 2;
        else if(key[0][2]==1)return 3;
        else if(key[0][3]==1)return 4;
        else if(key[1][0]==1)return 5;
        else if(key[1][1]==1)return 6;
        else if(key[1][2]==1)return 7;
        else if(key[1][3]==1)return 8;
        else if(key[2][0]==1)return 9;
        else if(key[2][1]==1)return 10;
        else if(key[2][2]==1)return 11;
        else if(key[2][3]==1)return 12;
        else if(key[3][0]==1)return 13;
        else if(key[3][1]==1)return 14;
        else if(key[3][2]==1)return 15;
        else if(key[3][3]==1)return 16;
				
				else return 0;
				
       
}

