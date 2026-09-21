/*
 * @Author: yao 3547780037@qq.com
 * @Date: 2026-08-21 14:08:08
 * @LastEditors: yao 3547780037@qq.com
 * @LastEditTime: 2026-08-21 16:15:17
 * @FilePath: \MDK-ARMd:\CudeMxTest\ququqntest\Int\button4_4.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __BUTTON4_4_H
#define	__BUTTON4_4_H

/*****************���絥Ƭ�����******************
											STM32
 * �ļ�			:	4��4�������h�ļ�                   
 * �汾			: V1.0
 * ����			: 2024.9.5
 * MCU			:	STM32F103C8T6
 * �ӿ�			:	������										
 * IP�˺�		:	���絥Ƭ����ƣ�ͬBILIBILI|����|����|С����|CSDN|���ں�|��Ƶ�ŵȣ�
 * ����			:	���� 
 * ������		: �췽�����ӹ�����
 * ������Ƶ	:	https://www.bilibili.com/video/BV1fbpgeVEDJ/?share_source=copy_web
 * �ٷ���վ	:	www.yfcdz.cn

**********************BEGIN***********************/

/***************�����Լ��������****************/
// 4��4������� GPIO�궨��

#define 	BUTTON_ROW1_GPIO_PORT								GPIOB
#define 	BUTTON_ROW1_GPIO_PIN							    GPIO_PIN_12	
#define 	BUTTON_ROW2_GPIO_PORT								GPIOB
#define 	BUTTON_ROW2_GPIO_PIN								GPIO_PIN_13
#define 	BUTTON_ROW3_GPIO_PORT								GPIOB
#define 	BUTTON_ROW3_GPIO_PIN								GPIO_PIN_14
#define 	BUTTON_ROW4_GPIO_PORT								GPIOB
#define 	BUTTON_ROW4_GPIO_PIN								GPIO_PIN_15

#define 	BUTTON_COL1_GPIO_PORT								GPIOA
#define 	BUTTON_COL1_GPIO_PIN								GPIO_PIN_15
#define 	BUTTON_COL2_GPIO_PORT								GPIOA
#define 	BUTTON_COL2_GPIO_PIN								GPIO_PIN_10
#define 	BUTTON_COL3_GPIO_PORT								GPIOA
#define 	BUTTON_COL3_GPIO_PIN								GPIO_PIN_9
#define 	BUTTON_COL4_GPIO_PORT								GPIOA
#define 	BUTTON_COL4_GPIO_PIN								GPIO_PIN_8

	
/*********************END**********************/

void Button4_4_Init(void);
uint8_t Button4_4_Scan(void);
void button_OLED_start(void);

#endif

