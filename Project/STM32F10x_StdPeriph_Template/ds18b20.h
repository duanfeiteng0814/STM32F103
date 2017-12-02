/********************       (C) COPYRIGHT 2015  ***************************
 * �ļ���  ��ds18b20.h
 * ����    ��ds18b20 Ӧ�ú�����         
 * ʵ��ƽ̨��
 * Ӳ�����ӣ�-----------------------
 *          |   PB6 - DS18B20 DQ    |   
 *           ----------------------- 
 * ��汾  ��ST3.5.0
 * ��д���ڣ�2015-03-17
 * �޸����ڣ�
 * ����    ��
****************************************************************************/
#ifndef __DS18B20_H
#define	__DS18B20_H

#include "stm32f10x.h"


#define RCC_DS18B20							RCC_APB2Periph_GPIOB
#define GPIO_DS18B20_PORT				GPIOB
#define GPIO_DS18B20_Pin				GPIO_Pin_6


#define DS18B20_H  		GPIO_SetBits(GPIOB,GPIO_Pin_6)
#define DS18B20_L  		GPIO_ResetBits(GPIOB,GPIO_Pin_6)

typedef union Temperature_U
{
       uint8_t Temperature[4];
      float Temperature_Value;
 

}Temperature_U;





void DS18B20_GPIO_Config(void);
void DS18B20_Reset(void);
void DS18B20_Init(void);
void DS18B20_WriteBit1(void);
void DS18B20_WriteBit0(void);
uint8_t DS18B20_ReadBit(void);
void DS18B20_WriteByte(uint8_t udata);
uint8_t DS18B20_ReadByte(void);
void DS18B20_GetTemp(void);





#endif /* __LED_H */
