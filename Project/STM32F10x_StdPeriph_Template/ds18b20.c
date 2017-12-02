/********************   (C) COPYRIGHT 2015      ***************************
 * 文件名  ：DS18B20.c
 * 描述    ：DS18B20温度传感器测试程序
 * 实验平台：STM32F103RBT6
 * 硬件连接：------------------------
 *          |   PB6 - DS18B20 DQ		|
 *           ------------------------ 
 * 库版本  ：ST3.5.0
 * 编写时间：2015-03-17
 * 修改时间：
 * 作者    ：
****************************************************************************/
#include "ds18b20.h"





#define DS18B20_TIMER    1000U
#define ReMapp_CAN


uint16_t ds18b20_timer = DS18B20_TIMER;

extern CanTxMsg Tx_Message_123;

uint8_t temp1,temp2;   /* 温度传感器数值：整数部分与小数部分 */
  unsigned int temp;     //定义温度缓冲寄存器
Temperature_U  temperature;
/*
 * 函数名：DS18B20_GPIO_Config
 * 描述  ：配置Relay用到的I/O口
 * 输入  ：无
 * 输出  ：无
 */
void DS18B20_GPIO_Config(void)
{		
	GPIO_InitTypeDef GPIO_InitStructure;	

	/*开启DS18B20对应的GPIO 的外设时钟*/
	RCC_APB2PeriphClockCmd(RCC_DS18B20, ENABLE); 	
	/*选择要控制的DS18B20引脚*/															   
  GPIO_InitStructure.GPIO_Pin = GPIO_DS18B20_Pin;	
	/*设置引脚模式为 OD输出*/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   
	/*设置引脚速率为50MHz */   
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	/*调用库函数，初始化相应GPIO*/
  GPIO_Init(GPIO_DS18B20_PORT, &GPIO_InitStructure);	
}


/*
 * 函数名：Delay_us
 * 描述	 ：us延时程序，1us为一个单位
 * 输入  : - nTime
 * 输出  ：无
 * 示例  : Delay_us(1) 实现的延时为：1*1us=1us
 * 调用  ：外部精确延时调用
 */
void Delay_us(uint16_t nTime)
{

     int i=0;
      for(i=0;i<nTime*10;i++);
     
}


/*
 * 函数名：Delay_ms
 * 描述	 ：ms延时程序，1ms为一个单位
 * 输入  : - nTime
 * 输出  ：无
 * 示例  : Delay_ms(1) 实现的延时为：1*1ms=1ms
 * 调用  ：粗延时调用
 */
void Delay_ms(uint16_t nTime)
{
	uint16_t i;
	for(i=0;i<nTime;i++)
	Delay_us(1000);
}


void DS18B20_Reset(void)
{
	DS18B20_H;
	Delay_us(20); 		/*  10us延时  */
	DS18B20_L;
	Delay_us(550);    /*  550us延时  */
	DS18B20_H;
	while(GPIO_ReadInputDataBit(GPIO_DS18B20_PORT,GPIO_DS18B20_Pin));
	Delay_us(500);		/*  500us延时  */
	DS18B20_H;
}


void DS18B20_Init(void)
{
	DS18B20_Reset();
	DS18B20_WriteByte(0xCC);
	DS18B20_WriteByte(0x4E);
	DS18B20_WriteByte(0x64);
	DS18B20_WriteByte(0x8A);
	DS18B20_WriteByte(0x1F);
}



void DS18B20_WriteBit0(void)
{
	DS18B20_H;
	Delay_us(1);		/*  1us延时  */
	DS18B20_L;
	Delay_us(55);   /*  55us延时  */
	DS18B20_H;
	Delay_us(1);    /*  1us延时  */
}

void DS18B20_WriteBit1(void)
{
	DS18B20_H;
	Delay_us(1);    /*  1us延时  */
	DS18B20_L;
	Delay_us(5);   	/*  5us延时  */
	DS18B20_H;
	Delay_us(5);   	/*  5us延时  */
	Delay_us(50);   /*  50us延时  */
}

uint8_t DS18B20_ReadBit(void)
{
	uint8_t bdata;
	DS18B20_H;
	Delay_us(1);   	/*  1us延时  */
	DS18B20_L;
	Delay_us(4);   	/*  4us延时  */
	DS18B20_H;
	Delay_us(8);   	/*  8us延时  */	
	bdata=GPIO_ReadInputDataBit(GPIO_DS18B20_PORT,GPIO_DS18B20_Pin);
	Delay_us(60);   /*  60us延时  */
	Delay_us(2);    /*  2us延时  */
	return bdata;
}

void DS18B20_WriteByte(uint8_t udata)
{
	uint8_t i;
	for(i=0;i<8;i++)
	{
		if(udata&0x01)
			DS18B20_WriteBit1();
		else
			DS18B20_WriteBit0();
		udata=udata>>1;
	}
	Delay_us(10);  /*  10us延时  */
}

uint8_t DS18B20_ReadByte(void)
{
	uint8_t i,udata,j;
	udata=0;
	for(i=0;i<8;i++)
	{
		udata=udata>>1;
		j=DS18B20_ReadBit();
		if(j==0x01)
			udata|=0x80;
		else
			udata|=0x00;
		Delay_us(2);	/*  2us延时  */
	}
	return udata;
}



void DS18B20_GetTemp(void)
{

	DS18B20_WriteByte(0xCC);
	DS18B20_WriteByte(0x44);
	Delay_ms(100);
	DS18B20_Reset();
	DS18B20_WriteByte(0xCC);
	DS18B20_WriteByte(0xBE);
	
	temp2=DS18B20_ReadByte();
	temp1=DS18B20_ReadByte();

        
        temp=temp1*0xFF+temp2;
    
        temperature.Temperature_Value = temp*0.0625;
        
        
         Tx_Message_123.Data[0]=temperature.Temperature[0];
         Tx_Message_123.Data[1]=temperature.Temperature[1];
         Tx_Message_123.Data[2]=temperature.Temperature[2];
         Tx_Message_123.Data[3]=temperature.Temperature[3];
	DS18B20_Reset();
            	
}



void DS18B20_task()
{
  DS18B20_GetTemp();

}



/*******************   (C) COPYRIGHT 2015   *****END OF FILE************/
