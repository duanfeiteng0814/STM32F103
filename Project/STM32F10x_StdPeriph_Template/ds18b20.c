/********************   (C) COPYRIGHT 2015      ***************************
 * �ļ���  ��DS18B20.c
 * ����    ��DS18B20�¶ȴ��������Գ���
 * ʵ��ƽ̨��STM32F103RBT6
 * Ӳ�����ӣ�------------------------
 *          |   PB6 - DS18B20 DQ		|
 *           ------------------------ 
 * ��汾  ��ST3.5.0
 * ��дʱ�䣺2015-03-17
 * �޸�ʱ�䣺
 * ����    ��
****************************************************************************/
#include "ds18b20.h"





#define DS18B20_TIMER    1000U
#define ReMapp_CAN


uint16_t ds18b20_timer = DS18B20_TIMER;

extern CanTxMsg Tx_Message_123;

uint8_t temp1,temp2;   /* �¶ȴ�������ֵ������������С������ */
  unsigned int temp;     //�����¶Ȼ���Ĵ���
Temperature_U  temperature;
/*
 * ��������DS18B20_GPIO_Config
 * ����  ������Relay�õ���I/O��
 * ����  ����
 * ���  ����
 */
void DS18B20_GPIO_Config(void)
{		
	GPIO_InitTypeDef GPIO_InitStructure;	

	/*����DS18B20��Ӧ��GPIO ������ʱ��*/
	RCC_APB2PeriphClockCmd(RCC_DS18B20, ENABLE); 	
	/*ѡ��Ҫ���Ƶ�DS18B20����*/															   
  GPIO_InitStructure.GPIO_Pin = GPIO_DS18B20_Pin;	
	/*��������ģʽΪ OD���*/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   
	/*������������Ϊ50MHz */   
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	/*���ÿ⺯������ʼ����ӦGPIO*/
  GPIO_Init(GPIO_DS18B20_PORT, &GPIO_InitStructure);	
}


/*
 * ��������Delay_us
 * ����	 ��us��ʱ����1usΪһ����λ
 * ����  : - nTime
 * ���  ����
 * ʾ��  : Delay_us(1) ʵ�ֵ���ʱΪ��1*1us=1us
 * ����  ���ⲿ��ȷ��ʱ����
 */
void Delay_us(uint16_t nTime)
{

     int i=0;
      for(i=0;i<nTime*10;i++);
     
}


/*
 * ��������Delay_ms
 * ����	 ��ms��ʱ����1msΪһ����λ
 * ����  : - nTime
 * ���  ����
 * ʾ��  : Delay_ms(1) ʵ�ֵ���ʱΪ��1*1ms=1ms
 * ����  ������ʱ����
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
	Delay_us(20); 		/*  10us��ʱ  */
	DS18B20_L;
	Delay_us(550);    /*  550us��ʱ  */
	DS18B20_H;
	while(GPIO_ReadInputDataBit(GPIO_DS18B20_PORT,GPIO_DS18B20_Pin));
	Delay_us(500);		/*  500us��ʱ  */
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
	Delay_us(1);		/*  1us��ʱ  */
	DS18B20_L;
	Delay_us(55);   /*  55us��ʱ  */
	DS18B20_H;
	Delay_us(1);    /*  1us��ʱ  */
}

void DS18B20_WriteBit1(void)
{
	DS18B20_H;
	Delay_us(1);    /*  1us��ʱ  */
	DS18B20_L;
	Delay_us(5);   	/*  5us��ʱ  */
	DS18B20_H;
	Delay_us(5);   	/*  5us��ʱ  */
	Delay_us(50);   /*  50us��ʱ  */
}

uint8_t DS18B20_ReadBit(void)
{
	uint8_t bdata;
	DS18B20_H;
	Delay_us(1);   	/*  1us��ʱ  */
	DS18B20_L;
	Delay_us(4);   	/*  4us��ʱ  */
	DS18B20_H;
	Delay_us(8);   	/*  8us��ʱ  */	
	bdata=GPIO_ReadInputDataBit(GPIO_DS18B20_PORT,GPIO_DS18B20_Pin);
	Delay_us(60);   /*  60us��ʱ  */
	Delay_us(2);    /*  2us��ʱ  */
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
	Delay_us(10);  /*  10us��ʱ  */
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
		Delay_us(2);	/*  2us��ʱ  */
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
