/**
  ******************************************************************************
  * @file    stm32_.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "stm32_can.h"    
#include "stm32_Led.h"
    
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define CAN_MESSAGE_TIMER    1000
#define ReMapp_CAN
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

uint16_t CAN_timer = CAN_MESSAGE_TIMER;
uint32_t flag=0;   

uint32_t flag_timer=0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/
void CAN_Config_Init()
{
    NVIC_InitTypeDef NVIC_InitStructure;
    CAN_InitTypeDef CAN_InitStructure;
    CAN_FilterInitTypeDef CAN_FilterInitStructure;
 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);
     /* CANx Periph clock enable */
     RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
     
    /* configure the NVIC */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    
    NVIC_InitStructure. NVIC_IRQChannelPreemptionPriority =14; 
    NVIC_InitStructure. NVIC_IRQChannelSubPriority = 0x0;
    NVIC_InitStructure. NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure );

    /* CAN GPIO config */
    GPIO_InitTypeDef GPIO_InitStructure;
    
   
#ifdef ReMapp_CAN

  /* Configure CAN pin: RX */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB,&GPIO_InitStructure);
    
    /* Configure CAN PIN: TX*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB,&GPIO_InitStructure);
    
      /* Remap CAN1  */
  GPIO_PinRemapConfig(GPIO_Remap1_CAN1 , ENABLE);
    
#else
     /* Configure CAN pin: RX */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA,&GPIO_InitStructure);
    
    /* Configure CAN PIN: TX*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA,&GPIO_InitStructure);
    
#endif  
  
  /* CAN register init*/
  CAN_DeInit(CAN1);
  CAN_StructInit(&CAN_InitStructure);
  
  /* CAN cell init */
 CAN_InitStructure.CAN_TTCM = DISABLE;
 CAN_InitStructure.CAN_ABOM = DISABLE;
 CAN_InitStructure.CAN_AWUM = DISABLE;
 CAN_InitStructure.CAN_NART = ENABLE;
 CAN_InitStructure.CAN_RFLM = DISABLE;
 CAN_InitStructure.CAN_TXFP = DISABLE;
 CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;

   /* CAN Baudrate = 500KBps*/
  /* CANbps = FPCLK /(BRP +1)*((1+Tseg1)+(1+Tseg2)+1)*/
  /* CANbps = APB1 frequency 36000000/(1+2+3)/12=500k bps */
  /*  Tesg1>=Tesg2  Tseg2>=tq Tesg2 >= 2TSJW */
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
  CAN_InitStructure.CAN_BS1 = CAN_BS1_9tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;
  CAN_InitStructure.CAN_Prescaler = 4;
  CAN_Init(CAN1, &CAN_InitStructure);
 
      /* CAN filter init */

  CAN_FilterInitStructure.CAN_FilterNumber = 0;

  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);
  
  
  CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.
  /* Transmit */
  Tx_Message_123.StdId = 0x323;
  Tx_Message_123.ExtId = 0x00;
  Tx_Message_123.RTR = CAN_RTR_DATA;
  Tx_Message_123.IDE = CAN_ID_STD;
  Tx_Message_123.DLC = 8;
  Tx_Message_123.Data[0]=0;
  Tx_Message_123.Data[1]=0;
  Tx_Message_123.Data[2]=0;
  Tx_Message_123.Data[3]=0;
  Tx_Message_123.Data[4]=0;
  Tx_Message_123.Data[5]=0;
  Tx_Message_123.Data[6]=0;
  Tx_Message_123.Data[7]=0;
  
  
}



void CAN_task(void)
{
 
  if(CAN_timer==0)
  {  
    CAN_timer = CAN_MESSAGE_TIMER;
    
      flag_timer++;
     CAN_Transmit(CAN1,&Tx_Message_123);
  }

}

void USB_LP_CAN1_RX0_IRQHandler(void)
{
        CAN_Receive(CAN1, 0, &RxMessage);
        flag++;
        if(flag%2==0)
        {
          LED1_ON();
        }
        else
        {
          LED1_OFF();
        }
        
}


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
