/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32_.h 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   This file contains the headers of the interrupt handlers.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32_LED_H
#define __STM32_LED_H


   

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_can.h"

#ifdef __cplusplus
 extern "C" {
#endif 
   
/* Exported types ------------------------------------------------------------*/
CanRxMsg RxMessage;
CanTxMsg Tx_Message_123;
CanTxMsg Tx_Message_456;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void CAN_Config_Init();
void CAN_task(void);


#ifdef __cplusplus
}
#endif

#endif /* __STM32_LED_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
