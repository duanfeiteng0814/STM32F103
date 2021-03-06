/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
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
#include "stm32f10x.h"
#include <stdio.h>
#include "stm32_Led.h"
#include "stm32_can.h" 
#include "stm32_pwm.h"
#include  "ds18b20.h"
#include  "LCD.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern Temperature_U  temperature;
/* Private function prototypes -----------------------------------------------*/
char Step_count[4];
/* Private functions ---------------------------------------------------------*/
void Init_SysTick(void);
void CAN_task(void);
void CAN_Config_Init();
/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
char strTemp[4];

int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */     

  /* Add your application code here
     */
    Init_SysTick();
    DS18B20_GPIO_Config();
    CAN_Config_Init();
    Led_GPIO_Init();
    PWM_Init();
    LCD_Init();                      //oled ��ʼ��  
    LCD_Fill(0xff);                  //��ȫ�� 
    LCD_CLS();                      //ȫ��Ļ��

 
  /* Infinite loop */
  while (1)
  {
      CAN_task();
      DS18B20_task();
      PWM_task();
         sprintf(strTemp, "%.01f", temperature.Temperature_Value); 
       LCD_P8x16Str(20,2,"Temperature"); 
       LCD_P8x16Str(40,4,strTemp); 

   
  }
}

 void Init_SysTick(void)
{
  
    if(SysTick_Config(SystemCoreClock / 1000))  //1ms tick
    while(1);

}






/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
