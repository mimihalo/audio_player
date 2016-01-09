/**
  ******************************************************************************
  * @file    Audio_playback_and_record/src/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    28-October-2011
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
#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4_discovery_audio_codec.h"
#include "stm32f4_discovery_lis302dl.h"
#include "ff.h"
#include <stdio.h>

/** @addtogroup STM32F4-Discovery_Audio_Player_Recorder
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint8_t PauseResumeStatus = 2, Count = 0, LED_Toggle = 0;
uint16_t capture = 0;
extern __IO uint16_t CCR_Val;
extern __IO uint8_t RepeatState, AudioPlayStart;
extern uint8_t Buffer[];

__IO uint16_t Time_Rec_Base = 0;
extern FIL file;
extern __IO uint8_t Data_Status;
extern __IO uint32_t XferCplt ;
extern __IO uint8_t Command_index;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
/*void SysTick_Handler(void)
{
  TimingDelay_Decrement();
  if ( Command_index == 1)
  {
    Time_Rec_Base ++;
  }
}*/

void EXTI1_IRQHandler(void)
{
  /* Check the clic on the accelerometer to Pause/Resume Playing */
  if(EXTI_GetITStatus(EXTI_Line1) != RESET)
  {
    if( Count==1)
    {
      PauseResumeStatus = 1;
      Count = 0;
    }
    else
    {
      PauseResumeStatus = 0;
      Count = 1;
    }
    /* Clear the EXTI line 1 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line1);
  }
}

/**
  * @brief  This function handles TIM4 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM4_IRQHandler(void)
{
   uint8_t clickreg = 0;

  if (AudioPlayStart != 0x00)
  {
    /* Read click status register */
    LIS302DL_Read(&clickreg, LIS302DL_CLICK_SRC_REG_ADDR, 1); 
    LIS302DL_Read(Buffer, LIS302DL_STATUS_REG_ADDR, 6);
  }
  
  /* Checks whether the TIM interrupt has occurred */
  if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET)
  {
    TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);

	
    capture = TIM_GetCapture1(TIM4);
    TIM_SetCompare1(TIM4, capture + CCR_Val);
  }
}


/**
  * @brief  EXTI0_IRQHandler
  *         This function handles External line 0 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void)
{
  /* Checks whether the User Button EXTI line is asserted*/
  if (EXTI_GetITStatus(EXTI_Line0) != RESET) 
  { 
    if (Command_index == 1)
    {
      RepeatState = 0;
      /* Switch to play command */
      Command_index = 0;
    }
    else if (Command_index == 0)
    {
      /* Switch to record command */
      Command_index = 1;
      XferCplt = 1;
      EVAL_AUDIO_Stop(CODEC_PDWN_SW);
    }
    else
    {
      RepeatState = 0;
      /* Switch to play command */
      Command_index = 0; 
    }
  } 
  /* Clears the EXTI's line pending bit.*/ 
  EXTI_ClearITPendingBit(EXTI_Line0);
}


/**
  * @}
  */ 
  
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
