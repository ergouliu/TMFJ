/**
  ******************************************************************************
  * @file    CAN/LoopBack/stm32f2xx_it.c 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    13-April-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx_it.h"
#include "main.h"

int debug_x = 0;
/** @addtogroup STM32F2xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup CAN_LoopBack
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern __IO uint32_t ret;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
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
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
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
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	  TimingDelay_Decrement();
}

/******************************************************************************/
/*                 STM32F2xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f2xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/


void TIM2_IRQHandler(void)
{
	/*
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	TIM_SetCounter(TIM2,0x0000);
 
	if(debug_x == 0)
	{
  GPIO_ResetBits (GPIOC, GPIO_Pin_1); 
		debug_x = 1;
	}
  else
	{
		GPIO_SetBits (GPIOC, GPIO_Pin_1); 
		debug_x =0;
	}
	*/
	
   TIM_ClearFlag(TIM2, TIM_FLAG_Update);
   TIM_Cmd(TIM2, DISABLE);
   T15EndHandle();
	
}

/*******************************************************************************
* Function Name  : TIM3_IRQHandler
* Description    : This function handles TIM3 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void TIM3_IRQHandler(void)
{
  
  TIM_ClearFlag(TIM3, TIM_FLAG_Update);
  TIM_Cmd(TIM3, DISABLE);
  T35EndHandle();
}


/*******************************************************************************
* Function Name  : USART3_IRQHandler
* Description    : This function handles USART3 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART3_IRQHandler(void)
{

   u8 i;
  //接收中断

    if(USART_GetITStatus(USART3,USART_IT_RXNE)==SET)
    {

        USART_ClearITPendingBit(USART3,USART_IT_RXNE);
		i=USART_ReceiveData(USART3);
		ReceOneChar(i);	
		
    }
    
    if(USART_GetITStatus(USART3,USART_IT_TXE)==SET){
        USART_ClearITPendingBit(USART3,USART_IT_TXE);
    }
    
    //溢出-如果发生溢出需要先读SR,再读DR寄存器 则可清除不断入中断的问题
    if(USART_GetFlagStatus(USART3,USART_FLAG_ORE)==SET)
    {
        USART_ClearFlag(USART3,USART_FLAG_ORE);    //读SR
        USART_ReceiveData(USART3);                //读DR
    }
  
}

#ifdef USE_CAN1
/**
  * @brief  This function handles CAN1 RX0 request.
  * @param  None
  * @retval None
  */
void CAN1_RX0_IRQHandler(void)
{
  CanRxMsg RxMessage;

  RxMessage.StdId = 0x00;
  RxMessage.ExtId = 0x00;
  RxMessage.IDE = 0;
  RxMessage.DLC = 0;
  RxMessage.FMI = 0;
  RxMessage.Data[0] = 0x00;
  RxMessage.Data[1] = 0x00;

  CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);

  if((RxMessage.ExtId == 0x1234) && (RxMessage.IDE == CAN_ID_EXT)
     && (RxMessage.DLC == 2) && ((RxMessage.Data[1]|RxMessage.Data[0]<<8) == 0xDECA))
  {
    ret = 1; 
  }
  else
  {
    ret = 0; 
  }
}
#endif  /* USE_CAN1 */

#ifdef USE_CAN2
/**
  * @brief  This function handles CAN2 RX0 request.
  * @param  None
  * @retval None
  */
void CAN2_RX0_IRQHandler(void)
{
  CanRxMsg RxMessage;

  RxMessage.StdId = 0x00;
  RxMessage.ExtId = 0x00;
  RxMessage.IDE = 0;
  RxMessage.DLC = 0;
  RxMessage.FMI = 0;
  RxMessage.Data[0] = 0x00;
  RxMessage.Data[1] = 0x00;

  CAN_Receive(CAN2, CAN_FIFO0, &RxMessage);

  if((RxMessage.ExtId == 0x1234) && (RxMessage.IDE == CAN_ID_EXT)
     && (RxMessage.DLC == 2) && ((RxMessage.Data[1]|RxMessage.Data[0]<<8) == 0xDECA))
  {
    ret = 1; 
  }
  else
  {
    ret = 0; 
  }
}
#endif  /* USE_CAN2 */

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
