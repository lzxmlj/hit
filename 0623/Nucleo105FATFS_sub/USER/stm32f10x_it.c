/**
  ******************************************************************************
  * @file GPIO/IOToggle/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version  V3.0.0
  * @date  04/06/2009
  * @brief  Main Interrupt Service Routines.
  *         This file provides template for all exceptions handler and 
  *         peripherals interrupt service routine.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_can.h"
#include "rtc.h"           


#include "usart.h"
u8 loop_enable;
extern u8 first_start, backup_type;
u8 VblistA_full;
u16 count_canmessage;
u8 complete_count, complete_count1;
u16 count_canmessage1, count_canmessage2;
u32 time_count, time_count_ms, time_count_us,time_count_sec;
u32 time_count1, test_count, Keep_count;
u8 time_err, Vbcould_update;
u8 test_buff[40970], CAN_need_update_data[8];
u8 needupdate, CAN_need_update = 1;
u32 down_edge_time, up_edge_time, down_edge_count, up_edge_count;
void hit_event(u8 i);
extern u32 DAQ_id[4];
u8 UpdateforASCII_H(u8 number);
u8 UpdateforASCII_L(u8 number);
/** @addtogroup StdPeriph_Examples
  * @{
  */

/** @addtogroup GPIO_IOToggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval : None
  */
void NMI_Handler(void)
{
}

/*
  USART2中断服务程序
*/


void EXTI2_IRQHandler(void)
{
   u32 i;
   if(EXTI_GetITStatus(EXTI_Line2) != RESET)
   {
      EXTI_ClearITPendingBit(EXTI_Line2);
      
      time_count1 = time_count1 + 1;
      hit_event(2);

   //sent_uart();
   }

}
void USART2_IRQHandler(void)
{
   u8 rece;

    /* 接收 */
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
         rece=USART_ReceiveData(USART2);      //接收数据
    }

    /* 发送 */
    if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)                    
    { 
        USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
    }
}

/*******************************************************************************
* Function Name  : USB_LP_CAN_RX0_IRQHandler
* Description    : This function handles USB Low Priority or CAN RX0 interrupts 
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

/***********************
 CAN2 FIFO1
***********************/

u8 UpdateforASCII_H(u8 number)
{
   if(((number>> 4) >= 10) && ((number >> 4) <= 15) )
      return  (number >> 4) + 55;
   else
      return (number >> 4) + 48;
}
u8 UpdateforASCII_L(u8 number)
{
   if(((number & 0xF) >= 10) && ((number & 0xF) <= 15) )
      return  (number & 0xF) + 55;
   else
      return (number & 0xF) + 48;
}

/***********************
 CAN2 FIFO    暂不使用
***********************/                                                     
                                                 



/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval : None
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
  * @retval : None
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
  * @retval : None
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
  * @retval : None
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
  * @retval : None
  */
void SVC_Handler(void)
{
}
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    CanRxMsg RxMessage;
    u8 i;
    RxMessage.StdId=0x00;
    RxMessage.ExtId=0x00;
   RxMessage.RTR=CAN_RTR_DATA;  // 数据帧or远程
    RxMessage.IDE=CAN_ID_STD;    // 标准or扩展
    RxMessage.DLC=0;
    RxMessage.FMI=0;
    RxMessage.Data[0]=0x00;
    RxMessage.Data[1]=0x00;

    CAN_Receive(CAN1,CAN_FIFO0, &RxMessage);    // CAN接收数据      
    CAN_need_update = RxMessage.StdId;
    if(CAN_need_update == 0xFF)
    {
       for(i =0; i < 8; i++)
       {
          CAN_need_update_data[i] = RxMessage.Data[i];
       }
    }
   //Comm_Send_CANmsg_str(1,&RxMessage);
}
/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval : None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval : None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval : None
  */
void SysTick_Handler(void)
{
}

void TIM2_IRQHandler(void)
{
   u16 i;
   if(TIM_GetITStatus(TIM2,TIM_IT_CC1) != RESET)
   {
     TIM_ClearITPendingBit(TIM2,TIM_IT_CC1);
     time_count = time_count + 1;
     loop_enable = 1;
     TIM_ClearFlag(TIM2, TIM_FLAG_Update);
   }
}
void TIM4_IRQHandler(void)
{
   u16 i;
   if(TIM_GetITStatus(TIM4,TIM_IT_CC1) != RESET)
   {
     TIM_ClearITPendingBit(TIM4,TIM_IT_CC1);
     time_count_ms = time_count_ms + 1;
     TIM_ClearFlag(TIM4, TIM_FLAG_Update);
   }
}
void TIM3_IRQHandler(void)
{
   u16 i;
   if(TIM_GetITStatus(TIM3,TIM_IT_CC1) != RESET)
   {
     TIM_ClearITPendingBit(TIM3,TIM_IT_CC1);
     time_count_ms = 0;
     TIM_ClearFlag(TIM3, TIM_FLAG_Update);
   }
}
void hit_event(u8 i)
{
  u32  j, temp_pin;
  if((needupdate ==0))
  {
     if(( test_count < 40960))
     {
           test_buff[test_count] = calendar.w_month;
           test_buff[test_count+1] = calendar.w_date;
           test_buff[test_count+2] = calendar.hour;
           test_buff[test_count+3] = calendar.min;
           test_buff[test_count+4] = calendar.sec;
           test_buff[test_count+5] = time_count_ms / 0x100;
           test_buff[test_count+6] = time_count_ms & 0xFF;
           test_buff[test_count+7] = TIM_GetCounter(TIM4) / 0x100;
           test_buff[test_count+8] = TIM_GetCounter(TIM4) & 0xFF;
           test_buff[test_count+9] = 1;
           test_count = test_count + 10;
      }
      else
      {
         test_buff[test_count] = calendar.w_month;
         test_buff[test_count+1] = calendar.w_date;
         test_buff[test_count+2] = calendar.hour;
         test_buff[test_count+3] = calendar.min;
         test_buff[test_count+4] = calendar.sec;
         test_buff[test_count+5] = time_count_ms / 0x100;
         test_buff[test_count+6] = time_count_ms & 0xFF;
         test_buff[test_count+7] = TIM_GetCounter(TIM4) / 0x100;
         test_buff[test_count+8] = TIM_GetCounter(TIM4) & 0xFF;
         test_buff[test_count+9] = 0xFF;
         test_count = test_count + 10;
         needupdate = 1;
      }
   }
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval : None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
