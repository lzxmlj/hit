#include "stm32f10x_conf.h"
#include "stdint.h"
#include "stm32f10x_can.h"

uint16_t crc=0xffff;
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;
TestStatus status;
uint32_t ret = 0; /* for return of the interrupt handling */
uint32_t can_message_id1;
CanRxMsg RxMessage;
float temp_float;
u8 Rxmessage10[8], RxMessage1[8];
extern u32 baudrate;
CanTxMsg TxMessage,TxMessage1;
CanRxMsg RxMessage0;
extern void CAN_DeInit(CAN_TypeDef* CANx);

extern void CAN_ITConfig(CAN_TypeDef* CANx, uint32_t CAN_IT, FunctionalState NewState);
extern uint8_t CAN_Init(CAN_TypeDef* CANx, CAN_InitTypeDef* CAN_InitStruct);
extern uint8_t CAN_MessagePending(CAN_TypeDef* CANx, uint8_t FIFONumber);

extern void CAN_Receive(CAN_TypeDef* CANx, uint8_t FIFONumber, CanRxMsg* RxMessage);
extern void CAN_StructInit(CAN_InitTypeDef* CAN_InitStruct);
extern uint8_t CAN_Transmit(CAN_TypeDef* CANx, CanTxMsg* TxMessage);
extern uint8_t CAN_TransmitStatus(CAN_TypeDef* CANx, uint8_t TransmitMailbox);

/* Private function prototypes -----------------------------------------------*/
void CAN_Config(void);
void calcrc(uint8_t *crcbuf,uint8_t size,uint8_t count);
TestStatus Receive(CanRxMsg *RxMessage0);
TestStatus CAN_Polling(void);
TestStatus CAN_Interrupt(void);
#if 0
int main(void)
{
  TestStatus status;              
  /* CAN configuration */
  CAN_Config();
        
        status=CAN_Polling();
        if(status)
        {
                CAN_Transmit(CAN1,&TxMessage);
        }
  
        status=CAN_Interrupt();
        if(status)
        {
                CAN_Transmit(CAN1,&TxMessage);
        }
        
}
#endif
void CAN1_Config(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;
  CAN_InitTypeDef        CAN_InitStructure;
  CAN_FilterInitTypeDef  CAN_FilterInitStructure;
    
  /* CAN GPIOs configuration **************************************************/

  /* Enable GPIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

  /* Connect CAN pins to AF7 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  /* CAN1 TX PB9 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  //GPIO_PinRemapConfig(GPIO_Remap1_CAN1,ENABLE);  // CAN1 remap
  
   /* CAN1 Enabling interrupt */                             
   NVIC_InitStructure.NVIC_IRQChannel=USB_LP_CAN1_RX0_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);                         

  CAN_DeInit(CAN1);
   CAN_StructInit(&CAN_InitStructure);   
  
   CAN_InitStructure.CAN_TTCM=DISABLE;
   CAN_InitStructure.CAN_ABOM=ENABLE;
   CAN_InitStructure.CAN_AWUM=DISABLE;
   CAN_InitStructure.CAN_NART=DISABLE;
   CAN_InitStructure.CAN_RFLM=DISABLE;
   CAN_InitStructure.CAN_TXFP=DISABLE;
   CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;   
  //CAN_InitStructure.CAN_Mode=CAN_Mode_LoopBack;
   CAN_InitStructure.CAN_SJW=0;
   CAN_InitStructure.CAN_BS1=3;  
   CAN_InitStructure.CAN_BS2=3;  
   CAN_InitStructure.CAN_Prescaler=8;
   
  
   CAN_Init(CAN1,&CAN_InitStructure); // CAN1                                
  
   CAN_FilterInitStructure.CAN_FilterNumber=0;  
   CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;  // 标识符屏蔽位模式
   CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;   // 32位过滤器
   CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;        // 过滤器标识符
   CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;            
   CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;    // 过滤器屏蔽标识符
   CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;    // FIFO0指向过滤器
   CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
   CAN_FilterInit(&CAN_FilterInitStructure);
  
   CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);  // CAN1

}

void CAN_SendData(CAN_TypeDef* CANx,CanTxMsg* CanData)
{
    uint8_t retrys=0;
    uint8_t mailbox=0;

    do
   {
       mailbox=CAN_Transmit(CANx,CanData);
      retrys++;
   }
   while((mailbox==CAN_TxStatus_NoMailBox)&&(retrys<0xFE));
   retrys=0;
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * CRC
  */

void calcrc(uint8_t *crcbuf,uint8_t size,uint8_t count)
{
        uint8_t bit=0;
        uint8_t i=0;
        uint8_t pos=0;
        if(size-count<2)
                while(1);
        
        while(pos<count)
        {
                crc ^= crcbuf[pos];
                for(i=0; i<8; i++)
                {
        
                        bit = crc&0x1;
                        crc >>= 1;
                        crc=crc & 0x7fff;
                        if(1==bit)
                        {
                                crc ^= 0xa001;
                        }
                        crc=crc &0xffff;
                }
                pos++;
        }
        crcbuf[pos++]=crc & 0x00ff;
        crcbuf[pos]=crc & 0xff00;
}

/**
  * 
  */


TestStatus CAN_Polling(void)
{

  uint8_t TransmitMailbox = 0;
        int i=0;

  TransmitMailbox = CAN_Transmit(CAN1, &TxMessage);
  i = 0;
  while((CAN_TransmitStatus(CAN1, TransmitMailbox)  !=  CANTXOK) && (i  !=  0xFFFF))
  {
    i++;
  }

  i = 0;
  while((CAN_MessagePending(CAN1, CAN_FIFO0) < 1) && (i  !=  0xFFFF))
  {
    i++;
  }

  /* receive */
  RxMessage0.StdId = 0x00;
  RxMessage0.IDE = CAN_ID_STD;
  RxMessage0.DLC = 0;
  RxMessage0.Data[0] = 0x00;
  RxMessage0.Data[1] = 0x00;
  CAN_Receive(CAN1, CAN_FIFO0, &RxMessage0);

  if (RxMessage0.StdId != 0x01)
  {
    return FAILED;  
  }

  if (RxMessage0.IDE != CAN_ID_STD)
  {
    return FAILED;
  }

  if (RxMessage0.DLC != 8)
  {
    return FAILED;  
  }

  
  return PASSED; /* Test Passed */
}

TestStatus CAN_Interrupt(void)
{
  uint32_t i = 0;

  /* transmit 1 message */
  TxMessage.StdId = 0;
  TxMessage.IDE = CAN_ID_STD;
  TxMessage.RTR = CAN_RTR_DATA;
  TxMessage.DLC = 2;
  TxMessage.Data[0] = 0xDE;
  TxMessage.Data[1] = 0xCA;
  CAN_Transmit(CAN1, &TxMessage);

  /* initialize the value that will be returned */
  ret = 0xFF;
       
  /* Receive message with interrupt handling */
  i = 0;
  while((ret ==  0xFF) && (i < 0xFFF))
  {
    i++;
  }
  
  if (i ==  0xFFF)
  {
    ret = 0;  
  }

  /* disable interrupt handling */
  CAN_ITConfig(CAN1, CAN_IT_FMP0, DISABLE);

  return (TestStatus)ret;
}
u8 CAN1_Send_Msg(u32 can_message_id, u8* msg)
{   
  u8 mbox, CAN_transmit_message_number;
  u16 i=0;
  uint8_t requested;
  CanTxMsg TxMessage;
  can_message_id1 = can_message_id;
  requested = 0;
  if ( can_message_id == 0xf5070000)
   can_message_id = 0x7f5;
  requested = 1;
  if( requested == 1)
  {
     TxMessage.StdId= can_message_id;    // 标准标识符为0
     TxMessage.IDE=0;        // 使用扩展标识符
     TxMessage.RTR=0;        // 消息类型为数据帧，一帧8位
     TxMessage.DLC=8;                      // 发送两帧信息
     for(i=0;i<8;i++)
     TxMessage.Data[i]=msg[i];             // 第一帧信息          
     mbox= CAN_Transmit(CAN1, &TxMessage);   
     i=0;
     while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;   //等待发送结束
     if(i>=0XFFF)return 1;
     return 0;      
   }
}
u8 CAN1_Receive_Msg(u8 *buf)
{                  
    u32 i;
    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;      //没有接收到数据,直接退出 
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//读取数据   
          for(i=0;i<RxMessage.DLC;i++)
          buf[i]=RxMessage.Data[i];  
   return RxMessage.DLC;   
}
u8 CAN2_Receive_Msg(u8 *buf)
{                  
    u32 i;
    if( CAN_MessagePending(CAN2,CAN_FIFO0)==0)return 0;      //没有接收到数据,直接退出 
    CAN_Receive(CAN2, CAN_FIFO0, &RxMessage);//读取数据   
          for(i=0;i<RxMessage.DLC;i++)
          buf[i]=RxMessage.Data[i];  
   return RxMessage.DLC;   
}

