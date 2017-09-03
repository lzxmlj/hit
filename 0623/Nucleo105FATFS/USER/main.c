#include "stm32f10x.h"
#include "usart.h"
#include "delay.h"
#include "rtc.h"
//#include "MMC_SD.h"
//#include "ff.h"
#define HUGE
CanTxMsg TxMsg1={0xAB,0,CAN_ID_STD,CAN_RTR_DATA,8,{0xAB,1,2,3,4,5,6,7}};
CanTxMsg TxMsg2={0xFF,0,CAN_ID_STD,CAN_RTR_DATA,8,{0xAC,0,0,0,0,0,0,0}};
u32 delay_time;
int test_temp;
extern u32 time_count_ms;
extern u8 complete_count,complete_count1;
extern  u16 count_canmessage1, count_canmessage2;
extern u8 loop_enable, VblistA_full;
unsigned char test_char;
u16 text_count, text_count_pre;
extern u32 time_count, test_count;
u32 time_count_pre, time_count_diff, time_count_curr, get_count[25];
u8 backup_type, irror_happen,switch_pre, need_update_rtc, send_data_check, run_reset, Vbfirst_time;
extern u8 CAN_need_update, test_buff[40970], CAN_need_update_data[8];
u8 Vbcan_pending, rtc_update, Roll_CAN_node, send_count, Vbfirst_time_update, send_can_count, count_second;
extern u8 receive_data[20], receive_count;
typedef enum CCP_Command_Code_tag
{
  CC_CONNECT  = 0x01,
  CC_SET_MTA  = 0x02,
  CC_DNLOAD  = 0x03,
  CC_UPLOAD  = 0x04,
  CC_TEST = 0x05, // OPTIONAL COMMAND
  CC_START_STOP= 0x06,
  CC_DISCONNECT= 0x07,
  CC_START_STOP_ALL = 0x08, // OPTIONAL COMMAND
  CC_GET_ACTIVE_CAL_PAGE = 0x09, // OPTIONAL COMMAND
  CC_SET_S_STATUS  = 0x0C, // OPTIONAL COMMAND
  CC_GET_S_STATUS  = 0x0D, // OPTIONAL COMMAND
  CC_BUILD_CHKSUM  = 0x0E, // OPTIONAL COMMAND
  CC_SHORT_UP = 0x0F, // OPTIONAL COMMAND
  CC_CLEAR_MEMORY  = 0x10, // OPTIONAL COMMAND
  CC_SELECT_CAL_PAGE= 0x11, // OPTIONAL COMMAND
  CC_GET_SEED = 0x12, // OPTIONAL COMMAND
  CC_UNLOCK  = 0x13, // OPTIONAL COMMAND
  CC_GET_DAQ_SIZE  = 0x14,
  CC_SET_DAQ_PTR  = 0x15,
  CC_WRITE_DAQ = 0x16,
  CC_EXCHANGE_ID  = 0x17,
  CC_PROGRAM  = 0x18, // OPTIONAL COMMAND
  CC_MOVE = 0x19, // OPTIONAL COMMAND
  CC_GET_CCP_VERSION= 0x1B,
  CC_DIAG_SERVICE  = 0x20, // OPTIONAL COMMAND
  CC_ACTION_SERVICE = 0x21, // OPTIONAL COMMAND
  CC_PROGRAM_6 = 0x22, // OPTIONAL COMMAND
  CC_DNLOAD_6 = 0x23 // OPTIONAL COMMAND

} CCP_Command_Code_T;
typedef struct CCP_DAQ_Resume_Data_tag
{
  u32 message_id;  /* from GET_DAQ_SIZE message */
  u16 transmission_rate_prescaler; /* from START_STOP message  */
  u8 event_channel_number;  /* from START_STOP message  */
  u8 first_PID;  /* from GET_DAQ_SIZE message */
  u8 last_ODT_to_transmit;  /* from START_STOP message  */
  u8 started; /* flag DAQ has been started */
  u8 dummy1; /* assure 32-bit alignment  */
  u8 dummy2; /* assure 32-bit alignment  */

} CCP_DAQ_Resume_Data_T;


typedef struct DAQ_Reference_tag
{
  u8 DAQ_number;
  u8 ODT_number;
  u8 element_number;

} DAQ_Reference_T;
typedef struct CCP_DAQ_Configuration_tag
{
  uint8_t number_of_ODTs;
  uint8_t number_of_elements_per_ODT;
  HUGE uint16_t*  config_buffer_ptr;
  HUGE uint8_t*data_buffer_ptr;

} CCP_DAQ_Configuration_T;
void EXIT_Config(void);
void TIM3_MS(void);
void TIM4_uS(void);
void Stm32_Clock_Init(u8 PLL);
void MYRCC_DeInit(void);
void CAN_APP_Main(void);
void clearCANbuffer();

extern void CAN1_Config();

extern void MYRCC_DeInit();
extern void CAN_SendData(CAN_TypeDef* CANx,CanTxMsg* CanData);
static void RCC_Configuration(void);
static u8 CTR, MTA_Number, Add_ext, DAQ_number, object_ODT, element_ODT, last_ODT_number, event_channel_no;
static u32 DTO_ID, pended_message;
static u16 transmit_rate;
u16 DAQ_ID[4];
u8 sizeofvalue, Add_ext_element, count_space;
u32 address_value;
void MY_NVIC_SetVectorTable(u32 NVIC_VectTab, u32 Offset);
void test_reset();

static void CAN_Analysis();
u8 Updateforhex(u8 number);
void GPIO_init_base(void);
u8 to_ascii(u8 a);
u8 to_hex(u8 b);
u8 to_hex(u8 b)
{
  if( ( b >= 0x30 ) && (b <= 0x39))
  {
 return( b - 0x30);
  }
  else
  {
 return(b - 0x41 + 10);
  }
}

u8 to_ascii(u8 a)
{
  if( ( a >= 0 ) && (a <= 9))
  {
 return( a + 48);
  }
  else
  {
 return(a + 0x61 - 10);
  }
}

void LED_Init(void)
{

}
void nDelay(vu32 nCount)
{
  while(nCount--);
}
void RTE_INITIAL(void);
void Usart_Send(u8 ch);
u8 CAN_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);


u32 br1;
u16 txid_byte, rxid_byte, Adress_support;


u32 baudrate, temp1;

u32 file_result=0;
int main(void)
{
  u32 i, j,k,l,m;
 //RCC_Configuration();
    Stm32_Clock_Init(9);
   uart_init(72,230400);
    TIM3_MS();
    TIM4_uS();
// CAN1_Config();
    RTC_Init();
    CAN_Mode_Init(1,8,9,4,0);//CAN初始化,波特率500Kbps  
    GPIO_init_base();
    EXIT_Config();
    RTE_INITIAL();
    Roll_CAN_node = 2;

   text_count = 0;
   text_count_pre = 0;
   backup_type = 0;

  while(1)
  {
    if(loop_enable == 1)
    {
     loop_enable = 0;
     if(run_reset == 1)
        test_reset();
     //CAN_APP_Main();
     if(Vbfirst_time_update == 3)
     {
      if(pended_message != test_count)
      {
         Usart_Send(0x55);
         Usart_Send(12);
         for(i = pended_message; i < pended_message + 10;i++)
         {
            Usart_Send(test_buff[i]);
         }
         Usart_Send(0xAA);
         pended_message = pended_message + 10;
      }
      else
      {
         if(Vbcan_pending == 1)
         {
            if((CAN_need_update != 12) && (CAN_need_update_data[0] != 0xFF))
            {
               Usart_Send(0x55);
               Usart_Send(CAN_need_update);
               Usart_Send(calendar.w_month);
               for(i = 0; i < 8;i++)
               {
                  Usart_Send(CAN_need_update_data[i]);
               }
               

               if(get_count[CAN_need_update] == 4096)
                  Usart_Send(0xFF);
               else
                  Usart_Send(1);
               Usart_Send(0xAA);
               get_count[CAN_need_update] = get_count[CAN_need_update] + 1;
               //TxMsg2.Data[0] = CAN_need_update;
               //TxMsg2.Data[1] = 0xAA;

               //CAN_SendData(CAN1, &TxMsg2);
               //clearCANbuffer();
               CAN_need_update = 12;
            }
            else if(CAN_need_update_data[0] == 0xFF)
            {
							 if(Roll_CAN_node != 25)
							 {
								 Roll_CAN_node = Roll_CAN_node + 1;
						 	 }
               else
							 {
                  Roll_CAN_node = 1;
							 }
               TxMsg2.Data[0] = 0xFF;
               TxMsg2.Data[1] = Roll_CAN_node;
               TxMsg2.Data[7] = 0xFF;
               CAN_SendData(CAN1, &TxMsg2);
               clearCANbuffer();
							CAN_need_update = 12;
            }
            Vbcan_pending = 0;
            
            send_count = 0;
          }
          else
          {

            if( send_count < 50)
            {
               send_count= send_count + 1;
            }
            else
            {
               send_count = 0;
                if(Roll_CAN_node != 25)
                  Roll_CAN_node = Roll_CAN_node + 1;
               else
                  Roll_CAN_node = 1;
            }
               TxMsg2.Data[0] = 0xFF;
               TxMsg2.Data[1] = Roll_CAN_node;
               TxMsg2.Data[7] = 0xFF;
               CAN_SendData(CAN1, &TxMsg2);
               clearCANbuffer();
         }
         }
      }
      }
   }
}
void Usart_Send(u8 ch)
{
   send_data_check = to_ascii(ch / 0x10);
   while((USART1->SR&0X40)==0);//等待发送结束

  USART1->DR= send_data_check;
  while((USART1->SR&0X40)==0);//等待发送结束
  send_data_check = to_ascii(ch & 0xF);
  USART1->DR= send_data_check;
}
static void RCC_Configuration(void)
{  
 /* Setup the microcontroller system. Initialize the Embedded Flash Interface, 
initialize the PLL and update the SystemFrequency variable. */
 SystemInit();
 
}
void TIM4_uS(void)
{

  GPIO_InitTypeDef GPIO_InitStructure; 
  NVIC_InitTypeDef NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_ICInitTypeDef TIM_ICInitStructure;
  
  RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM4, ENABLE);
 TIM_DeInit(TIM4);
  TIM_TimeBaseStructure.TIM_Period = 999;
  TIM_TimeBaseStructure.TIM_Prescaler = 72-1;//1439
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  TIM_ClearFlag(TIM4, TIM_FLAG_Update);
  TIM_ITConfig(TIM4,TIM_IT_CC1,ENABLE);
  TIM_Cmd(TIM4,ENABLE);
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void TIM3_MS(void)
{

  GPIO_InitTypeDef GPIO_InitStructure; 
  NVIC_InitTypeDef NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_ICInitTypeDef TIM_ICInitStructure;
  
  RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM3, ENABLE);
  TIM_DeInit(TIM3);
  TIM_TimeBaseStructure.TIM_Period = 1999;
  TIM_TimeBaseStructure.TIM_Prescaler = 36000-1;//1439
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  TIM_ClearFlag(TIM3, TIM_FLAG_Update);
  TIM_ITConfig(TIM3,TIM_IT_CC1,ENABLE);
  TIM_Cmd(TIM3,ENABLE);
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void RTE_INITIAL(void)
{

  GPIO_InitTypeDef GPIO_InitStructure; 
  NVIC_InitTypeDef NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_ICInitTypeDef TIM_ICInitStructure;
  
  RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2, ENABLE);
 TIM_DeInit(TIM2);
  TIM_TimeBaseStructure.TIM_Period = 29;
  TIM_TimeBaseStructure.TIM_Prescaler = 36000-1;//1439
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  TIM_ClearFlag(TIM2, TIM_FLAG_Update);
  TIM_ITConfig(TIM2,TIM_IT_CC1,ENABLE);
  TIM_Cmd(TIM2,ENABLE);
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void GPIO_init_base(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  CAN_InitTypeDef  CAN_InitStructure;
  CAN_FilterInitTypeDef CAN_FilterInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* 打开GPIO时钟、AFIO时钟，CAN时钟 */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
  
  NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
void EXIT_Config(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;
  EXTI_InitTypeDef exti;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource2); 
  
  EXTI_InitStructure.EXTI_Line = EXTI_Line2; //线路0
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;  //触发模式为中断
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  //下降沿触发
  EXTI_InitStructure.EXTI_LineCmd = ENABLE; //开外部中断
  EXTI_Init(&EXTI_InitStructure);
  EXTI_ClearITPendingBit(EXTI_Line2);  //清外部线路0中断
}
void Stm32_Clock_Init(u8 PLL)
{
unsigned char temp=0;  
MYRCC_DeInit(); //复位并配置向量表
 RCC->CR|=0x00010000; //外部高速时钟使能HSEON
while(!(RCC->CR>>17));//等待外部时钟就绪
RCC->CFGR=0X00000400; //APB1=DIV2;APB2=DIV1;AHB=DIV1;
PLL-=2; //抵消2个单位（因为是从2开始的，设置0就是2）
RCC->CFGR|=PLL<<18;  //设置PLL值 2~16
RCC->CFGR|=1<<16; //PLLSRC ON 
FLASH->ACR|=0x32; //FLASH 2个延时周期
RCC->CR|=0x01000000; //PLLON
while(!(RCC->CR>>25));//等待PLL锁定
RCC->CFGR|=0x00000002;//PLL作为系统时钟 
while(temp!=0x02)//等待PLL作为系统时钟设置成功
{  
temp=RCC->CFGR>>2;
temp&=0x03;
}  
RCC->BDCR|=1<<0;         //开启外部低速振荡器   

}
void MYRCC_DeInit(void)
{
 RCC->APB1RSTR = 0x00000000;//复位结束 
RCC->APB2RSTR = 0x00000000; 
 
 RCC->AHBENR = 0x00000014; //睡眠模式闪存和SRAM时钟使能.其他关闭. 
 RCC->APB2ENR = 0x00000000; //外设时钟关闭.  
 RCC->APB1ENR = 0x00000000;  
RCC->CR |= 0x00000001;//使能内部高速时钟HSION 
RCC->CFGR &= 0xF8FF0000;  //复位SW[1:0],HPRE[3:0],PPRE1[2:0],PPRE2[2:0],ADCPRE[1:0],MCO[2:0] 
RCC->CR &= 0xFEF6FFFF;//复位HSEON,CSSON,PLLON
RCC->CR &= 0xFFFBFFFF;//复位HSEBYP
RCC->CFGR &= 0xFF80FFFF;  //复位PLLSRC, PLLXTPRE, PLLMUL[3:0] and USBPRE 
RCC->CIR = 0x00000000;//关闭所有中断 
//配置向量表 
#ifdef VECT_TAB_RAM
MY_NVIC_SetVectorTable(0x20000000, 0x0);
#else  
MY_NVIC_SetVectorTable(0x08000000,0x0);
#endif
}
void MY_NVIC_SetVectorTable(u32 NVIC_VectTab, u32 Offset) 
{
SCB->VTOR = NVIC_VectTab|(Offset & (u32)0x1FFFFF80);//设置NVIC的向量表偏移寄存器
}
//用于标识向量表是在CODE区还是在RAM区
u8 CAN_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{
 u16 i=0;
 if(tsjw==0||tbs2==0||tbs1==0||brp==0)return 1;
 tsjw-=1;//先减去1.再用于设置
 tbs2-=1;
 tbs1-=1;
 brp-=1;

 RCC->APB2ENR|=1<<2;//使能PORTA时钟 
 GPIOA->CRH&=0XFFF00FFF; 
 GPIOA->CRH|=0X000B8000; //PA11 RX,PA12 TX推挽输出
  GPIOA->ODR|=3<<11;
  
 RCC->APB1ENR|=1<<25; //使能CAN时钟 CAN使用的是APB1的时钟(max:36M)
 CAN1->MCR=0x0000; //退出睡眠模式(同时设置所有位为0)
 CAN1->MCR|=1<<0; //请求CAN进入初始化模式
 while((CAN1->MSR&1<<0)==0)
 {
 i++;
 if(i>100)return 2; //进入初始化模式失败
 }
 CAN1->MCR|=0<<7; //非时间触发通信模式
 CAN1->MCR|=0<<6; //软件自动离线管理
 CAN1->MCR|=0<<5; //睡眠模式通过软件唤醒(清除CAN1->MCR的SLEEP位)
 CAN1->MCR|=1<<4; //禁止报文自动传送
 CAN1->MCR|=0<<3; //报文不锁定,新的覆盖旧的
 CAN1->MCR|=0<<2; //优先级由报文标识符决定
 CAN1->BTR=0x00000000; //清除原来的设置.
 CAN1->BTR|=mode<<30; //模式设置 0,普通模式;1,回环模式;
 CAN1->BTR|=tsjw<<24; //重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位
 CAN1->BTR|=tbs2<<20; //Tbs2=tbs2+1个时间单位
 CAN1->BTR|=tbs1<<16; //Tbs1=tbs1+1个时间单位
 CAN1->BTR|=brp<<0;  //分频系数(Fdiv)为brp+1
 //波特率:Fpclk1/((Tbs1+Tbs2+1)*Fdiv)
 CAN1->MCR&=~(1<<0); //请求CAN退出初始化模式
 while((CAN1->MSR&1<<0)==1)
 {
 i++;
 if(i>0XFFF0)return 3;//退出初始化模式失败
 }
 //过滤器初始化
 CAN1->FMR|=1<<0; //过滤器组工作在初始化模式
 CAN1->FA1R&=~(1<<0); //过滤器0不激活
 CAN1->FS1R|=1<<0;  //过滤器位宽为32位.
 CAN1->FM1R|=0<<0; //过滤器0工作在标识符屏蔽位模式
 CAN1->FFA1R|=0<<0; //过滤器0关联到FIFO0
 CAN1->sFilterRegister[0].FR1=0X00000000;//32位ID
 CAN1->sFilterRegister[0].FR2=0X00000000;//32位MASK
 CAN1->FA1R|=1<<0; //激活过滤器0
 CAN1->FMR&=0<<0; //过滤器组进入正常模式

#if 1
 //使用中断接收
 CAN1->IER|=1<<1; //FIFO0消息挂号中断允许.
 MY_NVIC_Init(1,0,USB_LP_CAN1_RX0_IRQn,2);//组2
#endif
 return 0;
}  
void CAN_APP_Main(void)
{
   u8 calendar_min, calendar_sec, calendar_hour, i, calendar_month, calendar_date;
   if( ( rtc_update == 1))
   {
      TxMsg2.Data[0] = calendar.w_month;
      TxMsg2.Data[1] = calendar.w_date;
      TxMsg2.Data[2] = calendar.hour;
      TxMsg2.Data[3] = calendar.min;
      TxMsg2.Data[4] = calendar.sec;
      TxMsg2.Data[5] = time_count_ms / 0x100;
      TxMsg2.Data[6] = time_count_ms & 0xFF;
      TxMsg2.Data[7] = 0x55;
      //Vbfirst_time++;

      CAN_SendData(CAN1, &TxMsg2);
      clearCANbuffer();
		 if(send_can_count != 5)
		 {

      //delay_ms(10000);

			 send_can_count = send_can_count + 1;
		 }
		 else
		 {
		    send_can_count = 0;
			 rtc_update = 0;
			 Vbfirst_time_update = 2;
		 }
   }
   if(receive_count == 14)
   {
      calendar_month = to_hex(receive_data[2]) * 0x10 + to_hex(receive_data[3]);
      calendar_date = to_hex(receive_data[4]) * 0x10 + to_hex(receive_data[5]);
      calendar_hour = to_hex(receive_data[6]) * 0x10 + to_hex(receive_data[7]);
      calendar_min = to_hex(receive_data[8]) * 0x10 + to_hex(receive_data[9]);
      calendar_sec = to_hex(receive_data[10]) * 0x10 + to_hex(receive_data[11]);
      RTC_Set(2017,calendar_month,calendar_date,calendar_hour,calendar_min,calendar_sec);
      receive_count = 0;
      rtc_update = 1;
      Vbfirst_time_update = 1;
   }
   else if((Vbfirst_time_update != 1) && (Vbfirst_time_update != 2) && (Vbfirst_time_update !=3))
   {
      count_second = count_second + 1;
      if(count_second == 10)
      {
      count_second = 0;
      for(i = 0; i < 13; i++)
      {
         Usart_Send(0xFF);
      }
      }
   }
   if(Vbfirst_time_update == 2)
   {
		 
      Vbfirst_time_update = 3;
   }
  #if 0
  else if(Vbfirst_time == 130)
  {
     TxMsg2.Data[0] = calendar.sec;
     TxMsg2.Data[1] = time_count_ms / 0x100;
     TxMsg2.Data[2] = time_count_ms & 0xFF;
     TxMsg2.Data[3] = TIM_GetCounter(TIM4) / 0x100;
     TxMsg2.Data[4] = TIM_GetCounter(TIM4) & 0xFF;
     TxMsg2.Data[5] = calendar.min;
     TxMsg2.Data[7] = 0x99;
     Vbfirst_time = 100;
  }
  #endif



}
void test_reset()
{
   u8 i;
   for(i = 0;i < 8; i++)
   {
      TxMsg2.Data[0] = 0x11;
   }
   CAN_SendData(CAN1, &TxMsg2);
   clearCANbuffer();
   nDelay(65535);
   NVIC_SystemReset();
}
void clearCANbuffer()
{
   u8 i;
   for(i = 0; i < 8; i++)
   {
      TxMsg2.Data[i] = 0;
   }
}
