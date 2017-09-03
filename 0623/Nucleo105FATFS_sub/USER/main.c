#include "stm32f10x.h"
#include "usart.h"
#include "delay.h"
#include "rtc.h"
//#include "MMC_SD.h"
//#include "ff.h"
#define HUGE
CanTxMsg TxMsg1={0xAB,0,CAN_ID_STD,CAN_RTR_DATA,8,{0xAB,1,2,3,4,5,6,7}};
CanTxMsg TxMsg2={1,0,CAN_ID_STD,CAN_RTR_DATA,8,{0xAC,0,0,0,0,0,0,0}};
u32 delay_time;
int test_temp;
extern u32 time_count_ms;
extern u8 complete_count,complete_count1;
extern  u16 count_canmessage1, count_canmessage2;
extern u8 loop_enable, VblistA_full, Vbcould_update;;
unsigned char test_char;
u16 text_count, text_count_pre;
extern u32 time_count, test_count;
u32 test_count_pre;
u32 time_count_pre, time_count_diff, time_count_curr;
u8 backup_type, irror_happen,switch_pre, need_update_rtc, date_time_update;
extern u8 CAN_need_update, test_buff[40970], CAN_need_update_data[8];
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

extern void CAN1_Config();
void clearCANbuffer();

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
     RTC_Init();

    TIM3_MS();
    TIM4_uS();
// CAN1_Config();

    CAN_Mode_Init(1,8,9,4,0);//CAN��ʼ��,������500Kbps  
    GPIO_init_base();
    //uart_init(72,230400);
    EXIT_Config();
   RTE_INITIAL();

   text_count = 0;
   text_count_pre = 0;
   backup_type = 0;

  while(1)
  {
    if(loop_enable == 1)
    {
     loop_enable = 0;
     //Usart_Send(0x55);
       CAN_APP_Main();
      //CAN_SendData(CAN1, &TxMsg2);
      if((CAN_need_update_data[0] == 0xFF) && (CAN_need_update_data[1] == TxMsg2.StdId) && ((CAN_need_update_data[7] == 0xFF)))
      {
         Vbcould_update = 1;
         CAN_need_update_data[0] = 0;
         CAN_need_update_data[1] = 0;
      }
      if ((Vbcould_update == 1) && (pended_message != test_count)&& (test_count_pre == test_count))
      {
          j = 0;
         Vbcould_update = 0;

         if(pended_message <= ( test_count) )
         {
            for(i = pended_message +1; i < pended_message + 9;i++)
            {
               TxMsg2.Data[j] = test_buff[i];
               j = j + 1;
            }
            CAN_SendData(CAN1, &TxMsg2);
            clearCANbuffer();
            pended_message = pended_message + 10;
         }

      }
      else if((Vbcould_update == 1))
      {
                Vbcould_update = 0;
        TxMsg2.Data[0] = 0xFF;
        
        CAN_SendData(CAN1, &TxMsg2);
        clearCANbuffer();
      }

      test_count_pre = test_count;
      }
   }
}
void Usart_Send(u8 ch)
{
  USART2->DR= to_ascii(ch / 0x10);
  while((USART2->SR&0X40)==0);//�ȴ����ͽ���
  USART2->DR= to_ascii(ch & 0xF);
  while((USART2->SR&0X40)==0);//�ȴ����ͽ���
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
  TIM_TimeBaseStructure.TIM_Period = 19;
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

  /* ����GPIOʱ�ӡ�AFIOʱ�ӣ�CANʱ�� */
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
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource2); 
  
  EXTI_InitStructure.EXTI_Line = EXTI_Line2; //��·0
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;  //����ģʽΪ�ж�
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  //�½��ش���
  EXTI_InitStructure.EXTI_LineCmd = ENABLE; //���ⲿ�ж�
  EXTI_Init(&EXTI_InitStructure);
  EXTI_ClearITPendingBit(EXTI_Line2);  //���ⲿ��·0�ж�
}
void Stm32_Clock_Init(u8 PLL)
{
unsigned char temp=0;  
MYRCC_DeInit(); //��λ������������
 RCC->CR|=0x00010000; //�ⲿ����ʱ��ʹ��HSEON
while(!(RCC->CR>>17));//�ȴ��ⲿʱ�Ӿ���
RCC->CFGR=0X00000400; //APB1=DIV2;APB2=DIV1;AHB=DIV1;
PLL-=2; //����2����λ����Ϊ�Ǵ�2��ʼ�ģ�����0����2��
RCC->CFGR|=PLL<<18;  //����PLLֵ 2~16
RCC->CFGR|=1<<16; //PLLSRC ON 
FLASH->ACR|=0x32; //FLASH 2����ʱ����
RCC->CR|=0x01000000; //PLLON
while(!(RCC->CR>>25));//�ȴ�PLL����
RCC->CFGR|=0x00000002;//PLL��Ϊϵͳʱ�� 
while(temp!=0x02)//�ȴ�PLL��Ϊϵͳʱ�����óɹ�
{  
temp=RCC->CFGR>>2;
temp&=0x03;
}  
RCC->BDCR|=1<<0;         //�����ⲿ����������   

}
void MYRCC_DeInit(void)
{
 RCC->APB1RSTR = 0x00000000;//��λ���� 
RCC->APB2RSTR = 0x00000000; 
 
 RCC->AHBENR = 0x00000014; //˯��ģʽ������SRAMʱ��ʹ��.�����ر�. 
 RCC->APB2ENR = 0x00000000; //����ʱ�ӹر�.  
 RCC->APB1ENR = 0x00000000;  
RCC->CR |= 0x00000001;//ʹ���ڲ�����ʱ��HSION 
RCC->CFGR &= 0xF8FF0000;  //��λSW[1:0],HPRE[3:0],PPRE1[2:0],PPRE2[2:0],ADCPRE[1:0],MCO[2:0] 
RCC->CR &= 0xFEF6FFFF;//��λHSEON,CSSON,PLLON
RCC->CR &= 0xFFFBFFFF;//��λHSEBYP
RCC->CFGR &= 0xFF80FFFF;  //��λPLLSRC, PLLXTPRE, PLLMUL[3:0] and USBPRE 
RCC->CIR = 0x00000000;//�ر������ж� 
//���������� 
#ifdef VECT_TAB_RAM
MY_NVIC_SetVectorTable(0x20000000, 0x0);
#else  
MY_NVIC_SetVectorTable(0x08000000,0x0);
#endif
}
void MY_NVIC_SetVectorTable(u32 NVIC_VectTab, u32 Offset) 
{
SCB->VTOR = NVIC_VectTab|(Offset & (u32)0x1FFFFF80);//����NVIC��������ƫ�ƼĴ���
}
//���ڱ�ʶ����������CODE��������RAM��
u8 CAN_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{
 u16 i=0;
 if(tsjw==0||tbs2==0||tbs1==0||brp==0)return 1;
 tsjw-=1;//�ȼ�ȥ1.����������
 tbs2-=1;
 tbs1-=1;
 brp-=1;

 RCC->APB2ENR|=1<<2;//ʹ��PORTAʱ�� 
 GPIOA->CRH&=0XFFF00FFF; 
 GPIOA->CRH|=0X000B8000; //PA11 RX,PA12 TX��������
  GPIOA->ODR|=3<<11;
  
 RCC->APB1ENR|=1<<25; //ʹ��CANʱ�� CANʹ�õ���APB1��ʱ��(max:36M)
 CAN1->MCR=0x0000; //�˳�˯��ģʽ(ͬʱ��������λΪ0)
 CAN1->MCR|=1<<0; //����CAN������ʼ��ģʽ
 while((CAN1->MSR&1<<0)==0)
 {
 i++;
 if(i>100)return 2; //������ʼ��ģʽʧ��
 }
 CAN1->MCR|=0<<7; //��ʱ�䴥��ͨ��ģʽ
 CAN1->MCR|=0<<6; //�����Զ����߹���
 CAN1->MCR|=0<<5; //˯��ģʽͨ����������(����CAN1->MCR��SLEEPλ)
 CAN1->MCR|=1<<4; //��ֹ�����Զ�����
 CAN1->MCR|=0<<3; //���Ĳ�����,�µĸ��Ǿɵ�
 CAN1->MCR|=0<<2; //���ȼ��ɱ��ı�ʶ������
 CAN1->BTR=0x00000000; //����ԭ��������.
 CAN1->BTR|=mode<<30; //ģʽ���� 0,��ͨģʽ;1,�ػ�ģʽ;
 CAN1->BTR|=tsjw<<24; //����ͬ����Ծ����(Tsjw)Ϊtsjw+1��ʱ�䵥λ
 CAN1->BTR|=tbs2<<20; //Tbs2=tbs2+1��ʱ�䵥λ
 CAN1->BTR|=tbs1<<16; //Tbs1=tbs1+1��ʱ�䵥λ
 CAN1->BTR|=brp<<0;  //��Ƶϵ��(Fdiv)Ϊbrp+1
 //������:Fpclk1/((Tbs1+Tbs2+1)*Fdiv)
 CAN1->MCR&=~(1<<0); //����CAN�˳���ʼ��ģʽ
 while((CAN1->MSR&1<<0)==1)
 {
 i++;
 if(i>0XFFF0)return 3;//�˳���ʼ��ģʽʧ��
 }
 //��������ʼ��
 CAN1->FMR|=1<<0; //�������鹤���ڳ�ʼ��ģʽ
 CAN1->FA1R&=~(1<<0); //������0������
 CAN1->FS1R|=1<<0;  //������λ��Ϊ32λ.
 CAN1->FM1R|=0<<0; //������0�����ڱ�ʶ������λģʽ
 CAN1->FFA1R|=0<<0; //������0������FIFO0
 CAN1->sFilterRegister[0].FR1=0X00000000;//32λID
 CAN1->sFilterRegister[0].FR2=0X00000000;//32λMASK
 CAN1->FA1R|=1<<0; //����������0
 CAN1->FMR&=0<<0; //����������������ģʽ

#if 1
 //ʹ���жϽ���
 CAN1->IER|=1<<1; //FIFO0��Ϣ�Һ��ж�����.
 MY_NVIC_Init(1,0,USB_LP_CAN1_RX0_IRQn,2);//��2
#endif
 return 0;
}  
void CAN_APP_Main(void)
{
   u16 Counter_main;
   if(CAN_need_update_data[7] == 0x55)
   {
      RTC_Set(2017, CAN_need_update_data[0],CAN_need_update_data[1], CAN_need_update_data[2], CAN_need_update_data[3],
         CAN_need_update_data[4]  );
      time_count_ms  = CAN_need_update_data[5] * 0x100;
      time_count_ms  = time_count_ms + CAN_need_update_data[6] & 0xFF;
      CAN_need_update_data[7] = 0;
      date_time_update = 1;
   }
   else if((CAN_need_update_data[7] == 0x99))
   {
      calendar.sec = CAN_need_update_data[0];
      time_count_ms  = CAN_need_update_data[1] * 0x100;
      time_count_ms  = time_count_ms + (CAN_need_update_data[2] & 0xFF);
      Counter_main  = CAN_need_update_data[3] * 0x100;
      Counter_main  = Counter_main + (CAN_need_update_data[4] & 0xFF);
		 
      TIM_SetCounter(TIM4,Counter_main);
      CAN_need_update_data[7] = 0;
   }
   else if(CAN_need_update_data[0] == 0x11)
   {
      CAN_need_update_data[0] = 0;
     NVIC_SystemReset();
   }
   if(date_time_update == 0)
   {
      test_count = 0;
      pended_message = 0;
   }

}

void clearCANbuffer()
{
   u8 i;
   for(i = 0; i < 8; i++)
   {
      TxMsg2.Data[i] = 0;
   }
}

