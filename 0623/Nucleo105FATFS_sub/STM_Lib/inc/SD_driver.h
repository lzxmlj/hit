/******************************************************************************

* SD_driver.h

* ������SD��������ص�Ӳ��ƽ̨����
  ����SD���ײ�����������������
  
* ��һ���ߣ�  �ͳ�   (���Ľ�����ͷ)
  �������������Ľ� (����������ͷ)
* �汾��V1.15 (��V1.13��v1.14�������,��ʹ��˫����ʱ��V1.10֮��İ汾�����)
* �汾����ʱ�䣺2012��1��30��
  
* ����оƬ��MK10N512VMD100
  ����ƽ̨��CodeWarrior 10.1

* (!!!����Ӳ��ƽ̨������޸�!!!)

******************************************************************************/

#ifndef _SD_PLATFINTERFACE_H_
#define _SD_PLATFINTERFACE_H_

#include "stm32f10x.h"    /***********����spi.c*********/
#include "stm32f10x_spi.h"


/**************************************Ӳ�����ö���**************************************/           
/* STM32F107SPI1�˿����� */
#define SPI_SD                      SPI1
#define SPI_SD_CLK                  RCC_APB2Periph_SPI1

#define SPI_SD_SCK_PIN              GPIO_Pin_5
#define SPI_SD_SCK_PORT             GPIOA
#define SPI_SD_SCK_CLK              RCC_APB2Periph_GPIOA

#define SPI_SD_MISO_PIN             GPIO_Pin_6
#define SPI_SD_MISO_PORT            GPIOA
#define SPI_SD_MISO_CLK             RCC_APB2Periph_GPIOA

#define SPI_SD_MOSI_PIN             GPIO_Pin_7
#define SPI_SD_MOSI_PORT            GPIOA
#define SPI_SD_MOSI_CLK             RCC_APB2Periph_GPIOA

#define SPI_SD_CS_PIN               GPIO_Pin_1      
#define SPI_SD_CS_PORT              GPIOB   
#define SPI_SD_CS_CLK               RCC_APB2Periph_GPIOB  

           

/*�ײ��������*/
#define DELAY_WaitRead_SD  0   /* ��ȡ1Byte����ʱ���е���ʱ(��λ10��nop) */
/* ����ʱʱ��ĳ����Ӵ�����������Ӳ����·�ȶ��Զ���,����K10,����ʹSPIʱ�ӳ���10M,����ҪԼ2us����ʱ */

/**************************�������Ͷ���**************************/
typedef unsigned char  byte_sd;      /*�����ֽ��ͱ���*/
typedef unsigned short word_sd;      /*�������ͱ���*/
typedef unsigned long  dword_sd;     /*����˫���ͱ���*/

#define  ON_SD_CS           GPIO_ResetBits(SPI_SD_CS_PORT, SPI_SD_CS_PIN)   /*����CS(ע��0Ϊѡͨ)*/
#define  OFF_SD_CS          GPIO_SetBits(SPI_SD_CS_PORT, SPI_SD_CS_PIN)     /*�ر�CS(ע��0Ϊѡͨ)*/

#define DELAY_TIME 2000 //SD���ĸ�λ���ʼ��ʱSPI����ʱ����������ʵ�������޸���ֵ����������SD����λ���ʼ��ʧ��
#define TRY_TIME 200   //��SD��д������֮�󣬶�ȡSD���Ļ�Ӧ����������TRY_TIME�Σ������TRY_TIME���ж�������Ӧ��������ʱ��������д��ʧ��

//�����붨��
//-------------------------------------------------------------
#define INIT_CMD0_ERROR     0x01 //CMD0����
#define INIT_CMD1_ERROR     0x02 //CMD1����
#define WRITE_BLOCK_ERROR   0x03 //д�����
#define READ_BLOCK_ERROR    0x04 //�������
//-------------------------------------------------------------

extern byte_sd SD_Reset(void);
extern byte_sd SD_Init(void);
extern   unsigned char GET_INFO_CARD(void);  //��ʼ����ʹ��CMD1��1�����
extern   unsigned char init_INFO_CARD(void);  //��ʼ����ʹ��CMD1��1�����
extern void SD_driver_Init(void);
extern byte_sd SD_Write_Sector(unsigned long addr,unsigned char *Buffer);
extern byte_sd SD_Read_Sector(unsigned long addr,unsigned char *buffer);
extern void SD_WriteByte(byte_sd jc_data);
extern byte_sd SD_ReadByte(byte_sd jc_data);
extern  u8 SD_GetResponse(u8 Response);
extern   u8 SD_ReceiveData(u8 *data, u16 len, u8 release);
extern   u8 SD_GetCSD(u8 *csd_data);
extern   u8 SD_ReadSingleBlock(u32 sector, u8 *buffer) ;
#endif
