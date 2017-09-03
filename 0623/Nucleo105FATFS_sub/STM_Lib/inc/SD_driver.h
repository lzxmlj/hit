/******************************************************************************

* SD_driver.h

* 定义与SD卡操作相关的硬件平台配置
  并对SD卡底层驱动函数进行声明
  
* 第一作者：  纪成   (第四届摄像头)
  完善与整理：孙文健 (第六届摄像头)
* 版本：V1.15 (与V1.13、v1.14版相兼容,不使能双缓存时与V1.10之后的版本相兼容)
* 版本更新时间：2012年1月30日
  
* 主控芯片：MK10N512VMD100
  开发平台：CodeWarrior 10.1

* (!!!更改硬件平台后必须修改!!!)

******************************************************************************/

#ifndef _SD_PLATFINTERFACE_H_
#define _SD_PLATFINTERFACE_H_

#include "stm32f10x.h"    /***********包含spi.c*********/
#include "stm32f10x_spi.h"


/**************************************硬件配置定义**************************************/           
/* STM32F107SPI1端口配置 */
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

           

/*底层参数定义*/
#define DELAY_WaitRead_SD  0   /* 读取1Byte数据时进行的延时(单位10个nop) */
/* 该延时时间的长短视处理器特性与硬件电路稳定性而定,对于K10,若想使SPI时钟超过10M,则需要约2us的延时 */

/**************************数据类型定义**************************/
typedef unsigned char  byte_sd;      /*定义字节型变量*/
typedef unsigned short word_sd;      /*定义字型变量*/
typedef unsigned long  dword_sd;     /*定义双字型变量*/

#define  ON_SD_CS           GPIO_ResetBits(SPI_SD_CS_PORT, SPI_SD_CS_PIN)   /*开启CS(注意0为选通)*/
#define  OFF_SD_CS          GPIO_SetBits(SPI_SD_CS_PORT, SPI_SD_CS_PIN)     /*关闭CS(注意0为选通)*/

#define DELAY_TIME 2000 //SD卡的复位与初始化时SPI的延时参数，根据实际速率修改其值，否则会造成SD卡复位或初始化失败
#define TRY_TIME 200   //向SD卡写入命令之后，读取SD卡的回应次数，即读TRY_TIME次，如果在TRY_TIME次中读不到回应，产生超时错误，命令写入失败

//错误码定义
//-------------------------------------------------------------
#define INIT_CMD0_ERROR     0x01 //CMD0错误
#define INIT_CMD1_ERROR     0x02 //CMD1错误
#define WRITE_BLOCK_ERROR   0x03 //写块错误
#define READ_BLOCK_ERROR    0x04 //读块错误
//-------------------------------------------------------------

extern byte_sd SD_Reset(void);
extern byte_sd SD_Init(void);
extern   unsigned char GET_INFO_CARD(void);  //初始化，使用CMD1（1号命令）
extern   unsigned char init_INFO_CARD(void);  //初始化，使用CMD1（1号命令）
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
