/******************************************************************************

* SD_System.h

* 定义是否使能SD卡(通过定义"EN_SD");
  定义sd卡系统的相关常量;
  对全局变量以及接口函数进行声明
    
* 第一作者：  纪成   (第四届摄像头)
  完善与整理：孙文健 (第六届摄像头)
* 版本：V1.15 (与V1.13、v1.14版相兼容,不使能双缓存时与V1.10之后的版本相兼容)
* 版本更新时间：2012年1月30日
 
******************************************************************************/
                       
#ifndef _SD_SYSTEM_H_    
#define _SD_SYSTEM_H_


#define EN_SD     /* !!!注释掉该句则屏蔽SD卡!!! */

#ifdef EN_SD

/**************************功能配置**************************/
#define EN_SDbuf       /* !!!注释掉该句则不使用SD双缓存模式!!! */
/*
使用双缓存会增加520字节左右的RAM开销,但在大数情况下会增加SD卡的写入速度
注意:如果卡的质量太差(即坏扇区过多,这种方法不仅不会加快速度,反而会略微降低写入速度)
*/
#define EN_Cnt_SDbuf   /* !!!注释掉该句则不进行SD第二缓存使用情况的统计!!! */
/*
增加缓存统计功能会额外消耗10字节内存,并略微降低运行速度
*/


#include "./SD_driver.h"            

/**************************数据常量定义**************************/
#define SD_MAX_cluster 700  /*一个文件最大可能占的簇数(按8个扇区一簇的话,700大约为1min)*/
#define SD_MAX_segment 35   /*最多的簇组数(一个簇组可能有1-250个簇)*/

#define SD_LONG_Wait   2000000L /*等待SD卡的长延时时限*/
#define SD_SHORT_Wait  5000L    /*等待SD卡的短延时时限*/
#define SD_VST_Wait    4000L    /*等待SD卡的超短延时时限*/

//SD卡错误类型对应表      /************By Sword************/
//(在线仿真时可观察变量flag_err_sd的值,然后按照其值查该表就可知道SD卡出错的具体情况)
#define SD_Normal            0   //SD卡初始化正常
#define SDERR_CMD0           1   //发送CMD0指令时出错
#define SDERR_ACMD41         2   //发送ACMD41指令时出错
#define SDERR_ReadOverTime   3   //读取超时
#define SDERR_StartSector    4   //启动扇区出错
#define SDERR_NoCluster      5   //没有找到足够的簇
   
    
/*IO操作定义*/
#define  ON_SD_CS           GPIO_ResetBits(SPI_SD_CS_PORT, SPI_SD_CS_PIN)   /*开启CS(注意0为选通)*/
#define  OFF_SD_CS          GPIO_SetBits(SPI_SD_CS_PORT, SPI_SD_CS_PIN)     /*关闭CS(注意0为选通)*/

     
/**************************全局变量声明**************************/                     
extern byte_sd sd_yes;          /* 判断是否用SD卡(没有为0，有为1) */
extern byte_sd flag_sdhc;       /* sdhc卡标志,=1有效 */
extern byte_sd flag_test_sd;    /* 调试使用 */  
extern byte_sd flag_err_sd;     /* SD出错标志 */
extern byte_sd Flag_SDRdy;      /* SD卡就绪标志(=1表SD卡就绪) */
extern byte_sd sd_initover;     /* SD卡初始化结束标志 */

#ifdef EN_SDbuf
#ifdef EN_Cnt_SDbuf
extern word_sd Cnt_SDbufS1,Cnt_SDbufS2,Cnt_SDbufS3,Cnt_SDbufS4,Cnt_SDbufS5;
#endif
#endif



/**************************API函数声明**************************/
/*API函数*/
extern void writebyte_ram(byte_sd wdata); /*写字节到ram区*/
extern void write_stop(void);/*停止写*/
extern void SD_System_Init(void); /************By Sword************/
extern void SD_System_Init_Ext(byte_sd * pbuf , word_sd len);
extern void test_sd(void);        /************By Sword************/

/*Fat32操作*/
extern void writeram_sd(void); /*ram区满512时写SD卡*/
extern void search_addr_sd(void); /************By Sword************/
extern void search_fat(void);     /************By Sword************/
extern void fat32_init(void);     /************By Sword************/

#endif

#endif 
