/******************************************************************************

* SD_System.h

* �����Ƿ�ʹ��SD��(ͨ������"EN_SD");
  ����sd��ϵͳ����س���;
  ��ȫ�ֱ����Լ��ӿں�����������
    
* ��һ���ߣ�  �ͳ�   (���Ľ�����ͷ)
  �������������Ľ� (����������ͷ)
* �汾��V1.15 (��V1.13��v1.14�������,��ʹ��˫����ʱ��V1.10֮��İ汾�����)
* �汾����ʱ�䣺2012��1��30��
 
******************************************************************************/
                       
#ifndef _SD_SYSTEM_H_    
#define _SD_SYSTEM_H_


#define EN_SD     /* !!!ע�͵��þ�������SD��!!! */

#ifdef EN_SD

/**************************��������**************************/
#define EN_SDbuf       /* !!!ע�͵��þ���ʹ��SD˫����ģʽ!!! */
/*
ʹ��˫���������520�ֽ����ҵ�RAM����,���ڴ�������»�����SD����д���ٶ�
ע��:�����������̫��(������������,���ַ�����������ӿ��ٶ�,��������΢����д���ٶ�)
*/
#define EN_Cnt_SDbuf   /* !!!ע�͵��þ��򲻽���SD�ڶ�����ʹ�������ͳ��!!! */
/*
���ӻ���ͳ�ƹ��ܻ��������10�ֽ��ڴ�,����΢���������ٶ�
*/


#include "./SD_driver.h"            

/**************************���ݳ�������**************************/
#define SD_MAX_cluster 700  /*һ���ļ�������ռ�Ĵ���(��8������һ�صĻ�,700��ԼΪ1min)*/
#define SD_MAX_segment 35   /*���Ĵ�����(һ�����������1-250����)*/

#define SD_LONG_Wait   2000000L /*�ȴ�SD���ĳ���ʱʱ��*/
#define SD_SHORT_Wait  5000L    /*�ȴ�SD���Ķ���ʱʱ��*/
#define SD_VST_Wait    4000L    /*�ȴ�SD���ĳ�����ʱʱ��*/

//SD���������Ͷ�Ӧ��      /************By Sword************/
//(���߷���ʱ�ɹ۲����flag_err_sd��ֵ,Ȼ������ֵ��ñ�Ϳ�֪��SD������ľ������)
#define SD_Normal            0   //SD����ʼ������
#define SDERR_CMD0           1   //����CMD0ָ��ʱ����
#define SDERR_ACMD41         2   //����ACMD41ָ��ʱ����
#define SDERR_ReadOverTime   3   //��ȡ��ʱ
#define SDERR_StartSector    4   //������������
#define SDERR_NoCluster      5   //û���ҵ��㹻�Ĵ�
   
    
/*IO��������*/
#define  ON_SD_CS           GPIO_ResetBits(SPI_SD_CS_PORT, SPI_SD_CS_PIN)   /*����CS(ע��0Ϊѡͨ)*/
#define  OFF_SD_CS          GPIO_SetBits(SPI_SD_CS_PORT, SPI_SD_CS_PIN)     /*�ر�CS(ע��0Ϊѡͨ)*/

     
/**************************ȫ�ֱ�������**************************/                     
extern byte_sd sd_yes;          /* �ж��Ƿ���SD��(û��Ϊ0����Ϊ1) */
extern byte_sd flag_sdhc;       /* sdhc����־,=1��Ч */
extern byte_sd flag_test_sd;    /* ����ʹ�� */  
extern byte_sd flag_err_sd;     /* SD�����־ */
extern byte_sd Flag_SDRdy;      /* SD��������־(=1��SD������) */
extern byte_sd sd_initover;     /* SD����ʼ��������־ */

#ifdef EN_SDbuf
#ifdef EN_Cnt_SDbuf
extern word_sd Cnt_SDbufS1,Cnt_SDbufS2,Cnt_SDbufS3,Cnt_SDbufS4,Cnt_SDbufS5;
#endif
#endif



/**************************API��������**************************/
/*API����*/
extern void writebyte_ram(byte_sd wdata); /*д�ֽڵ�ram��*/
extern void write_stop(void);/*ֹͣд*/
extern void SD_System_Init(void); /************By Sword************/
extern void SD_System_Init_Ext(byte_sd * pbuf , word_sd len);
extern void test_sd(void);        /************By Sword************/

/*Fat32����*/
extern void writeram_sd(void); /*ram����512ʱдSD��*/
extern void search_addr_sd(void); /************By Sword************/
extern void search_fat(void);     /************By Sword************/
extern void fat32_init(void);     /************By Sword************/

#endif

#endif 
