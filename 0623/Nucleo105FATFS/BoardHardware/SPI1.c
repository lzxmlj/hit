#include "SPI1.h"
#include "sd_driver.h"
/*
SPI1�ײ�����
*/
void SPI1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;
	static int SPI1_InitFlag=0;
	
        GPIO_InitTypeDef  gpio_initstruct;
	if(SPI1_InitFlag == 0)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
               gpio_initstruct.GPIO_Pin = SPI_SD_SCK_PIN;
               gpio_initstruct.GPIO_Speed = GPIO_Speed_50MHz;
               gpio_initstruct.GPIO_Mode = GPIO_Mode_AF_PP;
               GPIO_Init(SPI_SD_SCK_PORT, &gpio_initstruct);
               
               /* ����MOSI����(ע���Ǹ����������ģʽ) */
               gpio_initstruct.GPIO_Pin = SPI_SD_MISO_PIN;
               gpio_initstruct.GPIO_Speed = GPIO_Speed_50MHz;
               gpio_initstruct.GPIO_Mode = GPIO_Mode_AF_PP;
               GPIO_Init(SPI_SD_MISO_PORT, &gpio_initstruct);
               
               /* ����MISO����(ע���Ǹ����������ģʽ) */
               gpio_initstruct.GPIO_Pin = SPI_SD_MOSI_PIN;
               gpio_initstruct.GPIO_Speed = GPIO_Speed_50MHz;
               gpio_initstruct.GPIO_Mode = GPIO_Mode_AF_PP;
               GPIO_Init(SPI_SD_MOSI_PORT, &gpio_initstruct);
               
               /* ����CS����(ע�����������ģʽ,�����Ϊ�����������ģʽ,���ߵ����ܹ�,�����л������) */
               gpio_initstruct.GPIO_Pin = SPI_SD_CS_PIN;
               gpio_initstruct.GPIO_Speed = GPIO_Speed_10MHz;
               gpio_initstruct.GPIO_Mode = GPIO_Mode_Out_PP;
               GPIO_Init(SPI_SD_CS_PORT, &gpio_initstruct);
	
		SPI_I2S_DeInit(SPI1);
		SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//ȫ˫��
		SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;//8λ����ģʽ
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;//����ģʽ��SCKΪ1
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;//���ݲ����ӵ�2��ʱ����ؿ�ʼ
		SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;//NSS�������
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;//������
		SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;//���ģʽ
		SPI_InitStructure.SPI_CRCPolynomial = 7;//CRC����ʽ
		SPI_InitStructure.SPI_Mode = SPI_Mode_Master;//����ģʽ
		SPI_Init(SPI1, &SPI_InitStructure);
		SPI_Cmd(SPI1, ENABLE);
		SPI1_InitFlag=1;
	}
}

void SPI1_SetSpeed(u8 SpeedSet)
{
	SPI1->CR1&=0XFFC7;
	SPI1->CR1|=SpeedSet;
	SPI_Cmd(SPI1,ENABLE);
}
u8 SPI1_ReadWriteByte(u8 TxData)
{
	u8 retry=0;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) //���ָ����SPI��־λ�������:���ͻ���ձ�־λ
	{
		retry++;
		if(retry>200)return 0;
	}
	SPI_I2S_SendData(SPI1, TxData); //ͨ������SPIx����һ������
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) //���ָ����SPI��־λ�������:���ܻ���ǿձ�־λ
	{
		retry++;
		if(retry>200)return 0;
	}
	return SPI_I2S_ReceiveData(SPI1); //����ͨ��SPIx������յ�����
}


void CSPin_Init(void)
{
	SPI1_CS1Pin_Init();
	SPI1_CS2Pin_Init();
	SPI1_CS3Pin_Init();
}

__weak void SPI1_CS1Pin_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
	
         GPIO_InitStructure.GPIO_Pin = SPI_SD_CS_PIN;
         GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
         GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
         GPIO_Init(SPI_SD_CS_PORT, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB, GPIO_Pin_1);//��ѡ��
}

__weak void SPI1_CS2Pin_Init(void)
{
}

__weak void SPI1_CS3Pin_Init(void)
{
}
