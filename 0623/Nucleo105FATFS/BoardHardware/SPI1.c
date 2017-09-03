#include "SPI1.h"
#include "sd_driver.h"
/*
SPI1底层驱动
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
               
               /* 配置MOSI引脚(注意是复用推挽输出模式) */
               gpio_initstruct.GPIO_Pin = SPI_SD_MISO_PIN;
               gpio_initstruct.GPIO_Speed = GPIO_Speed_50MHz;
               gpio_initstruct.GPIO_Mode = GPIO_Mode_AF_PP;
               GPIO_Init(SPI_SD_MISO_PORT, &gpio_initstruct);
               
               /* 配置MISO引脚(注意是复用推挽输出模式) */
               gpio_initstruct.GPIO_Pin = SPI_SD_MOSI_PIN;
               gpio_initstruct.GPIO_Speed = GPIO_Speed_50MHz;
               gpio_initstruct.GPIO_Mode = GPIO_Mode_AF_PP;
               GPIO_Init(SPI_SD_MOSI_PORT, &gpio_initstruct);
               
               /* 配置CS引脚(注意是推挽输出模式,如果设为复用推挽输出模式,在线调试能过,但运行会出问题) */
               gpio_initstruct.GPIO_Pin = SPI_SD_CS_PIN;
               gpio_initstruct.GPIO_Speed = GPIO_Speed_10MHz;
               gpio_initstruct.GPIO_Mode = GPIO_Mode_Out_PP;
               GPIO_Init(SPI_SD_CS_PORT, &gpio_initstruct);
	
		SPI_I2S_DeInit(SPI1);
		SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//全双工
		SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;//8位数据模式
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;//空闲模式下SCK为1
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;//数据采样从第2个时间边沿开始
		SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;//NSS软件管理
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;//波特率
		SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;//大端模式
		SPI_InitStructure.SPI_CRCPolynomial = 7;//CRC多项式
		SPI_InitStructure.SPI_Mode = SPI_Mode_Master;//主机模式
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
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) //检查指定的SPI标志位设置与否:发送缓存空标志位
	{
		retry++;
		if(retry>200)return 0;
	}
	SPI_I2S_SendData(SPI1, TxData); //通过外设SPIx发送一个数据
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) //检查指定的SPI标志位设置与否:接受缓存非空标志位
	{
		retry++;
		if(retry>200)return 0;
	}
	return SPI_I2S_ReceiveData(SPI1); //返回通过SPIx最近接收的数据
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
	GPIO_SetBits(GPIOB, GPIO_Pin_1);//不选中
}

__weak void SPI1_CS2Pin_Init(void)
{
}

__weak void SPI1_CS3Pin_Init(void)
{
}
