#ifndef _SOFT_SPI_IO_H
#define _SOFT_SPI_IO_H
#include "main.h"



//GPIO pin definition
#define cs   PCout(6)																	//PC6
#define sclk PCout(7) 																//PC7
#define mosi PCout(8)																	//PC8
#define miso GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9)  //PC9


//SPI set
#define sclk_freq 500		//SPI_CLK 1000Hz
#define sclk_period_ms (int)(1/sclk_freq*1000) 		


void soft_spi_Init(void);//全部初始化
void soft_spi_io_Init(void);//引脚初始化		
uint32_t soft_spi_r_and_s(uint32_t send);	//receive and send
#endif
