 
/**********************************************************
                       康威科技
更多电子需求，请到淘宝店，康威电子竭诚为您服务 ^_^
https://kvdz.taobao.com/
**********************************************************/
 
#ifndef _adf4002_h_
#define _adf4002_h_
 
#include "main.h"

#define R_Address 0X000000
#define N_Address 0X000001
#define F_Address 0X000
#define I_Address 0X000003
#define Pre_R 0X000000          //X000,0000,DAT(14),00
#define Pre_N 0X000000          //XX0,DAT(13),XXXXXX,01
 
//管脚定义
#define PLL_SCK_GPIO_Port GPIOG
#define PLL_SCK_Pin GPIO_PIN_2
#define PLL_SDI_GPIO_Port GPIOG
#define PLL_SDI_Pin GPIO_PIN_4
#define PLL_SEN_GPIO_Port GPIOG
#define PLL_SEN_Pin GPIO_PIN_6

#define PLL_SCK_0 (HAL_GPIO_WritePin(PLL_SCK_GPIO_Port, PLL_SCK_Pin, GPIO_PIN_RESET))
#define PLL_SCK_1 (HAL_GPIO_WritePin(PLL_SCK_GPIO_Port, PLL_SCK_Pin, GPIO_PIN_SET))
#define PLL_SDI_0 (HAL_GPIO_WritePin(PLL_SDI_GPIO_Port, PLL_SDI_Pin, GPIO_PIN_RESET))
#define PLL_SDI_1 (HAL_GPIO_WritePin(PLL_SDI_GPIO_Port, PLL_SDI_Pin, GPIO_PIN_SET))
#define PLL_SEN_0 (HAL_GPIO_WritePin(PLL_SEN_GPIO_Port, PLL_SEN_Pin, GPIO_PIN_RESET))
#define PLL_SEN_1 (HAL_GPIO_WritePin(PLL_SEN_GPIO_Port, PLL_SEN_Pin, GPIO_PIN_SET))
 
void InitADF4002(void);
void Delay(unsigned int z);
void DelayMs(uint32_t ms);
void SendDataPll(unsigned long int Data);
 
void RDivideTest(uint16_t RData);
void NDivideTest(uint16_t NData);
 
 
#endif

