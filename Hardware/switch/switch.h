#include "main.h"

#define Switch0to0 HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET)
#define Switch0to1 HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET)
#define Switch1to0 HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_SET)
#define Switch1to1 HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_RESET)

//#define get_Switch0_state HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_2)
//#define get_Switch1_state HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_4)