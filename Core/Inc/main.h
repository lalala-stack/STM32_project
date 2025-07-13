/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void Process_FFT(uint16_t *adc_buf, double *max_fre, double *max_fre_vol);
void Draw_Spectrum(float32_t *draw_buf);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

#define ADC_BUF_SIZE 1024

#define FFT_BUF_SIZE 1024

// ��ͼ��غ궨���ȫ�ֱ���
#define BACKGROUND  g_back_color  	// ��ɫ����
#define AXIS_COLOR  WHITE  			// ��ɫ������
#define GRID_COLOR  GRAY  			// ǳ��ɫ����
#define SPECTRUM_COLOR RED 			// ��ɫƵ����
#define TEXT_COLOR  g_point_color   // ��ɫ����

#define ORIGIN_X 80         // ����ԭ��X
#define ORIGIN_Y 420        // ����ԭ��Y
#define X_AXIS_LEN 700      // X�᳤��
#define Y_AXIS_LEN 330      // Y�᳤��
#define MAX_FREQ 1000		//500kHz (1MHz������/2)
#define MAX_AMPLITUDE 3.3   //3.3V

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
