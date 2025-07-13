/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "adf4002.h"
#include "lcd.h"
#include "arm_const_structs.h"
#include "ad9959.h"
#include "am_fm.h"
#include "AD_DA.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 
volatile uint32_t pulseCount = 0;
volatile uint32_t signal_carrier_frequency = 0;
volatile uint8_t adc1_measurementReady = 0;
volatile uint8_t adc3_measurementReady = 0;
volatile double adc2_average_value = 0;
double AM_modulation_frequency = 0;
uint16_t adc1_buf[ADC_BUF_SIZE] = {0};
uint16_t last_adc1_buf[ADC_BUF_SIZE] = {0};
uint16_t last_adc3_buf[ADC_BUF_SIZE] = {0};
uint8_t dma_current_buf = 0;

float32_t fft_input[FFT_BUF_SIZE*2];  // 复数数组：实部+虚部
float32_t fft_output[FFT_BUF_SIZE];   // 幅度输出

//绘图相关
uint16_t prev_spectrum_x[FFT_BUF_SIZE/2] = {0}; // 存储上一次频谱点X坐标
uint16_t prev_spectrum_y[FFT_BUF_SIZE/2] = {0}; // 存储上一次频谱点Y坐标
uint16_t prev_num_points = 0;                   // 上一次绘制的点数
uint16_t adc3_buffer[ADC_BUF_SIZE] = {0}; 

uint16_t tim4_fre = 0; //定时器4频率
uint16_t tim4_nvic = 0;

uint8_t decoded_data[256] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

int fputc(int ch,FILE *f);
void Draw_Axes(void);
//void Draw_Spectrum(float32_t *draw_buf);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_TIM8_Init();
  MX_DAC_Init();
  MX_TIM12_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  lcd_init();		//LCD初始化
  Init_AD9959();    //锁相环初始化
  
  SignalDetectConfig my_config = {
        .fm_max_var = 300.0f,
        .am_min_var = 750.0f,
        .noise_threshold = 50.0f,
        .min_samples = 128
  };
  signal_detect_init(&my_config);
  
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start(&htim8);
  HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_buf, ADC_BUF_SIZE);
  // 初始化后启动ADC
  HAL_ADC_Start_DMA(&hadc3, 
                  (uint32_t*)adc3_buffer,   // 目标地址
                  ADC_BUF_SIZE // 总数据量=通道数×深度
                 );
//  
  
  lcd_display_dir(1);
  Draw_Axes();
  
  uint32_t test_wave[ADC_BUF_SIZE] = {0};
  for(int i = 0; i<512; i++)
  {
	  test_wave[i] = 2048;
  }
  
  // 初始化DAC
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1); // 启动通道1
  // 设置输出电n压为1.65V（VREF=3.3V）
  uint32_t dacValue = (1.65f / 3.3f) * 4095;
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dacValue);
  
//  AD9959_Set_Amplitude_Phase_Frequence(1, 900 ,0 ,15*MHz);
  
  HAL_Delay(10);
  IO_Update();
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(adc1_measurementReady)
	  {
		  if(decode_rz(last_adc1_buf, decoded_data, 1*MHz, ADC_BUF_SIZE, 2000, 0.5, 8)){
			  for(int i = 0; i < ADC_BUF_SIZE/8; i++)
			  {
				  printf("%d ", decoded_data[i]);
			  }
			  printf("\n");
			  uint8_t preamble_test[8] = {0};
			  uint16_t start = detect_preamble(decoded_data, ADC_BUF_SIZE/8, preamble_test, 8, 0);
			  printf("%d\n", start);
		  }
		  adc1_measurementReady = 0;
	  }
	
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//重定向printf
int fputc(int ch,FILE *f)
{
  uint8_t temp[1]={ch};
  HAL_UART_Transmit(&huart1,temp,1,2);
  return ch;
}

/* FFT处理函数 */
void Process_FFT(uint16_t *adc_buf, double *max_fre, double *max_fre_vol)
{
    // 1. 转换ADC数据到复数数组
	for(int i = 0; i < FFT_BUF_SIZE; i++)
	{
		// 实部：ADC值归一化到0-3.3V（假设12位ADC）
		fft_input[2*i] = (float32_t)adc_buf[i] * 3.3f / 4095.0f; 
		fft_input[2*i+1] = 0.0f;  // 虚部置零
	}

    // 2. 执行FFT变换
    arm_cfft_f32(&arm_cfft_sR_f32_len1024, fft_input, 0, 1);
    
    // 3. 计算幅度谱
    arm_cmplx_mag_f32(fft_input, fft_output, FFT_BUF_SIZE);
    
    // 4. 幅度归一化
    fft_output[0] = fft_output[0] * 3 / FFT_BUF_SIZE;  	// 直流分量
	float32_t max_value = 0;
	float32_t max_freq_index = 0;
    for(int i = 4; i < FFT_BUF_SIZE; i++) {
        fft_output[i] = fft_output[i] * 3 / (FFT_BUF_SIZE / 2.0f);    // 交流分量
		if(i < FFT_BUF_SIZE/2)
		{
			if(fft_output[i] > max_value)
			{
				max_value = fft_output[i];
				max_freq_index = i;
			}
		}
    }
	*max_fre = max_freq_index * MAX_FREQ / FFT_BUF_SIZE;
	*max_fre_vol = max_value;
}

// 绘制坐标轴函数
void Draw_Axes(void)
{
    // 清屏（背景色）
    lcd_clear(BACKGROUND);
    
    // 1. 绘制坐标轴
    lcd_draw_line(ORIGIN_X-1, ORIGIN_Y+1, ORIGIN_X + X_AXIS_LEN+8, ORIGIN_Y+1, AXIS_COLOR); // X轴
    lcd_draw_line(ORIGIN_X-1, ORIGIN_Y+1, ORIGIN_X-1, ORIGIN_Y - Y_AXIS_LEN-8, AXIS_COLOR); // Y轴
    
    // 2. 绘制X轴箭头
    lcd_draw_line(ORIGIN_X + X_AXIS_LEN+8, ORIGIN_Y+1, ORIGIN_X + X_AXIS_LEN+8 - 6, ORIGIN_Y+1 - 3, AXIS_COLOR);
    lcd_draw_line(ORIGIN_X + X_AXIS_LEN+8, ORIGIN_Y+1, ORIGIN_X + X_AXIS_LEN+8 - 6, ORIGIN_Y+1 + 3, AXIS_COLOR);
    // 绘制Y轴箭头
    lcd_draw_line(ORIGIN_X-1, ORIGIN_Y- Y_AXIS_LEN-8, ORIGIN_X-1 - 3, ORIGIN_Y - Y_AXIS_LEN-8 + 6, AXIS_COLOR);
    lcd_draw_line(ORIGIN_X-1, ORIGIN_Y - Y_AXIS_LEN-8, ORIGIN_X-1 + 3, ORIGIN_Y - Y_AXIS_LEN-8 + 6, AXIS_COLOR);
    
    // 3. 绘制网格和刻度
    // X轴网格和刻度 (100kHz间隔)
    for (uint16_t x = ORIGIN_X + 140; x < ORIGIN_X + X_AXIS_LEN; x += 140) {
//        lcd_draw_vline(x, ORIGIN_Y - Y_AXIS_LEN, Y_AXIS_LEN, GRID_COLOR); // 垂直网格线
        lcd_draw_vline(x, ORIGIN_Y+1, 5, AXIS_COLOR); // 刻度线
    }
    
    // Y轴网格和刻度 (0.5V间隔)
    for (uint16_t y = ORIGIN_Y - 50; y > ORIGIN_Y - Y_AXIS_LEN; y -= 50) {
//        lcd_draw_hline(ORIGIN_X, y, X_AXIS_LEN, GRID_COLOR); // 水平网格线
        lcd_draw_hline(ORIGIN_X-1 - 5, y, 5, AXIS_COLOR); // 刻度线
    }
    
    // 4. 显示坐标轴标签
    // X轴刻度值 (0, 100k, 200k, 300k, 400k, 500k)
    const uint16_t x_pos[] = {ORIGIN_X, ORIGIN_X+X_AXIS_LEN/5, ORIGIN_X+X_AXIS_LEN/5*2, ORIGIN_X+X_AXIS_LEN/5*3, ORIGIN_X+X_AXIS_LEN/5*4, ORIGIN_X+X_AXIS_LEN};
    const double freq_val[] = {0, MAX_FREQ/2/5, MAX_FREQ/2/5*2, MAX_FREQ/2/5*3, MAX_FREQ/2/5*4, MAX_FREQ/2};
    
    for (uint8_t i = 0; i < 6; i++) {
        lcd_show_num(x_pos[i] - 15, ORIGIN_Y+1 + 10, freq_val[i], 3, 16, TEXT_COLOR);
    }
    lcd_show_string(ORIGIN_X + X_AXIS_LEN+2 - 30, ORIGIN_Y+1 + 35, 20, 20, 16, "kHz", TEXT_COLOR);
    
    // Y轴刻度值 (0.0, 0.5, ..., 3.0)
    uint8_t val = 0;
    for (uint16_t y = ORIGIN_Y+1; y >= ORIGIN_Y+1 - 300; y -= 50) {
        lcd_show_double(ORIGIN_X - 40, y - 8, val*0.5, 1, 1, 16, TEXT_COLOR);
        val++;
    }
    lcd_show_string(ORIGIN_X-1 - 60, ORIGIN_Y - Y_AXIS_LEN-2, 20, 20, 16, "V", TEXT_COLOR);
}

// 动态绘制频谱图函数
void Draw_Spectrum(float32_t *draw_buf)
{
    // 动态绘制频谱线
    uint16_t prev_x = ORIGIN_X;
    uint16_t prev_y = ORIGIN_Y - (uint16_t)((draw_buf[0] * Y_AXIS_LEN) / MAX_AMPLITUDE);
	if (prev_y > ORIGIN_Y) prev_y = ORIGIN_Y;
    if (prev_y < ORIGIN_Y - Y_AXIS_LEN) prev_y = ORIGIN_Y - Y_AXIS_LEN;
    
    for (uint16_t i = 1; i < FFT_BUF_SIZE/2; i++) {  // 只取前512个点(单边频谱)
        // 计算当前点坐标
        uint16_t x = ORIGIN_X + (i * X_AXIS_LEN) / (FFT_BUF_SIZE/2);
        uint16_t y = ORIGIN_Y - (uint16_t)((draw_buf[i] * Y_AXIS_LEN) / MAX_AMPLITUDE);
        
        // 边界保护
        if (y > ORIGIN_Y) y = ORIGIN_Y;
        if (y < ORIGIN_Y - Y_AXIS_LEN) y = ORIGIN_Y - Y_AXIS_LEN;
        
		// 清除已绘制谱线
		lcd_fill(prev_x, ORIGIN_Y-Y_AXIS_LEN, x, ORIGIN_Y, BACKGROUND);
        // 绘制当前频谱线片段
        lcd_draw_line(prev_x, prev_y, x, y, SPECTRUM_COLOR);
        
        // 更新前一个点坐标
        prev_x = x;
        prev_y = y;

    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
