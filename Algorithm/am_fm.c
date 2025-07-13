#include "am_fm.h"
#include "arm_math.h"
#include "lcd.h"
#include "adc.h"
#include "ad9959.h"
#include "switch.h"
#include "stdio.h"

extern uint32_t signal_carrier_frequency;
extern double AM_modulation_frequency;
extern uint16_t last_adc1_buf[ADC_BUF_SIZE];
extern uint16_t last_adc3_buf[ADC_BUF_SIZE];
extern float32_t fft_input[FFT_BUF_SIZE*2];  // 复数数组：实部+虚部
extern float32_t fft_output[FFT_BUF_SIZE];  
extern uint8_t adc1_measurementReady;
extern uint8_t adc3_measurementReady;

#define AVERAGE_NUM 20

double value[AVERAGE_NUM] = {0};
uint16_t value_index = 0;
double average = 0;

// 默认配置参数
static SignalDetectConfig current_config = {
    .fm_max_var = 100.0f,
    .am_min_var = 800.0f,
    .noise_threshold = 5.0f,
    .min_samples = 64
};

static void calculate_signal_stats_dsp(const uint16_t* buffer, uint16_t len, float32_t* mean, float32_t* variance);
static bool confirm_with_zero_crossing_dsp(const uint16_t* buffer, uint16_t len);

// 初始化配置
void signal_detect_init(SignalDetectConfig* config) {
    if(config) {
        current_config = *config;
    }
}

// 检测信号类型
SignalType detect_signal_type(const uint16_t* adc_buffer, uint16_t buffer_len) {
    // 1. 检查信号存在性
    if(!is_signal_present(adc_buffer, buffer_len)) {
        return SIGNAL_NONE;
    }
    
    // 2. 计算统计特征
    float32_t mean, variance;
    calculate_signal_stats_dsp(adc_buffer, buffer_len, &mean, &variance);
    
    // 3. 决策逻辑
    if(variance < current_config.fm_max_var) {
        // FM二次验证
        if(confirm_with_zero_crossing_dsp(adc_buffer, buffer_len)) {
            return SIGNAL_FM;
        }
    } else if(variance > current_config.am_min_var) {
        return SIGNAL_AM;
    }
    
    return SIGNAL_UNKNOWN;
}

// DSP优化的信号统计计算
static void calculate_signal_stats_dsp(const uint16_t* buffer, uint16_t len, 
                                     float32_t* mean, float32_t* variance) {
    // 转换为float32_t类型 (DSP库要求)
    float32_t f32_buf[len];
    for(uint16_t i = 0; i < len; i++) {
        f32_buf[i] = (float32_t)buffer[i];
    }
    
    // 使用DSP库计算均值
    arm_mean_f32(f32_buf, len, mean);
    
    // 使用DSP库计算方差
    arm_var_f32(f32_buf, len, variance);
}

// DSP优化的过零检测
static bool confirm_with_zero_crossing_dsp(const uint16_t* buffer, uint16_t len) {
    // 创建中间值向量
    float32_t mid_val = 2048.0f; // 12-bit ADC中间值
    uint16_t crossings = 0;

    // 手动计算过零点
    for(uint16_t i = 1; i < len; i++) {
        float32_t prev = (float32_t)buffer[i-1];
        float32_t current = (float32_t)buffer[i];
        if(prev < mid_val && current >= mid_val) {
            crossings++;
        }
    }

    // 决策: FM应有极少过零点
    return (crossings < (len / 100.0f));
}

// 信号存在检测 (DSP优化)
bool is_signal_present(const uint16_t* adc_buffer, uint16_t buffer_len) {
    if(buffer_len < current_config.min_samples) 
        return false;
    
    // 使用DSP库找极值
    float32_t min_val, max_val;
    
    // 转换为float32
    float32_t f32_buf[buffer_len];
    for(uint16_t i = 0; i < buffer_len; i++) {
        f32_buf[i] = (float32_t)adc_buffer[i];
    }
    
    // 找最大值
    uint32_t max_index;
    arm_max_f32(f32_buf, buffer_len, &max_val, &max_index);
    
    // 找最小值
    uint32_t min_index;
    arm_min_f32(f32_buf, buffer_len, &min_val, &min_index);
    
    // 判断峰峰值
    return ((max_val - min_val) > current_config.noise_threshold);
}

//信号后续处理函数
//signal：信号类型
void Process_Signal(SignalType signal){
	switch(signal){
		case SIGNAL_AM:{
			//切换至调幅波测量
			Switch0to0;
			
			char* si = "SignalType:AM";
			lcd_fill(400 ,10, 800 ,50 ,BLACK);
			lcd_show_string(400 ,10 ,400 ,40 ,32 ,si ,GREEN);
			
			//测量载波
			uint32_t frequency = 0;
			frequency = signal_carrier_frequency;
			lcd_show_num( 100 , 50 ,frequency , 8 , 32 , BLUE ); 
			
			if(adc3_measurementReady){
				double MA = 0 ;
				Process_FFT(last_adc3_buf, &AM_modulation_frequency, &MA);
				value[(value_index++)%AVERAGE_NUM] = MA;
				lcd_show_double( 300 , 50 , AM_modulation_frequency + 0.234, 5 , 3 ,32 ,RED);
				average = 0;
				for(int i = 0; i < AVERAGE_NUM; i++)
				{
					average = average + value[i];
				}
				average = average / AVERAGE_NUM;
				double k = 2.564;//调整系数
				double b = -0.023;//调整系数
				MA = k*average + b;
				//显示调幅深度
				lcd_show_string( 600 , 50 ,100 , 50 ,32 , "MA:", LIGHTGREEN );
				lcd_show_double( 650 , 50 , MA , 1 ,3 ,32 , LIGHTGREEN);
				Draw_Spectrum(fft_output);
				adc3_measurementReady = 0;
			}
			
			//失落的相关法
//			AD9959_Set_Amplitude_Phase_Frequence(1, 900 ,0 ,frequency); 
//			HAL_Delay( 500 );
//			double carrier = GetADC_Max( &hadc2 , 10 , 1024 , 3.3 );
//			
//			AD9959_Set_Amplitude_Phase_Frequence(1, 900 ,0 ,frequency + AM_modulation_frequency*kHz); //测量边频
//			HAL_Delay( 500 );
//			double modulation_1 = GetADC_Max( &hadc2 , 10 ,1024 , 3.3 );
//			
//			
//			AD9959_Set_Amplitude_Phase_Frequence(1, 900 ,0 ,frequency - AM_modulation_frequency*kHz); //测量边频
//			HAL_Delay( 500 );
//			double modulation_2 = GetADC_Max( &hadc2 , 10 ,1024 , 3.3 );
//			
//			double MA = 0 ;
//			if( carrier > 1e-6 ){
//				double k = 0.5;//调整系数
//				MA = k * ( modulation_1 + modulation_2 ) / (carrier+1e-6);//计算MA
//			}
//			
//			//根据计算的载波频率进行混频
//			AD9959_Write_Frequence( 2 , frequency + 100*kHz );
//			HAL_Delay(100);
			
//			//频谱绘制
//			Process_FFT(last_adc1_buf);	
//			Draw_Spectrum(fft_output);
			
			break;
		}
		case SIGNAL_FM:{
			//切换至调频波测量
			Switch0to1;
			
			uint8_t trigger = 0 ;
			uint32_t carrier = 0;
			float fm_amp = 0;
			uint32_t fm_freq = 0;
			
			float sweep_freq = 20.2 * MHz;
			
			char* si = "SignalType:FM";
			lcd_fill(400 ,10, 800 ,50 ,BLACK);
			lcd_show_string(400 ,10 ,400 ,40 ,32 ,si ,YELLOW);
			
			//扫频（测量10~30MHz）
			float FM_trigger =  1.5f;     // 触发电平
			while( sweep_freq <= 40.7*MHz){
				sweep_freq += 500*kHz;
				
				AD9959_Write_Frequence( 0 ,sweep_freq );
				HAL_Delay(100);
				
				float temp_amp = GetADC_Average ( &hadc2 , ADC_CHANNEL_11 , 100 ,3.3 );
				lcd_show_double (100 ,50 ,temp_amp ,1 ,3 ,32 ,WHITE  );
				
				if( temp_amp > fm_amp){
					fm_amp = temp_amp ; 
					fm_freq = sweep_freq - 10.7*MHz;
				}
				
			}
			
			if( fm_amp > FM_trigger ){
				carrier = fm_freq;
				AD9959_Write_Frequence( 0 , carrier + 10.7*MHz );
				HAL_Delay(3000);
				
				lcd_show_num (300 ,50 ,carrier ,8 ,32,WHITE  );
				if(adc1_measurementReady) {
					adc1_measurementReady = 0;

					lcd_show_num(100,10,signal_carrier_frequency,12,32,WHITE);//显示调幅波载波频率
		
		
				}
			}
			
			//根据计算的载波频率进行混频
			AD9959_Write_Frequence( 1 , carrier + 100*kHz  );
			HAL_Delay(100);
			
			//检测解调后的波形
			float modulation_fre = 0;
			float modulation_amp = 0;
			
			
			//判断是否为CW波
			float fre_trigger = 10.0;//阈值，需要通过实际测量更改
			
			if(modulation_fre > fre_trigger){
				//判断为FM波
				
				//通过频率和幅度拟合mf
				double mf = 0;
				double k = 1;//系数待测
				mf = k * modulation_amp / (modulation_fre+1e-6);
				//显示调频指数
				lcd_show_string( 600 , 50 ,100 , 50 ,32 , "mf:", LIGHTGREEN );
				lcd_show_double( 650 , 50 , mf , 3 ,3 ,32 , LIGHTGREEN);
			}
			else{
				//CW波显示
				char* si = "SignalType:CW";
				lcd_fill(400 ,10, 800 ,50 ,BLACK);	
				lcd_show_string(400 ,10 ,400 ,40 ,32 ,si ,BLUE);
			}
			
			//频谱绘制
			Process_FFT(last_adc1_buf, 0, 0);	
			Draw_Spectrum(fft_output);
			
			break;
		}
		case SIGNAL_UNKNOWN:{
			char* si = "SIGNAL_UNKNOWN";
			lcd_fill(400 ,10, 800 ,50 ,BLACK);	
			lcd_show_string(400 ,10 ,400 ,40 ,32 ,si ,BLUE);
			//清空频谱
			lcd_fill(ORIGIN_X, ORIGIN_Y - Y_AXIS_LEN, ORIGIN_X + X_AXIS_LEN, ORIGIN_Y, BACKGROUND);
			break;
			
		}
		case SIGNAL_NONE:{
			char* si = "SIGNAL_NONE";
			lcd_fill(400 ,10, 800 ,50 ,BLACK);
			lcd_show_string(400 ,10 ,400 ,40 ,32 ,si ,WHITE);
			//清空频谱
			lcd_fill(ORIGIN_X, ORIGIN_Y - Y_AXIS_LEN, ORIGIN_X + X_AXIS_LEN, ORIGIN_Y, BACKGROUND);
			break;
		}
		default:
			break;
	}
}

/**
 * @brief 计算正弦波ADC采样数组的频率
 * 
 * @param adc_array 包含正弦波ADC采样值的数组
 * @param length 数组长度
 * @param sampling_rate 采样率（Hz）
 * @return double 计算得到的正弦波频率（Hz），如果无法计算则返回0.0
 */
double CalculateSineWaveFrequency(const uint16_t *adc_array, size_t length, double sampling_rate) {
    if (length < 3 || sampling_rate <= 0.0) {
        return 0.0; // 验证输入参数有效性
    }
    
    // 计算直流偏移量（平均值）
    double dc_offset = 0.0;
    for (size_t i = 0; i < length; i++) {
        dc_offset += adc_array[i];
    }
    dc_offset /= length;
    
    // 计算交流信号过零次数
    int zero_cross_count = 0;
    int prev_sign = 0; // 0表示初始状态，1表示正值，-1表示负值
	int time_span_0 = 0;
	int time_span_1 = 0;
    
    for (size_t i = 0; i < length; i++) {
        double sample = (double)adc_array[i] - dc_offset;
        int cur_sign = (sample > 0) ? 1 : ((sample < 0) ? -1 : 0);
        if(time_span_0 > 0) time_span_1++;
        if (i == 0) {
            prev_sign = cur_sign; // 初始化第一个点的符号
            continue;
        }

        // 检测过零点：符号变化且跳过零点附近的噪声
        if (prev_sign != 0 && cur_sign != 0 && prev_sign != cur_sign) {
			if(time_span_0 == 0) time_span_0 = i;
            zero_cross_count++;
        }
        prev_sign = (cur_sign != 0) ? cur_sign : prev_sign; // 保持上一个有效符号
    }
    
    // 计算频率：过零次数/2 = 完整周期数
    double time_span = (time_span_1 - time_span_0)/sampling_rate;
    return (zero_cross_count / 2.0) / time_span;
}


/**
 * @brief 使用过零点检测计算正弦波信号的频率和幅度
 * 
 * @param adc_buffer    输入参数：指向ADC采样数据的指针
 *                      作用：包含原始ADC采样值的数组
 * 
 * @param buffer_size   输入参数：ADC缓冲区的大小
 *                      作用：指定需要分析的采样点数量
 * 
 * @param sampling_rate 输入参数：采样率（单位：Hz）
 *                      作用：用于将样本数转换为时间值
 * 
 * @param frequency     输出参数：计算得到的信号频率（单位：Hz）
 *                      作用：存储计算出的主信号频率
 * 
 * @param amplitude     输出参数：计算得到的信号幅度（单位：伏特）
 *                      作用：存储计算出的信号峰值幅度
 */
// 使用过零点检测计算信号频率和幅度
void calculate_signal_params_zc(const uint16_t* adc_buffer, 
                               uint32_t buffer_size,
                               float sampling_rate,
                               float* frequency,
                               float* amplitude) {
    // 1. 计算信号幅度（时域方法）
    uint16_t max_val = 0;
    uint16_t min_val = 0xFFFF;
    
    for (uint32_t i = 0; i < buffer_size; i++) {
        if (adc_buffer[i] > max_val) max_val = adc_buffer[i];
        if (adc_buffer[i] < min_val) min_val = adc_buffer[i];
    }
    
    // 计算峰峰值幅度 (Vpp)
    const float v_ref = 3.3f; // 假设参考电压3.3V
    float vpp = (max_val - min_val) * (v_ref / 4095.0f);
    *amplitude = vpp / 2.0f; // 峰值幅度 = Vpp / 2
    
    // 2. 计算直流偏移（用于过零点检测）
    float dc_offset = 0;
    for (uint32_t i = 0; i < buffer_size; i++) {
        dc_offset += adc_buffer[i];
    }
    dc_offset /= buffer_size;
    
    // 3. 过零点检测
    uint32_t zero_crossings = 0;
    float prev_sample = (float)adc_buffer[0] - dc_offset;
    
    // 存储过零点位置（用于线性插值）
    uint32_t last_crossing_index = 0;
    float total_period_samples = 0.0f;
    
    for (uint32_t i = 1; i < buffer_size; i++) {
        float current_sample = (float)adc_buffer[i] - dc_offset;
        
        // 检测过零点（从正到负或负到正）
        if ((prev_sample >= 0 && current_sample < 0) || 
            (prev_sample < 0 && current_sample >= 0)) {
            
            // 使用线性插值提高精度
            float fraction = fabsf(prev_sample) / (fabsf(prev_sample) + fabsf(current_sample));
            float crossing_position = (i - 1) + fraction;
            
            // 计算周期长度（如果是第二次及以后的过零点）
            if (zero_crossings > 0) {
                float period_samples = crossing_position - last_crossing_index;
                total_period_samples += period_samples;
            }
            
            last_crossing_index = crossing_position;
            zero_crossings++;
        }
        
        prev_sample = current_sample;
    }
    
    // 4. 计算频率
    if (zero_crossings >= 2) {
        // 计算平均周期长度（样本数）
        float avg_period_samples = total_period_samples / (zero_crossings - 1);
        // 转换为频率 (Hz)
        *frequency = sampling_rate / avg_period_samples;
    } else {
        // 没有足够的过零点
        *frequency = 0.0f;
    }
}
