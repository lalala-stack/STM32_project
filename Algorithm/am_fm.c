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
extern float32_t fft_input[FFT_BUF_SIZE*2];  // �������飺ʵ��+�鲿
extern float32_t fft_output[FFT_BUF_SIZE];  
extern uint8_t adc1_measurementReady;
extern uint8_t adc3_measurementReady;

#define AVERAGE_NUM 20

double value[AVERAGE_NUM] = {0};
uint16_t value_index = 0;
double average = 0;

// Ĭ�����ò���
static SignalDetectConfig current_config = {
    .fm_max_var = 100.0f,
    .am_min_var = 800.0f,
    .noise_threshold = 5.0f,
    .min_samples = 64
};

static void calculate_signal_stats_dsp(const uint16_t* buffer, uint16_t len, float32_t* mean, float32_t* variance);
static bool confirm_with_zero_crossing_dsp(const uint16_t* buffer, uint16_t len);

// ��ʼ������
void signal_detect_init(SignalDetectConfig* config) {
    if(config) {
        current_config = *config;
    }
}

// ����ź�����
SignalType detect_signal_type(const uint16_t* adc_buffer, uint16_t buffer_len) {
    // 1. ����źŴ�����
    if(!is_signal_present(adc_buffer, buffer_len)) {
        return SIGNAL_NONE;
    }
    
    // 2. ����ͳ������
    float32_t mean, variance;
    calculate_signal_stats_dsp(adc_buffer, buffer_len, &mean, &variance);
    
    // 3. �����߼�
    if(variance < current_config.fm_max_var) {
        // FM������֤
        if(confirm_with_zero_crossing_dsp(adc_buffer, buffer_len)) {
            return SIGNAL_FM;
        }
    } else if(variance > current_config.am_min_var) {
        return SIGNAL_AM;
    }
    
    return SIGNAL_UNKNOWN;
}

// DSP�Ż����ź�ͳ�Ƽ���
static void calculate_signal_stats_dsp(const uint16_t* buffer, uint16_t len, 
                                     float32_t* mean, float32_t* variance) {
    // ת��Ϊfloat32_t���� (DSP��Ҫ��)
    float32_t f32_buf[len];
    for(uint16_t i = 0; i < len; i++) {
        f32_buf[i] = (float32_t)buffer[i];
    }
    
    // ʹ��DSP������ֵ
    arm_mean_f32(f32_buf, len, mean);
    
    // ʹ��DSP����㷽��
    arm_var_f32(f32_buf, len, variance);
}

// DSP�Ż��Ĺ�����
static bool confirm_with_zero_crossing_dsp(const uint16_t* buffer, uint16_t len) {
    // �����м�ֵ����
    float32_t mid_val = 2048.0f; // 12-bit ADC�м�ֵ
    uint16_t crossings = 0;

    // �ֶ���������
    for(uint16_t i = 1; i < len; i++) {
        float32_t prev = (float32_t)buffer[i-1];
        float32_t current = (float32_t)buffer[i];
        if(prev < mid_val && current >= mid_val) {
            crossings++;
        }
    }

    // ����: FMӦ�м��ٹ����
    return (crossings < (len / 100.0f));
}

// �źŴ��ڼ�� (DSP�Ż�)
bool is_signal_present(const uint16_t* adc_buffer, uint16_t buffer_len) {
    if(buffer_len < current_config.min_samples) 
        return false;
    
    // ʹ��DSP���Ҽ�ֵ
    float32_t min_val, max_val;
    
    // ת��Ϊfloat32
    float32_t f32_buf[buffer_len];
    for(uint16_t i = 0; i < buffer_len; i++) {
        f32_buf[i] = (float32_t)adc_buffer[i];
    }
    
    // �����ֵ
    uint32_t max_index;
    arm_max_f32(f32_buf, buffer_len, &max_val, &max_index);
    
    // ����Сֵ
    uint32_t min_index;
    arm_min_f32(f32_buf, buffer_len, &min_val, &min_index);
    
    // �жϷ��ֵ
    return ((max_val - min_val) > current_config.noise_threshold);
}

//�źź���������
//signal���ź�����
void Process_Signal(SignalType signal){
	switch(signal){
		case SIGNAL_AM:{
			//�л�������������
			Switch0to0;
			
			char* si = "SignalType:AM";
			lcd_fill(400 ,10, 800 ,50 ,BLACK);
			lcd_show_string(400 ,10 ,400 ,40 ,32 ,si ,GREEN);
			
			//�����ز�
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
				double k = 2.564;//����ϵ��
				double b = -0.023;//����ϵ��
				MA = k*average + b;
				//��ʾ�������
				lcd_show_string( 600 , 50 ,100 , 50 ,32 , "MA:", LIGHTGREEN );
				lcd_show_double( 650 , 50 , MA , 1 ,3 ,32 , LIGHTGREEN);
				Draw_Spectrum(fft_output);
				adc3_measurementReady = 0;
			}
			
			//ʧ�����ط�
//			AD9959_Set_Amplitude_Phase_Frequence(1, 900 ,0 ,frequency); 
//			HAL_Delay( 500 );
//			double carrier = GetADC_Max( &hadc2 , 10 , 1024 , 3.3 );
//			
//			AD9959_Set_Amplitude_Phase_Frequence(1, 900 ,0 ,frequency + AM_modulation_frequency*kHz); //������Ƶ
//			HAL_Delay( 500 );
//			double modulation_1 = GetADC_Max( &hadc2 , 10 ,1024 , 3.3 );
//			
//			
//			AD9959_Set_Amplitude_Phase_Frequence(1, 900 ,0 ,frequency - AM_modulation_frequency*kHz); //������Ƶ
//			HAL_Delay( 500 );
//			double modulation_2 = GetADC_Max( &hadc2 , 10 ,1024 , 3.3 );
//			
//			double MA = 0 ;
//			if( carrier > 1e-6 ){
//				double k = 0.5;//����ϵ��
//				MA = k * ( modulation_1 + modulation_2 ) / (carrier+1e-6);//����MA
//			}
//			
//			//���ݼ�����ز�Ƶ�ʽ��л�Ƶ
//			AD9959_Write_Frequence( 2 , frequency + 100*kHz );
//			HAL_Delay(100);
			
//			//Ƶ�׻���
//			Process_FFT(last_adc1_buf);	
//			Draw_Spectrum(fft_output);
			
			break;
		}
		case SIGNAL_FM:{
			//�л�����Ƶ������
			Switch0to1;
			
			uint8_t trigger = 0 ;
			uint32_t carrier = 0;
			float fm_amp = 0;
			uint32_t fm_freq = 0;
			
			float sweep_freq = 20.2 * MHz;
			
			char* si = "SignalType:FM";
			lcd_fill(400 ,10, 800 ,50 ,BLACK);
			lcd_show_string(400 ,10 ,400 ,40 ,32 ,si ,YELLOW);
			
			//ɨƵ������10~30MHz��
			float FM_trigger =  1.5f;     // ������ƽ
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

					lcd_show_num(100,10,signal_carrier_frequency,12,32,WHITE);//��ʾ�������ز�Ƶ��
		
		
				}
			}
			
			//���ݼ�����ز�Ƶ�ʽ��л�Ƶ
			AD9959_Write_Frequence( 1 , carrier + 100*kHz  );
			HAL_Delay(100);
			
			//�������Ĳ���
			float modulation_fre = 0;
			float modulation_amp = 0;
			
			
			//�ж��Ƿ�ΪCW��
			float fre_trigger = 10.0;//��ֵ����Ҫͨ��ʵ�ʲ�������
			
			if(modulation_fre > fre_trigger){
				//�ж�ΪFM��
				
				//ͨ��Ƶ�ʺͷ������mf
				double mf = 0;
				double k = 1;//ϵ������
				mf = k * modulation_amp / (modulation_fre+1e-6);
				//��ʾ��Ƶָ��
				lcd_show_string( 600 , 50 ,100 , 50 ,32 , "mf:", LIGHTGREEN );
				lcd_show_double( 650 , 50 , mf , 3 ,3 ,32 , LIGHTGREEN);
			}
			else{
				//CW����ʾ
				char* si = "SignalType:CW";
				lcd_fill(400 ,10, 800 ,50 ,BLACK);	
				lcd_show_string(400 ,10 ,400 ,40 ,32 ,si ,BLUE);
			}
			
			//Ƶ�׻���
			Process_FFT(last_adc1_buf, 0, 0);	
			Draw_Spectrum(fft_output);
			
			break;
		}
		case SIGNAL_UNKNOWN:{
			char* si = "SIGNAL_UNKNOWN";
			lcd_fill(400 ,10, 800 ,50 ,BLACK);	
			lcd_show_string(400 ,10 ,400 ,40 ,32 ,si ,BLUE);
			//���Ƶ��
			lcd_fill(ORIGIN_X, ORIGIN_Y - Y_AXIS_LEN, ORIGIN_X + X_AXIS_LEN, ORIGIN_Y, BACKGROUND);
			break;
			
		}
		case SIGNAL_NONE:{
			char* si = "SIGNAL_NONE";
			lcd_fill(400 ,10, 800 ,50 ,BLACK);
			lcd_show_string(400 ,10 ,400 ,40 ,32 ,si ,WHITE);
			//���Ƶ��
			lcd_fill(ORIGIN_X, ORIGIN_Y - Y_AXIS_LEN, ORIGIN_X + X_AXIS_LEN, ORIGIN_Y, BACKGROUND);
			break;
		}
		default:
			break;
	}
}

/**
 * @brief �������Ҳ�ADC���������Ƶ��
 * 
 * @param adc_array �������Ҳ�ADC����ֵ������
 * @param length ���鳤��
 * @param sampling_rate �����ʣ�Hz��
 * @return double ����õ������Ҳ�Ƶ�ʣ�Hz��������޷������򷵻�0.0
 */
double CalculateSineWaveFrequency(const uint16_t *adc_array, size_t length, double sampling_rate) {
    if (length < 3 || sampling_rate <= 0.0) {
        return 0.0; // ��֤���������Ч��
    }
    
    // ����ֱ��ƫ������ƽ��ֵ��
    double dc_offset = 0.0;
    for (size_t i = 0; i < length; i++) {
        dc_offset += adc_array[i];
    }
    dc_offset /= length;
    
    // ���㽻���źŹ������
    int zero_cross_count = 0;
    int prev_sign = 0; // 0��ʾ��ʼ״̬��1��ʾ��ֵ��-1��ʾ��ֵ
	int time_span_0 = 0;
	int time_span_1 = 0;
    
    for (size_t i = 0; i < length; i++) {
        double sample = (double)adc_array[i] - dc_offset;
        int cur_sign = (sample > 0) ? 1 : ((sample < 0) ? -1 : 0);
        if(time_span_0 > 0) time_span_1++;
        if (i == 0) {
            prev_sign = cur_sign; // ��ʼ����һ����ķ���
            continue;
        }

        // ������㣺���ű仯��������㸽��������
        if (prev_sign != 0 && cur_sign != 0 && prev_sign != cur_sign) {
			if(time_span_0 == 0) time_span_0 = i;
            zero_cross_count++;
        }
        prev_sign = (cur_sign != 0) ? cur_sign : prev_sign; // ������һ����Ч����
    }
    
    // ����Ƶ�ʣ��������/2 = ����������
    double time_span = (time_span_1 - time_span_0)/sampling_rate;
    return (zero_cross_count / 2.0) / time_span;
}


/**
 * @brief ʹ�ù������������Ҳ��źŵ�Ƶ�ʺͷ���
 * 
 * @param adc_buffer    ���������ָ��ADC�������ݵ�ָ��
 *                      ���ã�����ԭʼADC����ֵ������
 * 
 * @param buffer_size   ���������ADC�������Ĵ�С
 *                      ���ã�ָ����Ҫ�����Ĳ���������
 * 
 * @param sampling_rate ��������������ʣ���λ��Hz��
 *                      ���ã����ڽ�������ת��Ϊʱ��ֵ
 * 
 * @param frequency     �������������õ����ź�Ƶ�ʣ���λ��Hz��
 *                      ���ã��洢����������ź�Ƶ��
 * 
 * @param amplitude     �������������õ����źŷ��ȣ���λ�����أ�
 *                      ���ã��洢��������źŷ�ֵ����
 */
// ʹ�ù����������ź�Ƶ�ʺͷ���
void calculate_signal_params_zc(const uint16_t* adc_buffer, 
                               uint32_t buffer_size,
                               float sampling_rate,
                               float* frequency,
                               float* amplitude) {
    // 1. �����źŷ��ȣ�ʱ�򷽷���
    uint16_t max_val = 0;
    uint16_t min_val = 0xFFFF;
    
    for (uint32_t i = 0; i < buffer_size; i++) {
        if (adc_buffer[i] > max_val) max_val = adc_buffer[i];
        if (adc_buffer[i] < min_val) min_val = adc_buffer[i];
    }
    
    // ������ֵ���� (Vpp)
    const float v_ref = 3.3f; // ����ο���ѹ3.3V
    float vpp = (max_val - min_val) * (v_ref / 4095.0f);
    *amplitude = vpp / 2.0f; // ��ֵ���� = Vpp / 2
    
    // 2. ����ֱ��ƫ�ƣ����ڹ�����⣩
    float dc_offset = 0;
    for (uint32_t i = 0; i < buffer_size; i++) {
        dc_offset += adc_buffer[i];
    }
    dc_offset /= buffer_size;
    
    // 3. �������
    uint32_t zero_crossings = 0;
    float prev_sample = (float)adc_buffer[0] - dc_offset;
    
    // �洢�����λ�ã��������Բ�ֵ��
    uint32_t last_crossing_index = 0;
    float total_period_samples = 0.0f;
    
    for (uint32_t i = 1; i < buffer_size; i++) {
        float current_sample = (float)adc_buffer[i] - dc_offset;
        
        // ������㣨���������򸺵�����
        if ((prev_sample >= 0 && current_sample < 0) || 
            (prev_sample < 0 && current_sample >= 0)) {
            
            // ʹ�����Բ�ֵ��߾���
            float fraction = fabsf(prev_sample) / (fabsf(prev_sample) + fabsf(current_sample));
            float crossing_position = (i - 1) + fraction;
            
            // �������ڳ��ȣ�����ǵڶ��μ��Ժ�Ĺ���㣩
            if (zero_crossings > 0) {
                float period_samples = crossing_position - last_crossing_index;
                total_period_samples += period_samples;
            }
            
            last_crossing_index = crossing_position;
            zero_crossings++;
        }
        
        prev_sample = current_sample;
    }
    
    // 4. ����Ƶ��
    if (zero_crossings >= 2) {
        // ����ƽ�����ڳ��ȣ���������
        float avg_period_samples = total_period_samples / (zero_crossings - 1);
        // ת��ΪƵ�� (Hz)
        *frequency = sampling_rate / avg_period_samples;
    } else {
        // û���㹻�Ĺ����
        *frequency = 0.0f;
    }
}
