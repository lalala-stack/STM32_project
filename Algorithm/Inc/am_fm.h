#ifndef AM_FM_H
#define AM_FM_H

#include <stdint.h>
#include <stdbool.h>
#include "arm_math.h"  // CMSIS-DSPͷ�ļ�

// �ź�����ö��
typedef enum {
    SIGNAL_AM,      // AM�����ź�
    SIGNAL_FM,      // FM��Ƶ�ź�
    SIGNAL_UNKNOWN, // �޷�ʶ����ź�
    SIGNAL_NONE     // ���ź�/����
} SignalType;

// �źż�����ýṹ��
typedef struct {
    float fm_max_var;    // FM��󷽲���ֵ
    float am_min_var;    // AM��С������ֵ
    float noise_threshold; // ��������
    uint16_t min_samples; // ��С��������
} SignalDetectConfig;

// ��ʼ���������
void signal_detect_init(SignalDetectConfig* config);

// ���ļ�⺯��
SignalType detect_signal_type(const uint16_t* adc_buffer, uint16_t buffer_len);

// �źŴ��ڼ��
bool is_signal_present(const uint16_t* adc_buffer, uint16_t buffer_len);


//�źź�������
void Process_Signal(SignalType signal);

//���Ҳ�Ƶ�ʼ���
double CalculateSineWaveFrequency(const uint16_t *adc_array, size_t length, double sampling_rate);
double CalculateSineWaveFrequency_fft(const uint16_t *adc_array, size_t length, double sampling_rate);

//����Ƶ�ʺͷ��ȣ���������FM����ź�
void calculate_signal_params_zc(const uint16_t* adc_buffer, uint32_t buffer_size,float sampling_rate, float* frequency,float* amplitude) ;

#endif 
