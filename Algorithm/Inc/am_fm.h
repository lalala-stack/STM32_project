#ifndef AM_FM_H
#define AM_FM_H

#include <stdint.h>
#include <stdbool.h>
#include "arm_math.h"  // CMSIS-DSP头文件

// 信号类型枚举
typedef enum {
    SIGNAL_AM,      // AM调幅信号
    SIGNAL_FM,      // FM调频信号
    SIGNAL_UNKNOWN, // 无法识别的信号
    SIGNAL_NONE     // 无信号/噪声
} SignalType;

// 信号检测配置结构体
typedef struct {
    float fm_max_var;    // FM最大方差阈值
    float am_min_var;    // AM最小方差阈值
    float noise_threshold; // 噪声门限
    uint16_t min_samples; // 最小采样点数
} SignalDetectConfig;

// 初始化检测配置
void signal_detect_init(SignalDetectConfig* config);

// 核心检测函数
SignalType detect_signal_type(const uint16_t* adc_buffer, uint16_t buffer_len);

// 信号存在检测
bool is_signal_present(const uint16_t* adc_buffer, uint16_t buffer_len);


//信号后续处理
void Process_Signal(SignalType signal);

//正弦波频率计算
double CalculateSineWaveFrequency(const uint16_t *adc_array, size_t length, double sampling_rate);
double CalculateSineWaveFrequency_fft(const uint16_t *adc_array, size_t length, double sampling_rate);

//计算频率和幅度，用来处理FM解调信号
void calculate_signal_params_zc(const uint16_t* adc_buffer, uint32_t buffer_size,float sampling_rate, float* frequency,float* amplitude) ;

#endif 
