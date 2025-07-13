#include "AD_DA.h"

/**
 * @brief 解码非归零码（支持NRZ和NRZI编码）
 * @param samples[]: 采样数据数组（ADC量化值，范围0-4095对应电压0-3.3V）
 * @param decoded_data: 存储解码结果
 * @param sample_rate: 采样率 (Hz)，用于计算比特边界
 * @param length: 采样数组长度
 * @param threshold: 电平判定阈值（通常为Vcc/2，双极性时需考虑正负阈值）
 * @param is_nrzi: 编码类型选择（0=NRZ, 1=NRZI）
 * @retval 解码后的二进制数据长度
 */
uint16_t decode_nrz(uint8_t samples[], uint8_t* decoded_data, uint32_t sample_rate, uint16_t length, float threshold, uint16_t bit_duration, uint8_t is_nrzi){
    uint16_t bit_count = 0;            // 解码位计数
    uint8_t last_level = samples[0] > threshold ? 1 : 0;  // 初始电平
	uint32_t samples_per_bit = (sample_rate * bit_duration) / 1000000;

    for (uint16_t i = 1; i < length; i += samples_per_bit){
		uint16_t high_count = 0;
		uint16_t current_level = 0;
		// 检测周期内高电平次数
        for(uint16_t j = i; j < i + samples_per_bit; j++){
            if(samples[j] > threshold){
                high_count++;
            }
        }
		// 判断占空比是否大于0.5（去抖）
		if(high_count/samples_per_bit > 0.5){
			current_level = 1;
		}
		else{
			current_level = 0;
		}
        
        // NRZI解码：电平跳变表示"1"，不变表示"0"
        if (is_nrzi){
            if (current_level != last_level){
                decoded_data[bit_count++] = 1;  // 跳变代表1
            }
			else{
                decoded_data[bit_count++] = 0;  // 不变代表0
            }
        } 
        // NRZ解码：直接判断电平
        else{
            decoded_data[bit_count++] = current_level;
        }
        last_level = current_level;
    }
    return bit_count;  // 返回解码后的数据位数
}

/**
 * @brief 解码归零码（支持单极性/双极性）
 * @param samples[]: 采样数据数组（ADC量化值）
 * @param decoded_data: 存储解码结果
 * @param sample_rate: 采样率 (Hz)
 * @param length: 采样数组长度
 * @param threshold_vol: 电平判定阈值（通常为Vcc/2即2048）
 * @param threshold_duty_ratio: 占空比判定阈值，超过认定为1，未超过为0（通常为0.5）
 * @param bit_duration: 单个比特的持续时间（微秒）
 * @retval 解码后的二进制数据长度
 */
uint16_t decode_rz(uint16_t samples[], uint8_t* decoded_data, uint32_t sample_rate, uint16_t length, uint16_t threshold_vol, float threshold_duty_ratio, uint16_t bit_duration){
    uint16_t bit_count = 0;
    uint32_t samples_per_bit = (sample_rate * bit_duration) / 1000000;

    for(uint16_t i = 0; i < length; i += samples_per_bit){
        uint16_t high_count = 0;
        // 检测周期内高电平次数
        for(uint16_t j = i; j < i + samples_per_bit; j++){
            if(samples[j] > threshold_vol){
                high_count++;
            }
        }
		// 判断占空比是否大于阈值
		if((float)high_count/samples_per_bit > threshold_duty_ratio){
			decoded_data[bit_count] = 1;
		}
		else{
			decoded_data[bit_count] = 0;
		}
		bit_count++;
    }
    return bit_count;
}

/**
 * @brief 在解码数据中查找指定包头
 * @param decoded_data: 解码后的二进制数据数组(0/1值)
 * @param length: 解码数据数组长度
 * @param preamble: 目标包头序列数组
 * @param preamble_bits: 包头序列的比特长度
 * @param min_matches: 最小匹配比特数(0表示需要完全匹配)
 * @retval 匹配位置的起始索引(-1表示未找到)
 */
uint16_t detect_preamble(const uint8_t* decoded_data, uint16_t length,const uint8_t* preamble, uint16_t preamble_bits,uint16_t min_matches){
    // 参数检查
    if (!decoded_data || !preamble || length == 0 || preamble_bits == 0) 
        return PREAMBLE_NOT_FOUND;
    
    // 设置最小匹配阈值（完全匹配时设为preamble_bits）
    if (min_matches == 0 || min_matches > preamble_bits) 
        min_matches = preamble_bits;

    // 可配置的容错机制（允许的错误比特数）
    const uint16_t max_errors = preamble_bits - min_matches;
    
    // 遍历所有可能的起始位置
    for (uint16_t i = 0; i <= length - preamble_bits; i++){
        uint16_t match_count = 0;
        uint16_t errors = 0;
        
        // 检查当前起始位置是否匹配
        for (uint16_t j = 0; j < preamble_bits; j++){
            if (decoded_data[i + j] == preamble[j]){
                match_count++;
            } else {
                errors++;
                // 提前终止：当错误超过最大允许值
                if (errors > max_errors) break;
            }
        }
        
        // 检查是否达到最小匹配要求
        if (match_count >= min_matches) {
            return i;  // 返回匹配的起始位置
        }
    }
    
    return PREAMBLE_NOT_FOUND;  // 未找到符合条件的包头
}