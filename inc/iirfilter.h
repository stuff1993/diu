#ifndef IIRFILTER_H_
#define IIRFILTER_H_

#include <stdint.h>

uint32_t iir_filter_uint(uint32_t _data_in, uint32_t _cur_data, uint16_t _gain);
int32_t iir_filter_int(int32_t _data_in, int32_t _cur_data, uint16_t _gain);
float iir_filter_float(float _data_in, float _cur_data, uint16_t _gain);

#endif // IIRFILTER_H_
