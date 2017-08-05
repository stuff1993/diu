#include "iirfilter.h"
#include <stdint.h>

/******************************************************************************
 ** Function:    iir_filter_uint
 **
 ** Description: Filter to flatten out erratic data reads
 **
 ** Parameters:  1. Input data
 **              2. Existing data
 **              3. Gain factor
 ** Return:      Smoothed value
 **
 ******************************************************************************/
uint32_t iir_filter_uint(uint32_t _data_in, uint32_t _cur_data, uint16_t _gain)
{
	return (((_gain - 1) * _cur_data) + _data_in) / _gain;
}

/******************************************************************************
 ** Function:    iir_filter_int
 **
 ** Description: Filter to flatten out erratic data reads
 **
 ** Parameters:  1. Input data
 **              2. Existing data
 **              3. Gain factor
 ** Return:      Smoothed value
 **
 ******************************************************************************/
int32_t iir_filter_int(int32_t _data_in, int32_t _cur_data, uint16_t _gain)
{
	return (((_gain - 1) * _cur_data) + _data_in) / _gain;
}

/******************************************************************************
 ** Function:    iir_filter_float
 **
 ** Description: Filter to flatten out erratic data reads
 **
 ** Parameters:  1. Input data
 **              2. Existing data
 **              3. Gain factor
 ** Return:      Smoothed value
 **
 ******************************************************************************/
float iir_filter_float(float _data_in, float _cur_data, uint16_t _gain)
{
	return (((_gain - 1) * _cur_data) + _data_in) / _gain;
}
