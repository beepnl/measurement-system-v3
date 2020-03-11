/*!
 * \file      nRF_ADC.h
 *
 * \brief     ADC code.
 *
 */
#ifndef __NRF_ADC_H__
#define __NRF_ADC_H__

	#include <stdbool.h>
	#include <stdint.h>
	#include "beep_types.h"


    #define SAMPLES_IN_BUFFER			2
	#define SAADC_AVERAGING				10
    #define SAADC_LOG_ENABLE            0
	#define BATTERY_NOMINAL_mV			(3000.0)		// 2 * 1500mV
	#define BATTERY_CUTT_OFF_mV			(1600.0)		// 2 * 800mV

	bool	saadc_VbatOk				(void);
    bool	saadc_startUp				(void);
	void    get_saadc_result			(MEASUREMENT_RESULT_s * result);
    void	saadc_start_conversion		(CONTROL_SOURCE source);
	void	saadc_start					(uint32_t interval_sec, measurement_callback callback);

#endif // __NRF_ADC_H__
