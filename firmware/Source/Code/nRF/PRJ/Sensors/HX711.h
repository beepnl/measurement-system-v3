/*!
 * \file      HX711.h
 *
 * \brief     HX711 weighting scale sensor implementation
 *
 */
#ifndef __HX711_H__
#define __HX711_H__

	#include <stdbool.h>
	#include <stdint.h>
	#include "beep_protocol.h"


	#define CLK_CYCLES_CH_A_GAIN128		25	// 19
	#define CLK_CYCLES_CH_B_GAIN32		26	// 1A
	#define CLK_CYCLES_CH_A_GAIN64		27	// 1B


	typedef struct
	{
		HX711_callback	handler;
        HX711_GAIN		gain;
		bool			rate;	// 0 = 10SPS, 1 = 80 SPS.
		int32_t			last_result;
		uint32_t		sample_count;
	}HX711_s;


	void HX711_init		(HX711_callback callback, HX711_GAIN gain, bool highSampleRate);
	void HX711_setGain	(HX711_GAIN gain);
    void HX711_setRate	(bool highSampleRate);
    void HX711_enable	(bool enable);


#endif // __HX711_H__
