#ifndef HX711_APPLICATION_LAYER_H
#define	HX711_APPLICATION_LAYER_H

	#include <stdint.h>
	#include <stdbool.h>
	#include "beep_types.h"
	#include "HX711.h"


	typedef enum
	{
		HX_INIT,
		HX_IDLE,
		HX_START_SAMPLING,
		HX_SAMPLING,
        HX_STOP_SAMPLING,
		HX_UNKNOWN,
	}HX711_STATES_s;

    /* Structure for the HX711 sensor application */
	typedef struct
	{
		bool					on;
        uint16_t				convCount_counter;
		uint16_t				convCount_treshold;
		int32_t					convSum;
		int32_t					average_result;
        HX711_CONV_s			results;
        HX711_GAIN				gain;
		HX711_STATES_s			state;
		measurement_callback	handler;
        CONTROL_SOURCE			source;
		uint8_t					channels;
		uint32_t				sampleCount;
	}HX711_APP_s;

    void		HX711_app_While				(void);
    bool		HX711_app_busy				(void);
	void		HX711_app_Init				(measurement_callback measurement_handler);
    uint32_t	HX711_app_start_sampling	(CONTROL_SOURCE source, uint16_t Nsamples, uint8_t channels);
    void		HX711_app_getLastResult		(MEASUREMENT_RESULT_s * result);
    uint32_t    HX711_app_shutdown          (void);

#endif	/* HX711_APPLICATION_LAYER_H */

