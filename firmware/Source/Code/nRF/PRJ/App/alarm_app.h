#ifndef ERROR_APPLICATION_LAYER_H
#define	ERROR_APPLICATION_LAYER_H

	#include <stdint.h>
	#include <stdbool.h>
    #include "beep_protocol.h"

    typedef struct
    {
        DS18B20_RESULTS_s		ds18B20;
        BME280_RESULT_s			bme280;
        ADC_s					saadc;
        HX711_CONV_s			hx711;
        FFT_RESULTS             fft;
        uint16_t                activeErrors;
        uint16_t                transmitErrors;
    }APP_SAMPLE_ERROR_s;

    uint16_t        getAlarmsActive     (void);
    uint16_t        getAlarmsTransmit   (bool clear);
    void            readAlarmParam      (BEEP_protocol_s * reply);
    void            checkSample         (MEASUREMENT_RESULT_s * meas);
    void            testAlarm           (void);

#endif	/* ERROR_APPLICATION_LAYER_H */

