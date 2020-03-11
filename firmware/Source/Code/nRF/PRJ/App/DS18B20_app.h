#ifndef DS18B20_APPLICATION_LAYER_H
#define	DS18B20_APPLICATION_LAYER_H

	#include <stdint.h>
	#include <stdbool.h>
	#include "beep_types.h"
	#include "DS18B20.h"

	#define DS18B20_ERROR_COUNT_MAX	5
    #define DS18B20_SENSORS_MAX		10
	#define DS18B20_DISABLE_SENSOR_ON_ERROR_MAX	0
    #define DS18B20_LOG_ENABLE      0

	typedef enum
	{
		DS_STOP,
		DS_DETECT,
		DS_SEARCH,
		DS_READ_CONFIG,
		DS_SET_CONFIG,
		DS_IDLE,
		DS_START_TEMP_CONV,
		DS_WAIT_TEMP_CONV_COMPLETE,
		DS_GET_TEMP_CONV_RESULT,
        DS_SHUTDOWN,
		DS_UNKNOWN_STATE,
	}DS18B20_STATES_s;

    /* Structure for the DS18B20 sensor */
    typedef struct
    {		
		bool			enabled;
        float			temp;  
        DS_RESOLUTION	resolution;
        uint8_t			Thigh;
		uint8_t			Tlow;
        uint8_t			errorCount;
		uint8_t			ROM[DS18B20_ROM_LENGHT];
    }sDS18B20;

	typedef struct
	{
		bool					on;
		bool					settingsChanged;
		DS18B20_STATES_s		state;
		uint8_t					device_count;		// The number of DS18b20's found.
		uint8_t					read_index;
        DS_RESOLUTION			resolution;
		uint32_t				conversionCount;
		uint8_t					convIndex;
		uint32_t				interval;
        sDS18B20				dev[DS18B20_SENSORS_MAX];
        measurement_callback	handler;
        CONTROL_SOURCE			source;
		uint32_t				startTimestamp;
	}DS18B20_APP_s;

	uint32_t	DS18B20_App_setStatus			(uint8_t status, uint16_t interval);
	uint32_t	DS18B20_App_getStatus			(uint8_t * ret_status, uint16_t * interval);
	void		DS18B20_App_getTemp				(MEASUREMENT_RESULT_s * result);
	bool		DS18B20_App_busy				(void);
    void		DS18B20_App_init				(uint16_t interval, DS_RESOLUTION resolution, measurement_callback measurement_handler);
    void		DS18B20_App_while				(void);
    uint32_t	DS18B20_App_start_conversion	(CONTROL_SOURCE source, uint8_t index);
    uint32_t    DS18B20_App_shutdown            (void);

#endif	/* DS18B20_APPLICATION_LAYER_H */

