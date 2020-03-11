#ifndef _MAIN_H__
#define _MAIN_H__

	#include <stdint.h>
	#include <stdbool.h>
	#include "beep_protocol.h"

	#define DISABLE_SAMPLING_SENSORS	0

	typedef enum
	{
		/* Initial state in which the microcontroller gathers information about the battery and orientation. From this state the microcontroller will go to
		 * either Horizontal or Vertical. All sensors are in low power mode and are disabled.
		 * If battery=ok and orientation=horizontal: HORIZONTAL_START
		 * If battery=Nok or orientation=vertical: VERTICAL_START
		 */
		POWER_ON,			

		/*
		 * When the sensor is Horizontal this state starts the horizontal mode and turns sensors on when available.
		 */
		HORIZONTAL_START,

        /*
		 * While the sensor is Horizontal the sensors will be periodically sampled and the data transmitted with LoRaWAN.
		 * Will go to HORIZONTAL_STOP if the orientation changes to vertical.
		 * Will go to HORIZONTAL_STOP if the battery voltage = Nok
		 */
		HORIZONTAL_ACTIVE,

		/*
		 * State which waits for the application to finish it's task before entering the DFU ready state.
		 */
		HORIZONTAL_DFU_START,

		/*
		 * In this state the application is ready to enter DFU
		 */
		HORIZONTAL_DFU_READY,

        /*
		 * Powers the sensors down if needed, then continues to VERTICAL_START.
		 */
		HORIZONTAL_STOP,

        /*
		 * Wait state for when the sensor is in a vertical state but the battery votlage is too low for the snesor to operate. 
		 */
		HORIZONTAL_IDLE,

		/*
		 * Powers the sensors down if needed and preperes for power down, then continues to VERTICAL_POWER_DOWN.
		 */
		VERTICAL_START,

        /*
		 * The MCU will now powerdown and can only wake-up with a reset, orientation change or reed switch
		 */
		VERTICAL_POWER_DOWN,

		ERROR_STATE,
		
	}MAIN_STATE_e;

	typedef enum
	{
		SAMPLE_IDLE,
		SAMPLE_START,
		SAMPLE_WAIT,
        SAMPLE_AUDIO,
        SAMPLE_STORE,
		SAMPLE_SEND,
	}SAMPLE_STATE;

	typedef struct
	{
		MAIN_STATE_e	current_state;
		uint32_t		timestamp_current_state;
		uint32_t		sampleInterval_min;
		SAMPLE_STATE	sample_state;	
		uint32_t		sample_start_timestamp;
		uint8_t			lorawan_ratio_threshold;
		uint8_t			lorawan_ratio_counter;
		bool			new_Timer_config;
        bool            joinIndicationSend;
        bool            dataIndicationSend;
	}MAIN_APP_s;

    void sendResponse       (CONTROL_SOURCE source, BEEP_CID cmd, uint32_t error_code);
    void sendProtocolField  (BEEP_protocol_s * prot, CONTROL_SOURCE source);

#endif // _MAIN_H__