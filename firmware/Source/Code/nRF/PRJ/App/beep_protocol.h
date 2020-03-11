/*!
 * \file      Beep protocol.h
 * \brief     BEEP protocol decoding and encoding
 *
 */
#ifndef __BEEP_PROTOCOL_H__
#define __BEEP_PROTOCOL_H__

	#include <stdbool.h>
	#include <stdint.h>
	#include "nrf_error.h"
	#include "beep_types.h"
    #include "app_util.h"
	
    #define BEEP_PROTOCOL_LOGGING   0
	#define BEEP_WRITE			(0x80)
	#define BEEP_READ			(0x00)
	#define BEEP_RW_bitmask		(0x7F)

    typedef enum
    {
        READ_STOP       = 0,    //  Stops reading the stored data when in progress
        READ_START      = 1,    //  Starts reading the stored data
        READ_STATUS     = 2,    //  Returns the current state
        ERASE_FS        = 3,    //  Erases the files on the filesystem
    }BUS_COMMAND;


    #define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.1 seconds). */
	#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.2 second). */
    #define MIN_NUS_CONN_INTERVAL           MSEC_TO_UNITS(10, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.1 seconds). */
	#define MAX_NUS_CONN_INTERVAL           MSEC_TO_UNITS(50, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.2 second). */
	#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
	#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */



	/*
		DS18B20		= 0,
			-	R/W		State of the sensor application/Write to restart searching for devices.
			-	R/W		Start conversion/Read last conversion result
			-	R		read specific sensors ID + config

		BME280		= 1,
			-	R/W		State of the sensor application/Write to set to new state
			-	R/W		Start conversion/Read last conversion result
			-	R/W		I2C read/write

		HX711		= 2,
        	-	R/W		State of the sensor application/Write to set to new state
			-	R/W		Start conversion for N sample on channel A or B/Read last conversion results from channel A or B

		AUDIO_ADC,	= 3,
			-	R/W		State of the sensor application/Write to set to new state
			-	R/W		Start conversion for X samples and N bins of F Hz/Read last conversion result with the configuration
			-	R/W		I2C read/write
		ATECC		= 4,
			-	R		Read ID
			-	R/W		I2C read/write
		BUZZER		= 5,
			-	R/W		State of the buzzer
			-	W		Play a predefined tune
			-	W		Palf on F Hz, Duty Cycle %, onTime ms, offtime ms, repeat N
		SQ_MIN		= 6,
			-	R/W		Current state: IO input and state machine state. Enable/Disable shutdown mode.	
		LORAWAN		= 7,
			-	R/W		Current mode of the LoRaWAN statemachine. Enable and disable the LoRaWAN functionality. Joined flag.
						uint8_t status
							R/W bit[1:0]	= 00 = Off, 01=ON, 11=reset
							R	bit[2]		= Joined
							R	bit[3]		= dutycycle limit
			-	R/W		DEVEUI. LoRaWAN moet uitgeschakeld zijn voor aanpassingen.
			-	R/W		APPEUI. LoRaWAN moet uitgeschakeld zijn voor aanpassingen.
			-	R/W		APPKEY. LoRaWAN moet uitgeschakeld zijn voor aanpassingen.
			-	W		Send LoRaWAN message with given payload.
							

        nRF_FLASH	= 8,
			-	R/W		Read or write a flash variable.
		MX_FLASH	= 9,
		nRF_ADC		= 10,
			-	R/W		Read measurement interval
			-	R/W		Start or read the measurement result Vbat mV, vcc mV, batterij percentage
		APPLICATIE	= 11,
			-	Read/Write State of the application. Reset Applicatie
			-	Reset/Set BLE pin code.
			-		
	*/
	

	
	
	typedef struct
	{
		BEEP_CID        command;
        CONTROL_SOURCE  source;
		
		// Union to generalize data access.
		union
		{
			RESPONSE_s				error;
            STATUS_s				status;
            FH_VERSION_s			version;
            DS18B20_STATE_s			ds_state;
            DS18B20_CONFIG_s		ds_config;
            AUDIO_CONFIG_s          audio_config;
            MEASUREMENT_RESULT_s	meas_result;	// Measurement storage container for all sensors.
            ATTECC_s				atecc_id;
            BUZZER_s				buzz;
            SQ_MIN_s				sq;
            LORAWAN_s				lorawan_key;
            nRF_FLASH_s				flash;
            uint32_t                size;
            ALARM_CONFIG_s          alarm;
            BME_CONFIG_s            bme_config;
		}param;
	}BEEP_protocol_s;

	
	typedef void	(*protocol_cb_p)			(CONTROL_SOURCE source, BEEP_protocol_s * command);


	const char *	beep_protocol_sensor_strget	(SENSOR_TYPE sensorType);
	uint32_t		beep_protocol_decode		(BEEP_protocol_s * prot, uint8_t * data, uint8_t msg_lenght, uint8_t * offset);
	uint32_t		beep_protocol_encode		(bool cmdPrepend, BEEP_protocol_s * prot, uint8_t * data, uint8_t * msg_lenght, uint8_t maxBufferLenght);
    const char *	HX711_channel_str			(HX711_GAIN channel);

#endif // __BEEP_PROTOCOL_H__
