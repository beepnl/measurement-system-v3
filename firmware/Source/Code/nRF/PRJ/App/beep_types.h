#ifndef BEEP_TYPES_H
#define	BEEP_TYPES_H

	#include <stdint.h>
	#include <stdbool.h>

	#define FIRMWARE_MAJOR		1
    #define FIRMWARE_MINOR		3
    #define FIRMWARE_SUB		2
	#define FIRMWARE_TO_UINT32_T(major, minor, sub) ((((uint32_t)major) << 16) | (((uint32_t)major) << 8) | (((uint32_t)major) << 0))

	#define PIN_CODE_BLE_LENGHT			6
	#define PIN_CODE_LENGHT_MIN			7
    #define PIN_CODE_LENGHT_MAX			16
	#define PIN_CODE_DEFAULT			"123456"

	#define HARDWARE_MAJOR				1
	#define HARDWARE_MINOR				0
	#define HARDWARE_ID					190222

	#define MAX_TEMP_SENSORS			10
	#define CID_WRITE					0x80
    #define CID_READ					0x00
    #define BEEP_MAX_LENGHT				30
    #define BEEP_LORAWAN_MAX_LENGHT		52
	#define BEEP_MIN_LENGHT				1
	#define ATECC_ID_LENGHT				9

	#define SAMPLE_INTERVAL_MIN_MAX		1440
    #define FFT_MAX_BINS                12

    #define HX711_N_CHANNELS            3


    /**@brief   TX Service event types. */
    typedef enum
    {
        APP_FLASH_READ,                 /**< Read Flash data. */
        APP_FLASH_ERASE,                /**< Read Flash data. */
        APP_TX_RDY_FLASH,               /**< Service is ready to accept new data to be transmitted. */
        APP_FLASH_COMM_STARTED,         /**< Notification has been enabled. */
        APP_FLASH_COMM_STOPPED,         /**< Notification has been disabled. */
        APP_FLASH_COMM_CONNECTED,
        APP_FLASH_COMM_DISCONNECTED,
        APP_FLASH_SIZE_FLASH,
    } app_flash_evt_type_t;

	typedef enum
	{
		RESPONSE					= 0,	//	uint8_t cmd, uint16_t errorCode
		READ_FIRMWARE_VERSION		= 1,	//	uint16_t	Major.Minor.sub
		READ_HARDWARE_VERSION		= 2,	//	uint16_t	Major.Minor
		READ_DS18B20_STATE			= 3,	//	State of the sensor application/Write to restart searching for devices. Number of devices, interval, resolution
		WRITE_DS18B20_STATE			= CID_WRITE | READ_DS18B20_STATE,
		READ_DS18B20_CONVERSION		= 4,	//	Start conversion/Read last conversion result
        WRITE_DS18B20_CONVERSION	= CID_WRITE | READ_DS18B20_CONVERSION, // 0x84
		READ_DS18B20_CONFIG			= 5,	//	read specific sensors ID + config
        BME280_CONFIG_READ          = 6,	//	0x06 State of the sensor, interval and other config
        BME280_CONFIG_WRITE         = CID_WRITE | BME280_CONFIG_READ, // 0x86
        BME280_CONVERSION_READ      = 7,	// 0x07
        BME280_CONVERSION_START     = CID_WRITE | BME280_CONVERSION_READ,	// 0x87
        READ_BME280_I2C				= 8,	//	I2C read/write
        READ_HX711_STATE			= 9,	//	Interval, channels
        WRITE_HX711_STATE			= CID_WRITE | READ_HX711_STATE,
        READ_HX711_CONVERSION		= 10,	//	Start conversion for N sample on channel A or B/Read last conversion results from channel A or B
        WRITE_HX711_CONVERSION		= CID_WRITE | READ_HX711_CONVERSION,
		READ_AUDIO_ADC_CONFIG		= 11,	//	R/W		State of the sensor application/Write to set to new state
        WRITE_AUDIO_ADC_CONFIG		= CID_WRITE | READ_AUDIO_ADC_CONFIG,	//	R/W		State of the sensor application/Write to set to new state
        READ_AUDIO_ADC_CONVERSION	= 12,	//	R/W		Start conversion for X samples and N bins of F Hz/Read last conversion result with the configuration
        START_AUDIO_ADC_CONVERSION	= 13,	//	R/W		I2C read/write
		READ_ATECC_READ_ID			= 14,	//	R		Read ID 9 bytes
        READ_ATECC_I2C				= 15,	//	R/W		I2C read/write
        READ_BUZZER_STATE			= 16,	//
		WRITE_BUZZER_DEFAULT_TUNE	= 17 | CID_WRITE,	//	0x11W		Play a predefined tune
		WRITE_BUZZER_CUSTOM_TUNE	= 18 | CID_WRITE,	//	W		Palf on F Hz, Duty Cycle %, onTime ms, offtime ms, repeat N
        READ_SQ_MIN_STATE			= 19,	//	R/W		Current state: IO input and state machine state. Enable/Disable shutdown mode.	
        READ_LORAWAN_STATE			= 20,	//	R/W		Current mode of the LoRaWAN statemachine. Enable and disable the LoRaWAN functionality. Joined flag.
        WRITE_LORAWAN_STATE			= CID_WRITE | READ_LORAWAN_STATE,
        READ_LORAWAN_DEVEUI			= 21,	//	R/W		DEVEUI. LoRaWAN moet uitgeschakeld zijn voor aanpassingen.
        WRITE_LORAWAN_DEVEUI		= CID_WRITE | READ_LORAWAN_DEVEUI,
        READ_LORAWAN_APPEUI			= 22,	//	R/W		APPEUI. LoRaWAN moet uitgeschakeld zijn voor aanpassingen.
        WRITE_LORAWAN_APPEUI		= CID_WRITE | READ_LORAWAN_APPEUI,
        READ_LORAWAN_APPKEY			= 23,	//	R/W		APPKEY. LoRaWAN moet uitgeschakeld zijn voor aanpassingen.
        WRITE_LORAWAN_APPKEY		= CID_WRITE | READ_LORAWAN_APPKEY,
        WRITE_LORAWAN_TRANSMIT		= CID_WRITE | 24,	//	W		Send LoRaWAN message with given payload.
        READ_CID_nRF_FLASH			= 25,	//	R/W		Read or write a flash parameter
        READ_nRF_ADC_CONFIG			= 26,	//	R/W		Read or write the conversion interval
        READ_nRF_ADC_CONVERSION		= 27,	//	0x1B R/W		Read the conversion result of the ADC.
        WRITE_nRF_ADC_CONVERSION	= CID_WRITE | READ_nRF_ADC_CONVERSION,
        READ_APPLICATION_STATE		= 28,	//	R/W
        READ_APPLICATION_CONFIG		= 29,	//	0x1D	R/W
        WRITE_APPLICATION_CONFIG	= CID_WRITE | READ_APPLICATION_CONFIG, // 0x9D
		READ_PINCODE				= 30,	// 0x1E
        WRITE_PINCODE				= CID_WRITE | READ_PINCODE, // 0x9E
        READ_BOOT_COUNT             = 31,   // 0x1F
        WRITE_BOOT_COUNT            = CID_WRITE | 31,   // 0x9F
        READ_MX_FLASH               = 32,   // 0x20
        ERASE_MX_FLASH              = 33,   // 0x21
        SIZE_MX_FLASH               = 34,   // 0x22
        ALARM_CONFIG_READ           = 35,   // 0x23
        ALARM_CONFIG_WRITE          = CID_WRITE | ALARM_CONFIG_READ, // 0xA3
        ALARM_STATUS_READ           = 36,   // 0x24
		CID_UNKNOWN,
	}BEEP_CID; // Command Identifier


    // Audio input selection
	typedef enum
	{
		AIN_IN3LM       = 0,
		AIN_IN2LP       = 1,
		AIN_IN2RP       = 2,
		AIN_OFF         = 3,
	}AUDIO_INPUTe;

    typedef enum
    {
        // Value 0 may not be used.
        BEEP_SENSOR_OFF			= 1,
        BEEP_SENSOR_ON			= 2,
        BEEP_KEEP_ALIVE			= 3,
        BEEP_ALARM				= 4,
        BEEP_BLE_CUSTOM			= 5,
        BEEP_DOWNLINK_RESPONSE	= 6,
        BEEP_UNKNOWN			= 0xFF,
    }BEEP_STATUS;

	typedef enum
	{
		BITMASK_ENABLE		= 0x01,
		BITMASK_JOINED		= 0x02,
		BITMASK_DUTYCYCLE	= 0x04,
		BITMASK_ADR			= 0x08,
		BITMASK_VALID_KEYS	= 0x10,
	}LORAWAN_CONFIG_BITMASK;

    typedef enum
	{
		SQ_SEN_645B_VERTICAL	= 0,
        SQ_SEN_645B_HORIZONTAL	= 1,
	}SQ_ORIENTATIONS_e;


    typedef enum
	{
		DS18B20		= 0,
		BME280		= 1,
		HX711		= 2,
		AUDIO_ADC	= 3,
        nRF_ADC		= 4,
        SQ_MIN		= 5,
		ATECC		= 6,
		BUZZER		= 7,
		LORAWAN		= 8,
		MX_FLASH	= 9,
        nRF_FLASH	= 10,
		APPLICATIE	= 11,
	}SENSOR_TYPE;

	typedef struct
	{
		SQ_ORIENTATIONS_e	orientation;
	}SQ_ORIENTATION_s;


    typedef struct
	{
		uint8_t		devices;
		int16_t     temperatures[MAX_TEMP_SENSORS];
	}DS18B20_RESULTS_s;

    typedef struct
    {
        uint8_t     bins;
        uint8_t     start;
        uint8_t     stop;
        uint16_t    values[FFT_MAX_BINS];
    }FFT_RESULTS;

	typedef struct
	{
		uint8_t		index;
		uint8_t		ROM[8];
	}DS18B20_CONFIG_s;

    typedef struct
	{
		uint8_t		channel;
        uint8_t     gain;
        int8_t      volume;
        uint8_t     fft_count;
        uint8_t     fft_start;
        uint8_t     fft_stop;    
        bool        min6dB;    
	}AUDIO_CONFIG_s;

    typedef struct
    {
    
    }BME_CONFIG_s;

	// Typedef for the number of clock 
	typedef enum
	{
		CH_A_GAIN128	= (1<<0),	// 0x01
		CH_B_GAIN32		= (1<<1),	// 0x02
		CH_A_GAIN64		= (1<<2),	// 0x03
		CH_HX_BITMASK	= (CH_A_GAIN128 | CH_B_GAIN32 | CH_A_GAIN64), // 0x07
		CH_HX_INVALID	= 0xFF,
	}HX711_GAIN;

	typedef struct
	{
		uint8_t		channel;
		uint16_t	samples;
		int32_t		value[HX711_N_CHANNELS];
	}HX711_CONV_s;

	typedef void (*HX711_callback)(HX711_CONV_s * result);

	typedef struct
	{
		uint8_t ROM[ATECC_ID_LENGHT];
	}ATTECC_s;

	typedef struct
	{
		uint16_t	humidity;
		int16_t		temperature;
		uint16_t	airPressure;
	}BME280_RESULT_s;

	typedef struct
	{
		uint8_t		index;
		uint8_t		freq_100Hz;
		uint16_t	on_time_ms;
		uint16_t	off_time_ms;
		uint16_t	repeatCount;	
	}BUZZER_s;

	typedef struct
	{
		uint8_t		currentValue;
		uint8_t		currentState;
		uint8_t		hysteresis;
	}SQ_MIN_s;

	typedef struct
	{
		uint8_t		lenght;
		uint8_t		data[BEEP_MAX_LENGHT];
	}LORAWAN_s;

	typedef struct
	{
		uint8_t		variable;
	}nRF_FLASH_s;

	typedef struct
	{
    	int16_t		battADC;
		uint16_t	battVoltage_mV;
		int16_t		vccADC;
		uint16_t	vccVoltage_mV;
		uint8_t		battPercentage;
	}ADC_s;

    // The communication interface origin.
	typedef enum
	{
		INTERNAL_SOURCE,
		BLE_SOURCE,
		LORAWAN_SOURCE,
		UNKNOWN_SOURCE,
	}CONTROL_SOURCE;


	typedef struct
	{
		SENSOR_TYPE		type;
        CONTROL_SOURCE	source;

		union
        {
			DS18B20_RESULTS_s		ds18B20;
            BME280_RESULT_s			bme280;
            ADC_s					saadc;	
            HX711_CONV_s			hx711;
            FFT_RESULTS             fft;
            SQ_ORIENTATION_s		sq;
		}result;
	}MEASUREMENT_RESULT_s;

	// Error response command and error code.
	typedef struct
	{
		BEEP_CID	ErrorCmd;
		uint32_t	errorCode;	
	}RESPONSE_s;

	typedef struct
	{
		uint8_t		statusflag;
		uint16_t	interval;	
	}STATUS_s;

	// Firmware and hardware version numbering. For hardware the sub version isn't used.
	typedef struct
	{
		uint16_t	major;
		uint16_t	minor;
		uint16_t	sub;
		uint32_t	id;
	}FH_VERSION_s;

	// DS18B20
	typedef struct
	{
		uint8_t		state;
	}DS18B20_STATE_s;

    typedef struct
    {
        int16_t     Max;
        int16_t     Min;
        uint16_t    Diff;
    }DS_ALARM_s;

    typedef struct
    {
        uint16_t    Min;
        uint16_t    Max;
        uint16_t    Diff;
    }SUPPLY_ALARM_s;

    typedef struct
    {
        int32_t     Max;
        int32_t     Min;
        uint32_t    Diff;    
    }HX711_ALARM_s;

    typedef struct
    {
        int16_t     Temp_Max;
        int16_t     Temp_Min;
        uint16_t    Temp_Diff;

        uint16_t	humidity_Max;
		uint16_t	humidity_Min;
		uint16_t	humidity_Diff;

		uint16_t	press_Max;
		uint16_t	press_Min;
		uint16_t	press_Diff;
    }BME_ALARM_s;

    typedef struct
    {
        SENSOR_TYPE  type;
        
        union
        {
            DS_ALARM_s      ds;
            SUPPLY_ALARM_s  supply;
            HX711_ALARM_s   hx;
            BME_ALARM_s     bme;
        }thr; // threshold
    }ALARM_CONFIG_s;

	typedef void (*measurement_callback) (MEASUREMENT_RESULT_s * result);


#endif	/* BEEP_TYPES_H */

