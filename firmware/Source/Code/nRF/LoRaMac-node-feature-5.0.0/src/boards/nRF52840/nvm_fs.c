#include "nvm_fs.h"

#include <stdint.h>
#include <string.h>
#include "nrf.h"
#include "nordic_common.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "fds.h"
#include "app_error.h"
#define NRF_LOG_MODULE_NAME Flash
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
#include "Commissioning.h"


fds_stat_t stat;


/* Array to map FDS return values to strings. */
char const * fds_err_str[] =
{
    "FDS_SUCCESS",
    "FDS_ERR_OPERATION_TIMEOUT",
    "FDS_ERR_NOT_INITIALIZED",
    "FDS_ERR_UNALIGNED_ADDR",
    "FDS_ERR_INVALID_ARG",
    "FDS_ERR_NULL_ARG",
    "FDS_ERR_NO_OPEN_RECORDS",
    "FDS_ERR_NO_SPACE_IN_FLASH",
    "FDS_ERR_NO_SPACE_IN_QUEUES",
    "FDS_ERR_RECORD_TOO_LARGE",
    "FDS_ERR_NOT_FOUND",
    "FDS_ERR_NO_PAGES",
    "FDS_ERR_USER_LIMIT_REACHED",
    "FDS_ERR_CRC_CHECK_FAILED",
    "FDS_ERR_BUSY",
    "FDS_ERR_INTERNAL",
};

/* Array to map FDS events to strings. */
static char const * fds_evt_str[] =
{
    "FDS_EVT_INIT",
    "FDS_EVT_WRITE",
    "FDS_EVT_UPDATE",
    "FDS_EVT_DEL_RECORD",
    "FDS_EVT_DEL_FILE",
    "FDS_EVT_GC",
};



/* Flash configuration data. */
static flash_struct_s flash_cfg =
{
    .Major_version			= FIRMWARE_MAJOR,
    .Minor_version			= FIRMWARE_MINOR,
	.Sub_version			= FIRMWARE_SUB,
    .boot_count				= 0,
	.LoRaWANnvm				= {0},
    .devEUI					= LORAWAN_DEVICE_EUI,
	.appEUI					= LORAWAN_JOIN_EUI,
	.appKey					= LORAWAN_APP_KEY,
	.lorawanStatus			= BITMASK_ENABLE | BITMASK_DUTYCYCLE | BITMASK_ADR,	// Enable LoRaWAN, the dutycycle and ADR by default.
	.ds18B20Status			= ((4<<1) | 1),
	.ds18B20Interval		= 10,
	.lorawanRatio			= 0,
	.mainSampleInterval_min	= 1,
	.pinCodeLenght			= sizeof(PIN_CODE_DEFAULT) - 1,
	.pinCode				= {PIN_CODE_DEFAULT},
	.hx711ConvChannels		= CH_A_GAIN128,
	.hx711nSamples			= 2,
	.attecSerialNumber		= {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08},
    .audioChannel           = DEFAULT_AUDIO_CHANNEL,
    .audioCoarseGain_dB2    = DEFAULT_AUDIO_COARSE_GAIN,
    .audioVolume_dB2        = DEFAULT_AUDIO_VOLUME,
    .min6dBInputDivider     = DEFAULT_AUDIO_INPUT_DIV,
    .fft_count              = DEFAULT_FFT_DIVIDER,
    .fft_start              = DEFAULT_FFT_START,
    .fft_stop               = DEFAULT_FFT_STOP,
    .alarm_DS = 
    {
        .Min    = DEFAULT_DS_TEMP_MIN,
        .Max    = DEFAULT_DS_TEMP_MAX,
        .Diff   = DEFAULT_DS_TEMP_DIFF,
    },

    .alarm_Supply =
    {
        .Max    = DEFAULT_SUPPLY_MAX,
        .Min    = DEFAULT_SUPPLY_MIN,
        .Diff   = DEFAULT_SUPPLY_DIFF,
    },

    .alarm_HX =
    {
        .Max    = DEFAULT_HX_MAX,
        .Min    = DEFAULT_HX_MIN,
        .Diff   = DEFAULT_HX_DIFF,
    },

	.alarm_BME =
	{
        .Temp_Max		= DEFAULT_BME_TEMP_MAX,
        .Temp_Min		= DEFAULT_BME_TEMP_MIN,
        .Temp_Diff		= DEFAULT_BME_TEMP_DIFF,

        .humidity_Max	= DEFAULT_BME_HUMIDITY_MAX,
		.humidity_Min	= DEFAULT_BME_HUMIDITY_MIN,
		.humidity_Diff	= DEFAULT_BME_HUMIDITY_MIN,

		.press_Max		= DEFAULT_BME_PRESSURE_MAX,
		.press_Min		= DEFAULT_BME_PRESSURE_MIN,
		.press_Diff		= DEFAULT_BME_PRESSURE_DIFF,
	},

    // Added Firmware version 1.3.1
    .prev_main_state    = UINT8_MAX,
};

/* A record containing dummy configuration data. */
static fds_record_t const record =
{
    .file_id           = CONFIG_FILE,
    .key               = CONFIG_REC_KEY,
    .data.p_data       = &flash_cfg,
    /* The length of a record is always expressed in 4-byte units (words). */
    .data.length_words = (sizeof(flash_cfg) + 3) / sizeof(uint32_t),
};

typedef struct
{
    volatile bool   initialized;
    volatile bool   dataChanged;
    volatile bool   storeInitiated;
    volatile bool   gc_called;
    uint32_t	    unstoredDataCount;
}NVM_s;    


NVM_s nvm = {
    .initialized		= false,
    .dataChanged		= false,
    .storeInitiated		= false,
    .gc_called			= false,
    .unstoredDataCount	= 0,
};





static void fds_evt_handler(fds_evt_t const * p_evt)
{
    NRF_LOG_INFO("Event: %s received (%s)", fds_evt_str[p_evt->id], fds_err_str[p_evt->result]);

    switch (p_evt->id)
    {
        case FDS_EVT_INIT:
            if (p_evt->result == FDS_SUCCESS)
            {
                nvm.initialized = true;
            }
            break;

        case FDS_EVT_WRITE:
        {
            if (p_evt->result == FDS_SUCCESS)
            {
				NRF_LOG_INFO("FDS_EVT_WRITE");
                NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->write.record_id);
                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->write.file_id);
                NRF_LOG_INFO("Record key:\t0x%04x", p_evt->write.record_key);
				nvm.unstoredDataCount	= 0;
                nvm.dataChanged		= false;
				nvm.storeInitiated	= false;

            }
        } break;

        case FDS_EVT_DEL_RECORD:
        {
            if (p_evt->result == FDS_SUCCESS)
            {
				NRF_LOG_INFO("FDS_EVT_DEL_RECORD");
                NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->del.record_id);
                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->del.file_id);
                NRF_LOG_INFO("Record key:\t0x%04x", p_evt->del.record_key);
            }
        } break;

	case FDS_EVT_UPDATE:
	    nvm.dataChanged	= false;
	    nvm.storeInitiated	= false;
	    break;

	case FDS_EVT_GC:
	    nvm.dataChanged     = true;
	    nvm.storeInitiated	= false;    
	    nvm.gc_called       = false;
        default:
            break;
    }
}


void nvm_fds_changed(void)
{
    nvm.dataChanged = true;
    nvm.unstoredDataCount++;
}



static void nvm_fds_store(void)
{
//    NRF_LOG_INFO(">Store Flash data");
    
    fds_record_desc_t desc  = {0};
    fds_find_token_t  token = {0};

    ret_code_t rc = fds_record_find(CONFIG_FILE, CONFIG_REC_KEY, &desc, &token);

    if (rc == FDS_SUCCESS)
    {
        fds_flash_record_t config = {0};

        rc = fds_record_update(&desc, &record);
        if(rc == FDS_ERR_NO_SPACE_IN_FLASH) 
        {
            NRF_LOG_INFO("FDS_ERR_NO_SPACE_IN_FLASH, initiating garbage collect");
            if(!nvm.gc_called)
            {
               nvm.gc_called = true;
               fds_gc();
            }
        } 
        else if(rc == FDS_ERR_NO_SPACE_IN_QUEUES)
        {
            NRF_LOG_ERROR("FDS_ERR_NO_SPACE_IN_QUEUES after fds_record_update() call");
        }
        else if(rc != FDS_SUCCESS)
        {
			NRF_LOG_INFO("Update return value: %04X", rc);
            APP_ERROR_CHECK(rc);
        }
    }
    else
    {
        /* struct not found; write a new one. */
        NRF_LOG_INFO("Writing inital data to flash...");

        rc = fds_record_write(&desc, &record);
        APP_ERROR_CHECK(rc);
    }
}


bool nvm_fds_check_busy(void)
{
	return (nvm.dataChanged || nvm.storeInitiated);
}


void nvm_fds_check_pending(void)
{
    if(nvm.dataChanged && !nvm.storeInitiated)
    {
		nvm.storeInitiated = true;
        nvm_fds_store();
    }
}

static void nvm_fds_check_version(void)
{
	const uint32_t current_fw_version = FIRMWARE_TO_UINT32_T(FIRMWARE_MAJOR, FIRMWARE_MINOR, FIRMWARE_SUB);
	const uint32_t stored_fw_version  = FIRMWARE_TO_UINT32_T(flash_cfg.Major_version, flash_cfg.Minor_version, flash_cfg.Sub_version);

	// Check if the current firmware and flash config match
	if(current_fw_version == stored_fw_version)
	{
		return;
	}

    if(stored_fw_version < FIRMWARE_TO_UINT32_T(1, 3, 0)) // 1.3.0
    {
        flash_cfg.audioChannel				= DEFAULT_AUDIO_CHANNEL;
        flash_cfg.audioCoarseGain_dB2		= DEFAULT_AUDIO_COARSE_GAIN;
        flash_cfg.audioVolume_dB2			= DEFAULT_AUDIO_VOLUME;
        flash_cfg.min6dBInputDivider		= DEFAULT_AUDIO_INPUT_DIV;
        flash_cfg.fft_count					= DEFAULT_FFT_DIVIDER;
        flash_cfg.alarm_DS.Min				= DEFAULT_DS_TEMP_MIN;
        flash_cfg.alarm_DS.Max				= DEFAULT_DS_TEMP_MAX;
        flash_cfg.alarm_DS.Diff				= DEFAULT_DS_TEMP_DIFF;
        flash_cfg.alarm_Supply.Max			= DEFAULT_SUPPLY_MAX;
        flash_cfg.alarm_Supply.Min			= DEFAULT_SUPPLY_MIN;
        flash_cfg.alarm_Supply.Diff			= DEFAULT_SUPPLY_DIFF;
        flash_cfg.alarm_HX.Max				= DEFAULT_HX_MAX;
        flash_cfg.alarm_HX.Min				= DEFAULT_HX_MIN;
        flash_cfg.alarm_HX.Diff				= DEFAULT_HX_DIFF;
        flash_cfg.alarm_BME.Temp_Max		= DEFAULT_BME_TEMP_MAX;
		flash_cfg.alarm_BME.Temp_Min		= DEFAULT_BME_TEMP_MIN;
		flash_cfg.alarm_BME.Temp_Diff		= DEFAULT_BME_TEMP_DIFF;
		flash_cfg.alarm_BME.humidity_Max	= DEFAULT_BME_HUMIDITY_MAX;
		flash_cfg.alarm_BME.humidity_Min	= DEFAULT_BME_HUMIDITY_MIN;
		flash_cfg.alarm_BME.humidity_Diff	= DEFAULT_BME_HUMIDITY_DIFF;
		flash_cfg.alarm_BME.press_Max		= DEFAULT_BME_PRESSURE_MAX;
		flash_cfg.alarm_BME.press_Min		= DEFAULT_BME_PRESSURE_MIN;
		flash_cfg.alarm_BME.press_Diff		= DEFAULT_BME_PRESSURE_DIFF;
	}

    if(stored_fw_version < FIRMWARE_TO_UINT32_T(1, 3, 1)) // 1.3.1
    {
        flash_cfg.prev_main_state           = UINT8_MAX;
    }

	NRF_LOG_INFO("Older Flash version detected: %u != %u", current_fw_version, stored_fw_version);
	NRF_LOG_FLUSH();

	// Update the flash storage fw version 
	flash_cfg.Major_version	= FIRMWARE_MAJOR;
	flash_cfg.Minor_version = FIRMWARE_MINOR;
	flash_cfg.Sub_version	= FIRMWARE_SUB;
    nvm_fds_changed();
}

static void nvm_fds_check_parameters(void)
{
    if(flash_cfg.audioChannel > AIN_IN2RP)
    {
        flash_cfg.audioChannel = DEFAULT_AUDIO_CHANNEL;
    }

    if(flash_cfg.audioCoarseGain_dB2 > (40 * 2)) // Max +40 dBm
    {
        flash_cfg.audioCoarseGain_dB2   = DEFAULT_AUDIO_COARSE_GAIN;
    }

    if((flash_cfg.min6dBInputDivider != true) && (flash_cfg.min6dBInputDivider != false))
    {
        flash_cfg.min6dBInputDivider = DEFAULT_AUDIO_INPUT_DIV;
    }

    if(flash_cfg.audioVolume_dB2 < -24 || flash_cfg.audioVolume_dB2 > 40)
    {
        flash_cfg.audioVolume_dB2 = DEFAULT_AUDIO_VOLUME;
    }

    if(flash_cfg.fft_count == 0 || flash_cfg.fft_count > FFT_MAX_BINS)
    {
        flash_cfg.fft_count = DEFAULT_FFT_DIVIDER;
    }

    if(flash_cfg.fft_start >= flash_cfg.fft_stop)
    {
        flash_cfg.fft_start = DEFAULT_FFT_START;
        flash_cfg.fft_stop  = DEFAULT_FFT_STOP;
    }

    // Check if the pin code is invalid.
    if(!is_array_ASCIInumbers(flash_cfg.pinCode, PIN_CODE_BLE_LENGHT))
    {
        uint8_t pincode[PIN_CODE_BLE_LENGHT] = {PIN_CODE_DEFAULT};
        memcpy(flash_cfg.pinCode, pincode, PIN_CODE_BLE_LENGHT);
    }


    // Check the limits of the Alarms
    if( (uint16_t)flash_cfg.alarm_DS.Max  == UINT16_MAX && 
        (uint16_t)flash_cfg.alarm_DS.Min  == UINT16_MAX && 
        (uint16_t)flash_cfg.alarm_DS.Diff == UINT16_MAX)
    {
        flash_cfg.alarm_DS.Min    = DEFAULT_DS_TEMP_MIN;
        flash_cfg.alarm_DS.Max    = DEFAULT_DS_TEMP_MAX;
        flash_cfg.alarm_DS.Diff   = DEFAULT_DS_TEMP_DIFF;
    }

    if( (uint32_t)flash_cfg.alarm_HX.Max  == UINT32_MAX &&
        (uint32_t)flash_cfg.alarm_HX.Min  == UINT32_MAX &&
        (uint32_t)flash_cfg.alarm_HX.Diff == UINT32_MAX)
    {
        flash_cfg.alarm_HX.Max    = DEFAULT_HX_MAX ;
        flash_cfg.alarm_HX.Min    = DEFAULT_HX_MIN;
        flash_cfg.alarm_HX.Diff   = DEFAULT_HX_DIFF;
    }

    if( flash_cfg.alarm_Supply.Max  == UINT16_MAX && 
        flash_cfg.alarm_Supply.Min  == UINT16_MAX && 
        flash_cfg.alarm_Supply.Diff == UINT16_MAX)
    {
        flash_cfg.alarm_Supply.Max    = DEFAULT_SUPPLY_MAX;
        flash_cfg.alarm_Supply.Min    = DEFAULT_SUPPLY_MIN;
        flash_cfg.alarm_Supply.Diff   = DEFAULT_SUPPLY_DIFF;
    }

	// BME280 Temp
	if( (uint16_t)flash_cfg.alarm_BME.Temp_Max  == UINT16_MAX &&
		(uint16_t)flash_cfg.alarm_BME.Temp_Min  == UINT16_MAX &&
		(uint16_t)flash_cfg.alarm_BME.Temp_Diff == UINT16_MAX)
	{
		flash_cfg.alarm_BME.Temp_Max	= DEFAULT_BME_TEMP_MAX;
		flash_cfg.alarm_BME.Temp_Min	= DEFAULT_BME_TEMP_MIN;
		flash_cfg.alarm_BME.Temp_Diff	= DEFAULT_BME_TEMP_DIFF;
	}

    if( flash_cfg.alarm_BME.humidity_Max  == UINT16_MAX &&
		flash_cfg.alarm_BME.humidity_Min  == UINT16_MAX &&
		flash_cfg.alarm_BME.humidity_Diff == UINT16_MAX)
	{
		flash_cfg.alarm_BME.humidity_Max  = DEFAULT_BME_HUMIDITY_MAX;
		flash_cfg.alarm_BME.humidity_Min  = DEFAULT_BME_HUMIDITY_MIN;
		flash_cfg.alarm_BME.humidity_Diff = DEFAULT_BME_HUMIDITY_DIFF;
	}

    if( flash_cfg.alarm_BME.press_Max  == UINT16_MAX &&
		flash_cfg.alarm_BME.press_Min  == UINT16_MAX &&
		flash_cfg.alarm_BME.press_Diff == UINT16_MAX)
	{
		flash_cfg.alarm_BME.press_Max  = DEFAULT_BME_PRESSURE_MAX;
		flash_cfg.alarm_BME.press_Min  = DEFAULT_BME_PRESSURE_MIN;
		flash_cfg.alarm_BME.press_Diff = DEFAULT_BME_PRESSURE_DIFF;
	}
}

 

void nvm_fds_init(void)
{
    uint32_t retVal;
    NRF_LOG_INFO("FDS init started.")

    /* Register first to receive an event when initialization is complete. */
    fds_register(fds_evt_handler);

    NRF_LOG_INFO("Initializing fds...");

    retVal = fds_init();

    while (!nvm.initialized)
    {
        #ifdef SOFTDEVICE_PRESENT
	    (void) sd_app_evt_wait();
		#else
			__WFE();
		#endif
    }

    // Clear the stat structure
    memset(&stat, 0, sizeof(fds_stat_t));

    retVal = fds_stat(&stat);
    APP_ERROR_CHECK(retVal);

    NRF_LOG_INFO("Found %d valid records and %d dirty records (ready to be garbage collected).", stat.valid_records, stat.dirty_records);

    fds_record_desc_t desc = {0};
    fds_find_token_t  tok  = {0};

    // Find the record in the flash.
    retVal = fds_record_find(CONFIG_FILE, CONFIG_REC_KEY, &desc, &tok);

    if (retVal == FDS_SUCCESS)
    {
        /* A config file is in flash. Let's update it. */
        fds_flash_record_t config = {0};

        /* Open the record and read its contents. */
        retVal = fds_record_open(&desc, &config);
        APP_ERROR_CHECK(retVal); 

		// Copy the Flash data contents in the ram to the NVM_s structure.
		memcpy(&flash_cfg, config.p_data, sizeof(flash_struct_s));

        /* Close the record when done reading. */
        retVal = fds_record_close(&desc);
        APP_ERROR_CHECK(retVal);

        /* Write the updated record to flash. */
        retVal = fds_record_update(&desc, &record);

		if(retVal != FDS_SUCCESS)
		{
			NRF_LOG_INFO("fds_record_update error: %u/0x%04X = %s", retVal, retVal, fds_err_str[retVal]);

			if(retVal == FDS_ERR_NO_SPACE_IN_FLASH)
			{
				// Perform a garbage collect to remove all dirty records, then retry updating the record.
				fds_gc();
				retVal = fds_record_update(&desc, &record);
			}
			APP_ERROR_CHECK(retVal);
		}

        NRF_LOG_INFO("Config file found revision: %u.%u.%u, boot count: %d.", flash_cfg.Major_version, flash_cfg.Minor_version, flash_cfg.Sub_version, flash_cfg.boot_count);

        /* Update boot count. */
        flash_cfg.boot_count++;

        /* Check the stored firmware version to identify missing parameters which might have been adeed in a new release. */
        nvm_fds_check_version();

        /* Check if the parameters are within a valid range */
        nvm_fds_check_parameters();
    }
    else
    {
        /* System config not found; write a new one. */
        NRF_LOG_INFO("Config file not found, writing new file...");

        retVal = fds_record_write(&desc, &record);
        APP_ERROR_CHECK(retVal);
    }
}


void nvm_fds_eeprom_wrapper_read( uint16_t addr, uint8_t *buffer, uint16_t size )
{
	uint16_t i;

	if((addr + size) >= LORAWAN_NVM_SIZE){
		return;
	}

	for( i = 0 ; i < size ; i++)
	{
		buffer[i] = flash_cfg.LoRaWANnvm[addr + i];
	}
}


void nvm_fds_eeprom_wrapper_write( uint16_t addr, uint8_t *buffer, uint16_t size )
{
	uint16_t i;

	if((addr + size) >= LORAWAN_NVM_SIZE){
		return;
	}

	for( i = 0 ; i < size ; i++)
	{
		flash_cfg.LoRaWANnvm[addr + i] = buffer[i];
	}
}


uint32_t nvm_fds_eeprom_get(BEEP_protocol_s * prot)
{
	if(prot == NULL)
	{
		return NRF_ERROR_NULL;
	}

	switch(prot->command)
	{
		//-----------------------------------------------------------------------------
		case READ_FIRMWARE_VERSION:
		{
			FH_VERSION_s * version = &prot->param.version;
            version->major	= FIRMWARE_MAJOR;
            version->minor	= FIRMWARE_MINOR;
            version->sub	= FIRMWARE_SUB;
            version->id		= 0;
			break;
		}

        //-----------------------------------------------------------------------------
		case READ_HARDWARE_VERSION:
		{
			#ifdef DEBUG
				uint32_t revision = (HARDWARE_MAJOR << 16) | (HARDWARE_MINOR << 0);
				uint32_t hardware = HARDWARE_ID;
			#else
				uint32_t revision = NRF_UICR->CUSTOMER[UICR_HARDWARE_REVISION_LOCATION];
				uint32_t hardware = NRF_UICR->CUSTOMER[UICR_HARDWARE_SERIAL_NUMBER];
			#endif

            FH_VERSION_s * version = &prot->param.version;
            version->major	= (uint16_t)(revision >> 16);
            version->minor	= (uint16_t)(revision >> 0);
            version->sub	= 0;
			version->id		= hardware;
			break;
		}

		//-----------------------------------------------------------------------------
		case READ_LORAWAN_DEVEUI:
			prot->param.lorawan_key.lenght = 8;
			memcpy(&prot->param.lorawan_key.data, flash_cfg.devEUI,  prot->param.lorawan_key.lenght);
			break;
		case READ_LORAWAN_APPEUI:
			prot->param.lorawan_key.lenght = 8;
			memcpy(&prot->param.lorawan_key.data, flash_cfg.appEUI, prot->param.lorawan_key.lenght);
			break;
		case READ_LORAWAN_APPKEY:
			prot->param.lorawan_key.lenght = 16;
			memcpy(&prot->param.lorawan_key.data, flash_cfg.appKey, prot->param.lorawan_key.lenght);
			break;

		case READ_LORAWAN_STATE:
			prot->param.status.statusflag = flash_cfg.lorawanStatus;
			break;

		//-----------------------------------------------------------------------------
		case READ_DS18B20_STATE:
			prot->param.status.statusflag = flash_cfg.ds18B20Status; 
            prot->param.status.interval	  = flash_cfg.ds18B20Interval;
			break;

		//-----------------------------------------------------------------------------
		case READ_ATECC_READ_ID:
            memcpy(prot->param.atecc_id.ROM, flash_cfg.attecSerialNumber, ATECC_ID_LENGHT);
			break;

        //-----------------------------------------------------------------------------
		case READ_PINCODE:
		case WRITE_PINCODE:
			flash_cfg.pinCodeLenght = PIN_CODE_BLE_LENGHT;
			prot->param.lorawan_key.lenght	= flash_cfg.pinCodeLenght;
			memcpy(prot->param.lorawan_key.data, flash_cfg.pinCode, flash_cfg.pinCodeLenght);	
			break;

		//-----------------------------------------------------------------------------
		case READ_APPLICATION_CONFIG:
			prot->param.status.statusflag   = flash_cfg.lorawanRatio; 
            prot->param.status.interval     = flash_cfg.mainSampleInterval_min;		
			break;

		//-----------------------------------------------------------------------------
		case READ_HX711_STATE:
			prot->param.status.statusflag	= flash_cfg.hx711ConvChannels; 
            prot->param.status.interval		= flash_cfg.hx711nSamples;
			break;

		//-----------------------------------------------------------------------------
        case READ_BOOT_COUNT:
            prot->param.status.interval     = flash_cfg.boot_count;
            break;

		//-----------------------------------------------------------------------------
        case READ_AUDIO_ADC_CONFIG:
        {
            AUDIO_CONFIG_s * conf = &prot->param.audio_config;
            conf->channel       = flash_cfg.audioChannel;
            conf->gain          = flash_cfg.audioCoarseGain_dB2;
            conf->volume        = flash_cfg.audioVolume_dB2;
            conf->min6dB        = flash_cfg.min6dBInputDivider;
            conf->fft_count     = flash_cfg.fft_count;
            conf->fft_start     = flash_cfg.fft_start;
            conf->fft_stop      = flash_cfg.fft_stop;
            break;
        }

        //-----------------------------------------------------------------------------
        case ALARM_CONFIG_READ:
        {
            ALARM_CONFIG_s * alarm = &prot->param.alarm;
            switch(alarm->type)
            {
                case DS18B20:
                    memcpy(&alarm->thr, &flash_cfg.alarm_DS, sizeof(DS_ALARM_s));
                    break;

                case HX711:
                    memcpy(&alarm->thr, &flash_cfg.alarm_HX, sizeof(HX711_ALARM_s));
                    break;
                case BME280:
                    memcpy(&alarm->thr, &flash_cfg.alarm_BME, sizeof(BME_ALARM_s));
                    break;
//                case AUDIO_ADC:
//                    break;
                case nRF_ADC:
                    memcpy(&alarm->thr, &flash_cfg.alarm_Supply, sizeof(SUPPLY_ALARM_s));
                    break;
                default:
                    return NRF_ERROR_INVALID_PARAM;
                    break;
            }
            break;
        }
                    
		//-----------------------------------------------------------------------------
		default:
			return NRF_ERROR_NOT_FOUND;
			break;
	}
    return NRF_SUCCESS;
}


uint32_t nvm_fds_eeprom_set(BEEP_protocol_s * prot)
{
	if(prot == NULL)
	{
		return NRF_ERROR_NULL;
	}

	switch(prot->command)
	{
		//-----------------------------------------------------------------------------
		case WRITE_LORAWAN_DEVEUI:
			memcpy(flash_cfg.devEUI, &prot->param.lorawan_key.data, 8);
			break;
		case WRITE_LORAWAN_APPEUI:
			memcpy(flash_cfg.appEUI, &prot->param.lorawan_key.data, 8);
			break;
		case WRITE_LORAWAN_APPKEY:
			memcpy(flash_cfg.appKey, &prot->param.lorawan_key.data, 16);
			break;
		case READ_LORAWAN_STATE:
			flash_cfg.lorawanStatus = prot->param.status.statusflag;
			break;

		//-----------------------------------------------------------------------------
		case READ_DS18B20_STATE:
			flash_cfg.ds18B20Status		= prot->param.status.statusflag; 
            flash_cfg.ds18B20Interval	= prot->param.status.interval;
			break;

        //-----------------------------------------------------------------------------
		case READ_APPLICATION_CONFIG:
		case WRITE_APPLICATION_CONFIG:
			if(prot->param.status.interval == 0)
			{
				return NRF_ERROR_INVALID_PARAM;
			}
			flash_cfg.lorawanRatio				= prot->param.status.statusflag; 
            flash_cfg.mainSampleInterval_min	= prot->param.status.interval;		
			break;

        //-----------------------------------------------------------------------------
		case READ_PINCODE:
		case WRITE_PINCODE:
			if(prot->param.lorawan_key.lenght != PIN_CODE_BLE_LENGHT)
			{
				return NRF_ERROR_INVALID_PARAM;
			}
			flash_cfg.pinCodeLenght = prot->param.lorawan_key.lenght;
			memset(flash_cfg.pinCode, 0, PIN_CODE_LENGHT_MAX + 1); // Clear the pincode buffer
			memcpy(flash_cfg.pinCode, prot->param.lorawan_key.data, flash_cfg.pinCodeLenght);	
			break;

		//-----------------------------------------------------------------------------
		case WRITE_HX711_STATE:
			// Check if any other bits are set other than the channel bits.
			if(prot->param.status.statusflag & ~CH_HX_BITMASK)
			{
				return NRF_ERROR_INVALID_PARAM;
			}

			// Store the channels on which the HX711 must convert on each sample interval.
			flash_cfg.hx711ConvChannels = prot->param.status.statusflag;
            flash_cfg.hx711nSamples		= prot->param.status.interval;
			break;

        //-----------------------------------------------------------------------------
		case READ_ATECC_READ_ID:
            memcpy(flash_cfg.attecSerialNumber, prot->param.atecc_id.ROM, ATECC_ID_LENGHT);
			break;

        //-----------------------------------------------------------------------------
        case WRITE_BOOT_COUNT:
            flash_cfg.boot_count = prot->param.status.interval;
            break;

        //-----------------------------------------------------------------------------
        case READ_AUDIO_ADC_CONFIG:
        case WRITE_AUDIO_ADC_CONFIG:
        {
            AUDIO_CONFIG_s * conf = &prot->param.audio_config;

            if(conf->channel > AIN_IN2RP)
            {
                return NRF_ERROR_INVALID_PARAM;
            }

            if(conf->gain > (40 * 2)) // Max +40 dBm
            {
                return NRF_ERROR_INVALID_PARAM;
            }

            if(conf->volume < -24 || conf->volume > 40)
            {
                return NRF_ERROR_INVALID_PARAM;
            }

            if(conf->fft_count == 0 || conf->fft_count > FFT_MAX_BINS)
            {
                return NRF_ERROR_INVALID_PARAM;
            }

            if(conf->fft_start >= conf->fft_stop)
            {
                return NRF_ERROR_INVALID_PARAM;
            }

            flash_cfg.audioChannel          = conf->channel;
            flash_cfg.audioCoarseGain_dB2   = conf->gain;
            flash_cfg.audioVolume_dB2       = conf->volume;
            flash_cfg.fft_count             = conf->fft_count;
            flash_cfg.fft_start             = conf->fft_start;
            flash_cfg.fft_stop              = conf->fft_stop;
            break;
        }

		//-----------------------------------------------------------------------------
        case ALARM_CONFIG_WRITE:
        {
            ALARM_CONFIG_s * alarm = &prot->param.alarm;
            switch(alarm->type)
            {
                case DS18B20:
                    if(alarm->thr.ds.Min > alarm->thr.ds.Max || alarm->thr.ds.Diff == 0)
                    {
                        return NRF_ERROR_INVALID_PARAM;
                    }
                    memcpy(&flash_cfg.alarm_DS, &alarm->thr.ds, sizeof(DS_ALARM_s));
                    break;

                case HX711:
                    if(alarm->thr.hx.Min > alarm->thr.hx.Max || alarm->thr.hx.Diff == 0)
                    {
                        return NRF_ERROR_INVALID_PARAM;
                    }
                    memcpy(&flash_cfg.alarm_HX, &alarm->thr.hx, sizeof(HX711_ALARM_s));
                    break;

                case BME280:
                    if( alarm->thr.bme.Temp_Min > alarm->thr.bme.Temp_Max           || alarm->thr.bme.Temp_Diff == 0        ||
                        alarm->thr.bme.humidity_Min > alarm->thr.bme.humidity_Max   || alarm->thr.bme.humidity_Diff == 0    ||
                        alarm->thr.bme.press_Min > alarm->thr.bme.press_Max         || alarm->thr.bme.press_Diff == 0)
                    {
                        return NRF_ERROR_INVALID_PARAM;
                    }
					memcpy(&flash_cfg.alarm_BME, &alarm->thr.bme, sizeof(BME_ALARM_s));
                    break;

                case nRF_ADC:
                    if(alarm->thr.supply.Min > alarm->thr.supply.Max || alarm->thr.supply.Diff == 0)
                    {
                        return NRF_ERROR_INVALID_PARAM;
                    }
                    memcpy(&flash_cfg.alarm_Supply, &alarm->thr.supply, sizeof(SUPPLY_ALARM_s));
                    break;

                default:
                    return NRF_ERROR_INVALID_PARAM;
                    break;
            }
            break;
        }

		//-----------------------------------------------------------------------------
		default:
			return NRF_ERROR_NOT_FOUND;
			break;	
	}
    nvm_fds_changed();
    return NRF_SUCCESS;
}




uint32_t    nvm_fds_internal_param_get(INTERNAL_PARAM * internal)
{
    if(internal == NULL)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    switch(internal->param)
    {
        //----------------------------------------------------------------------------------
        case PREVIOUS_MAIN_STATE:
            internal->value.previous_state = flash_cfg.prev_main_state;
            break;

        //----------------------------------------------------------------------------------
        default:
            return NRF_ERROR_INVALID_STATE;
            break;
    }
    return NRF_SUCCESS;
}



uint32_t    nvm_fds_internal_param_set(INTERNAL_PARAM * internal)
{
    if(internal == NULL)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    switch(internal->param)
    {
        //----------------------------------------------------------------------------------
        case PREVIOUS_MAIN_STATE:
            flash_cfg.prev_main_state = internal->value.previous_state;
            break;

        //----------------------------------------------------------------------------------
        default:
            return NRF_ERROR_INVALID_STATE;
            break;
    }
    nvm_fds_changed();    
    return NRF_SUCCESS;
}





















