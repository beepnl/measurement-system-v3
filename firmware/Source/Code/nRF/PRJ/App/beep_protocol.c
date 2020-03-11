
#include <stdbool.h>
#include <stdint.h>
#include "SQ.h"
#include "app_timer.h"
#define NRF_LOG_MODULE_NAME BEEP_protocol
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include "boards.h"
#include "beep_protocol.h"
#include "app_util.h"
#include "nrf_strerror.h"


static const char * beep_protocol_cmd_strget(BEEP_CID cmd)
{
	uint8_t index = cmd & ~BEEP_WRITE;
	#if 1
		const char * cmdStrings[] =
		{
			"RESPONSE", // 0
			"FIRMWARE_VERSION",
			"HARDWARE_VERSION",
			"DS18B20_STATE",
			"DS18B20_CONVERSION",
			"DS18B20_CONFIG",
			"BME280_STATE",
			"BME280_CONVERSION",
			"BME280_I2C",
			"HX711_STATE",
			"HX711_CONVERSION", // 10
			"AUDIO_ADC_STATE",
			"AUDIO_ADC_CONVERSION",
			"AUDIO_ADC_I2C",
			"ATECC_READ_ID",
			"ATECC_I2C",
			"BUZZER_STATE",
			"BUZZER_TUNE",
			"BUZZER_CUSTOM_TUNE",
			"SQ_MIN_STATE",
			"LORAWAN_STATE", // 20
			"LORAWAN_DEVEUI",
			"LORAWAN_APPEUI",
			"LORAWAN_APPKEY",
			"LORAWAN_TRANSMIT",
			"CID_nRF_FLASH",
			"nRF_ADC_CONFIG",
			"nRF_ADC_CONVERSION",
			"APPLICATION_STATE",
			"APPLICATION_CONFIG",
			"PIN_CODE", // 30
            "BOOT_COUNT",
            "READ_MX_FLASH",
            "ERASE_MX_FLASH",
            "SIZE_FLASH",
			"CID_UNKNOWN", 
		};
		return ((index >= CID_UNKNOWN) ? cmdStrings[CID_UNKNOWN] : cmdStrings[index]);
	#else
		return NULL;

	#endif
}






const char * beep_protocol_sensor_strget(SENSOR_TYPE sensorType)
{
	#if 1
		const char * cmdStrings[] =
		{
			"DS18B20",
			"BME280",
			"HX711",
			"AUDIO_ADC",
			"nRF_ADC",
			"SQ_MIN",
			"ATECC",
			"BUZZER",
			"LORAWAN",
			"MX_FLASH",
			"nRF_FLASH",
			"APPLICATIE",
			"Unknown sensor type"
		};
		return ((sensorType >= APPLICATIE) ? cmdStrings[APPLICATIE + 1] : cmdStrings[sensorType]);
	#else
		return NULL;

	#endif
}

/*
 *@Brief: Function to check if the message lenght is ok. Returns true when the lenght is larger or equal to the expected amount, returns false when shorter than required
 */
static uint32_t beep_protocol_check_lenght(BEEP_CID cmd, uint8_t lenght, uint8_t * payload, uint8_t * cmdLenght)
{
	uint32_t retVal = NRF_ERROR_INVALID_LENGTH;
	
	switch(cmd)
	{
		//-----------------------------------------------------------------------------
		case RESPONSE:
			if(lenght >= 6)
			{
				*cmdLenght += 6;
				retVal = NRF_SUCCESS;
			}
			else
			{
				retVal = NRF_ERROR_INVALID_LENGTH;
			}
			break;

        //-----------------------------------------------------------------------------
		case WRITE_DS18B20_STATE:
			if(lenght >= 2)
			{
				*cmdLenght += 2;
				retVal = NRF_SUCCESS;
			}
			break;

        //-----------------------------------------------------------------------------
		case WRITE_APPLICATION_CONFIG:
        case WRITE_HX711_CONVERSION:
        case WRITE_HX711_STATE:
			if(lenght >= 3)
			{
				*cmdLenght += 3;
				retVal = NRF_SUCCESS;
			}
			break;

        //-----------------------------------------------------------------------------
        case WRITE_AUDIO_ADC_CONFIG:
            if(lenght >= 6)
			{
				*cmdLenght += 6;
				retVal = NRF_SUCCESS;
			}
            break;

		//-----------------------------------------------------------------------------
        case WRITE_LORAWAN_STATE:
		case WRITE_DS18B20_CONVERSION:
        case WRITE_BUZZER_DEFAULT_TUNE:
        case ALARM_CONFIG_READ:
        case ERASE_MX_FLASH:
			if(lenght >= 2)
			{
				*cmdLenght += 2;
				retVal = NRF_SUCCESS;
			}
			break;

		//-----------------------------------------------------------------------------
        case WRITE_LORAWAN_TRANSMIT:
		{
			uint8_t msgLenght = 0;
			if(lenght >= 2)
			{
				*cmdLenght += 2;
                msgLenght = payload[1];

				if(lenght < (2 + msgLenght))
				{
					return NRF_ERROR_INVALID_LENGTH;
				}
                *cmdLenght += msgLenght;
				retVal = NRF_SUCCESS;
			}
			break;
		}

		//-----------------------------------------------------------------------------
		case WRITE_PINCODE:
		{
			uint8_t msgLenght = 0;
			if(lenght >= 2)
			{
                msgLenght = payload[1];

				if(lenght < msgLenght || msgLenght != PIN_CODE_BLE_LENGHT)
				{
					return NRF_ERROR_INVALID_LENGTH;
				}
                *cmdLenght += msgLenght + 2;
				retVal = NRF_SUCCESS;
			}
			break;
		}

        //-----------------------------------------------------------------------------
		case READ_HX711_STATE:
        case READ_ATECC_READ_ID:
        case READ_FIRMWARE_VERSION:
        case READ_HARDWARE_VERSION:
        case READ_LORAWAN_STATE:
        case READ_LORAWAN_DEVEUI:
        case READ_LORAWAN_APPEUI:
        case READ_LORAWAN_APPKEY:
        case READ_nRF_ADC_CONVERSION:
		case WRITE_nRF_ADC_CONVERSION:
		case READ_DS18B20_STATE:
		case READ_DS18B20_CONVERSION:
        case READ_HX711_CONVERSION:
		case READ_APPLICATION_CONFIG:
		case READ_PINCODE:
        case SIZE_MX_FLASH:
        case READ_BOOT_COUNT:
        case READ_AUDIO_ADC_CONVERSION:
        case START_AUDIO_ADC_CONVERSION:
        case READ_AUDIO_ADC_CONFIG:
        case ALARM_STATUS_READ:
		case BME280_CONVERSION_READ:
		case BME280_CONVERSION_START:
			if(lenght >= 1)
			{
				*cmdLenght += 1;
				retVal = NRF_SUCCESS;
			}
			break;

        case READ_MX_FLASH:
        	if(lenght >= 5)
			{
				*cmdLenght += 5;
				retVal = NRF_SUCCESS;
			}
            break;

		//-----------------------------------------------------------------------------
        case READ_nRF_ADC_CONFIG:
			break;

        case WRITE_LORAWAN_DEVEUI:
        case WRITE_LORAWAN_APPEUI:
			if(lenght >= 9)
			{
				*cmdLenght += 9;
				retVal = NRF_SUCCESS;
			}
			break;
        case WRITE_LORAWAN_APPKEY:
			if(lenght >= 17)
			{
				*cmdLenght += 17;
				retVal = NRF_SUCCESS;
			}
			break;

        //-----------------------------------------------------------------------------
		case WRITE_BUZZER_CUSTOM_TUNE:
        	if(lenght >= 8)
			{
				*cmdLenght += 8;
				retVal = NRF_SUCCESS;
			}
			break;

        //-----------------------------------------------------------------------------
        case ALARM_CONFIG_WRITE:
        {
            const uint8_t type = payload[1];
            if(lenght >= 2)
            {
                *cmdLenght += 2;
            }

            switch(type)
            {
                case DS18B20:
                case nRF_ADC:
                    if(lenght >= (3 * sizeof(uint16_t))+2)
                    {
                        *cmdLenght += (3 * sizeof(uint16_t));
                        retVal = NRF_SUCCESS;
                    }
                    break;

                case HX711:
                    if(lenght >= (3 * sizeof(uint32_t))+2)
                    {
                        *cmdLenght += (3 * sizeof(int32_t));
                        retVal = NRF_SUCCESS;    
                    }
                    break;

                case BME280:
					if(lenght >= (9 * sizeof(int16_t))+2)
                    {
                        *cmdLenght += (9 * sizeof(int16_t));
                        retVal = NRF_SUCCESS;    
                    }
					break;
                case AUDIO_ADC:
                default:
                    return NRF_ERROR_INVALID_PARAM;
                    break;
            }
            break;
        }

        //-----------------------------------------------------------------------------
		case READ_DS18B20_CONFIG:
        case READ_ATECC_I2C:
        case READ_BUZZER_STATE:
        case READ_SQ_MIN_STATE:
        case READ_CID_nRF_FLASH:
        case READ_APPLICATION_STATE:
			break;
			
		//-----------------------------------------------------------------------------
		default:
			retVal = NRF_ERROR_NOT_FOUND;
			break;
	}
	
	return retVal;
}


static uint32_t beep_protocol_log(BEEP_protocol_s * prot)
{
	if(prot == NULL)
	{
		return NRF_ERROR_INVALID_PARAM;
	}

    switch(prot->command)
	{
		//-----------------------------------------------------------------------------
		case RESPONSE:
		{
			RESPONSE_s * err = &prot->param.error;

			NRF_LOG_INFO("Protocol command response: %u/0x%02X=%s, Error Code: %u/0x%04X=%s",
				err->ErrorCmd,
				err->ErrorCmd,
                beep_protocol_cmd_strget(err->ErrorCmd),
				err->errorCode,
				err->errorCode,
				nrf_strerror_get(err->errorCode));
		}
			break;

		//-----------------------------------------------------------------------------
		case READ_FIRMWARE_VERSION:
		{
			FH_VERSION_s * version = &prot->param.version;
			NRF_LOG_INFO("%s: %u.%u.%u",
					beep_protocol_cmd_strget(prot->command),
                    version->major,
                    version->minor,
                    version->sub);
			break;
		}

        //-----------------------------------------------------------------------------
		case READ_HARDWARE_VERSION:
		{
			FH_VERSION_s * version = &prot->param.version;
			NRF_LOG_INFO("%s: %u.%u ID:%u",
					beep_protocol_cmd_strget(prot->command),
                    version->major,
                    version->minor,
                    version->id);
			break;
		}

		//-----------------------------------------------------------------------------
		case READ_DS18B20_CONVERSION:
		{
			float temp = 0.0;
			uint8_t i;
			DS18B20_RESULTS_s * ds = &prot->param.meas_result.result.ds18B20;
    
            #if 0
                NRF_LOG_INFO("%u DS18B20 measurement results: ", ds->devices);
            #endif
			for(i=0; i<ds->devices; i++)
			{
				temp = ((float)((int16_t)ds->temperatures[i])) / 100.0;
                #if 0
                    NRF_LOG_INFO("[%u] - %i/0x%04X=" NRF_LOG_FLOAT_MARKER, i, (int16_t)ds->temperatures[i], ds->temperatures[i], NRF_LOG_FLOAT(temp));
                #endif
			}
			break;
		}

		//-----------------------------------------------------------------------------
        case READ_DS18B20_STATE:
			break;

		//-----------------------------------------------------------------------------
		case WRITE_DS18B20_STATE:
		{
			NRF_LOG_INFO("New state=%u/0x%02X", prot->param.status.statusflag, prot->param.status.statusflag);
			break;
		}

        //-----------------------------------------------------------------------------
		case READ_HX711_STATE:
			NRF_LOG_INFO("READ_HX711_STATE: %u/0x%02X", prot->param.status.statusflag, prot->param.status.statusflag);
			break;

        //-----------------------------------------------------------------------------
		case WRITE_HX711_STATE:
			NRF_LOG_INFO("WRITE_HX711_STATE: channels:%u/0x%02X, samples: %u", prot->param.status.statusflag, prot->param.status.statusflag, prot->param.status.interval);
			break;

        //-----------------------------------------------------------------------------
		case WRITE_DS18B20_CONVERSION:
			break;

        //-----------------------------------------------------------------------------
		case READ_DS18B20_CONFIG:
			break;

        //-----------------------------------------------------------------------------
        case WRITE_HX711_CONVERSION:
        case READ_HX711_CONVERSION:
		{
			uint8_t i, channel, chOffset = 0;
			const char * channelStrings[3] = {"CH_A_GAIN128", "CH_B_GAIN32", "CH_A_GAIN64"};
			HX711_CONV_s * hx = &prot->param.meas_result.result.hx711;
			if(prot->command == WRITE_HX711_CONVERSION && hx->channel == 0)
			{
				NRF_LOG_INFO("WRITE_HX711_CONVERSION");
			}
			else
			{
				for(i=0; i< 3; i++)
				{
					channel = hx->channel & (1<<i);
					if(channel)
					{
						NRF_LOG_INFO("HX711_CONVERSION result %s: %i", HX711_channel_str(channel), hx->value[chOffset++]);
					}
				}
			}
			
			break;
		}

        //-----------------------------------------------------------------------------
		case READ_AUDIO_ADC_CONFIG:
			break;

        //-----------------------------------------------------------------------------
        case READ_AUDIO_ADC_CONVERSION:
			break;

        //-----------------------------------------------------------------------------
        case START_AUDIO_ADC_CONVERSION:
            NRF_LOG_INFO("START_AUDIO_ADC_CONVERSION");
			break;

        //-----------------------------------------------------------------------------
		case READ_ATECC_READ_ID:
			break;

        //-----------------------------------------------------------------------------
        case READ_ATECC_I2C:
			break;

        //-----------------------------------------------------------------------------
        case READ_BUZZER_STATE:
			break;

        //-----------------------------------------------------------------------------
		case WRITE_BUZZER_DEFAULT_TUNE:
			NRF_LOG_INFO("Buzzer default tune: %u", prot->param.buzz.index);
			break;

        //-----------------------------------------------------------------------------
		case WRITE_BUZZER_CUSTOM_TUNE:
		{
			BUZZER_s * buzz = &prot->param.buzz;
			NRF_LOG_INFO("Buzzer tune: dutycycle=%u, freq=%u00 Hz, Off-time=%u, On-time=%u, repeat=%u",
							buzz->index,
							buzz->freq_100Hz,
							buzz->off_time_ms,
							buzz->on_time_ms,
							buzz->repeatCount);
			break;
		}

        //-----------------------------------------------------------------------------
        case READ_SQ_MIN_STATE:
			break;

		case WRITE_LORAWAN_TRANSMIT:
        case READ_LORAWAN_DEVEUI:
        case READ_LORAWAN_APPEUI:
        case READ_LORAWAN_APPKEY:
        case WRITE_LORAWAN_DEVEUI:
        case WRITE_LORAWAN_APPEUI:
		case READ_PINCODE:
		case WRITE_PINCODE:
        case WRITE_LORAWAN_APPKEY:
		{
			LORAWAN_s * key = &prot->param.lorawan_key;
			NRF_LOG_INFO("%s %s, lenght: %u", 
				((prot->command & BEEP_WRITE) ? "Write" : "Read"),
                beep_protocol_cmd_strget(prot->command),
				key->lenght);
			if(key->lenght != 0)
			{
				NRF_LOG_HEXDUMP_INFO(key->data, key->lenght);
			}
			break;
		}
			

        //-----------------------------------------------------------------------------
        case READ_LORAWAN_STATE:
        case WRITE_LORAWAN_STATE:
			NRF_LOG_INFO("LoRaWAN State: valid keys=%s, enabled=%s, joined=%s, dutycycleOn=%s, ADRenabled=%s, status=0x%02X", 
				bool_to_str(prot->param.status.statusflag & BITMASK_VALID_KEYS),
				bool_to_str(prot->param.status.statusflag & BITMASK_ENABLE),
                bool_to_str(prot->param.status.statusflag & BITMASK_JOINED),
                bool_to_str(prot->param.status.statusflag & BITMASK_DUTYCYCLE),
                bool_to_str(prot->param.status.statusflag & BITMASK_ADR),
				prot->param.status.statusflag);
			break;


		//-----------------------------------------------------------------------------
        case ALARM_CONFIG_READ:
            break;

        //-----------------------------------------------------------------------------
        case ALARM_CONFIG_WRITE:
            break;

        //-----------------------------------------------------------------------------
        case READ_CID_nRF_FLASH:
			break;

        //-----------------------------------------------------------------------------
        case READ_nRF_ADC_CONFIG:
			break;

        //-----------------------------------------------------------------------------
        case READ_nRF_ADC_CONVERSION:
			break;

        //-----------------------------------------------------------------------------
        case READ_APPLICATION_STATE:
			break;

        //-----------------------------------------------------------------------------
        case READ_APPLICATION_CONFIG:
        case WRITE_APPLICATION_CONFIG:
            #if BEEP_PROTOCOL_LOGGING
                NRF_LOG_INFO("Main application interval: %u min, ratio: %u N", prot->param.status.interval, prot->param.status.statusflag);
            #endif
			break;
			
		//-----------------------------------------------------------------------------
		default:
			return NRF_ERROR_NOT_FOUND;
			break;
	}
    return NRF_ERROR_INVALID_LENGTH;
}





uint32_t beep_protocol_decode(BEEP_protocol_s * prot, uint8_t * data, uint8_t msg_lenght, uint8_t * offset)
{
	uint32_t err, ret = NRF_ERROR_NOT_SUPPORTED;
	uint8_t cmdLenght = 0;

	if(prot == NULL || data == NULL || msg_lenght == 0)
	{
		return NRF_ERROR_INVALID_PARAM;
	}

	memset(prot, 0, sizeof(BEEP_protocol_s));
	
	// Retrieve the command.
	prot->command	= (BEEP_CID) data[0];

	// Check if the message lenght is ok.
    err = beep_protocol_check_lenght(prot->command, msg_lenght, data, &cmdLenght);
	if(err != NRF_SUCCESS)
	{
		NRF_LOG_INFO("invalid command lenght: %u/0x%02X, %u bytes, ret: 0x%04x", prot->command, prot->command, msg_lenght, err);
		return err;
	}
	else
	{
        #if BEEP_PROTOCOL_LOGGING
            NRF_LOG_INFO("valid command lenght: %u/0x%02X, %u bytes", prot->command, prot->command, msg_lenght);
        #endif
	}
	*offset += cmdLenght;
	
	switch(prot->command)
	{
		//-----------------------------------------------------------------------------
		case RESPONSE:
			return NRF_ERROR_NOT_SUPPORTED;
			break;

        //----------------------------------READ COMMANDS-------------------------------------------
		case READ_APPLICATION_CONFIG:
        case READ_ATECC_READ_ID:
        case READ_HX711_CONVERSION:
		case READ_HX711_STATE:
        case READ_LORAWAN_DEVEUI:
        case READ_LORAWAN_APPEUI:
        case READ_LORAWAN_APPKEY:
		case READ_DS18B20_CONVERSION:
		case READ_LORAWAN_STATE:
        case READ_FIRMWARE_VERSION:
		case READ_HARDWARE_VERSION:
        case READ_nRF_ADC_CONVERSION:
        case WRITE_nRF_ADC_CONVERSION:
		case READ_PINCODE:
        case SIZE_MX_FLASH:
        case READ_AUDIO_ADC_CONVERSION:
        case READ_BOOT_COUNT:
        case START_AUDIO_ADC_CONVERSION:
        case READ_AUDIO_ADC_CONFIG:
        case ALARM_STATUS_READ:
		case BME280_CONVERSION_READ:
		case BME280_CONVERSION_START:
			ret = NRF_SUCCESS;
			break;

		case WRITE_DS18B20_CONVERSION:
		{
			prot->param.meas_result.result.ds18B20.devices = data[1];
            ret = NRF_SUCCESS;
			break;
		}

		//-----------------------------------------------------------------------------
        case READ_MX_FLASH:
        {
            prot->param.size    = uint32_big_decode(&data[1]);
            ret = NRF_SUCCESS;
            break;
        }

		//------------------------------STATUS COMMANDS--------------------------------
        case ERASE_MX_FLASH:
		case READ_DS18B20_STATE:
		case WRITE_LORAWAN_STATE:
		{
			STATUS_s * sta = &prot->param.status;
			sta->statusflag = data[1];
            ret = NRF_SUCCESS;
			break;
		}	

		//-----------------------------------------------------------------------------
		case WRITE_LORAWAN_TRANSMIT:
		{
			LORAWAN_s * key = &prot->param.lorawan_key;
			uint8_t i;
            key->lenght = data[1];

			// Check if the expected message lenght matches with the received lenght
			if(msg_lenght < (key->lenght + 2))
			{
				return NRF_ERROR_INVALID_LENGTH;
			}

			for(i = 0 ; i < key->lenght ; i++)
			{
				key->data[i] = data[i+2];
			}
            ret = NRF_SUCCESS;
			break;
		}

		//-----------------------------------------------------------------------------
        case WRITE_LORAWAN_DEVEUI:
        case WRITE_LORAWAN_APPEUI:
        case WRITE_LORAWAN_APPKEY:
		{
            LORAWAN_s * key = &prot->param.lorawan_key;
			uint8_t i;
            key->lenght = (prot->command == WRITE_LORAWAN_APPKEY ? 16 : 8);
			for(i = 0 ; i < key->lenght ; i++)
			{
				key->data[i] = data[i+1];
			}
            ret = NRF_SUCCESS;
			break;
		}	
			
		//-----------------------------------------------------------------------------
		case WRITE_PINCODE:
		{
            LORAWAN_s * key = &prot->param.lorawan_key;
			uint8_t i;
            key->lenght = data[1];
			for(i = 0 ; i < key->lenght ; i++)
			{
				key->data[i] = data[i+2];

				// Check if the pin code consists of only ASCII numbers
				if(key->data[i] < '0' || key->data[i] > '9')
				{
					return NRF_ERROR_INVALID_DATA;
				}
			}
            ret = NRF_SUCCESS;
			break;
		}

        //-----------------------------------------------------------------------------
        case WRITE_APPLICATION_CONFIG:
		{
			prot->param.status.statusflag = data[1];
            prot->param.status.interval   = uint16_big_decode(&data[2]);
			if(prot->param.status.interval == 0 || prot->param.status.interval > SAMPLE_INTERVAL_MIN_MAX)
			{
				return NRF_ERROR_INVALID_PARAM;
			}
            ret = NRF_SUCCESS;
			break;
		}

		//-----------------------------------------------------------------------------
		case WRITE_DS18B20_STATE:
		{
			prot->param.status.statusflag = data[1];
            ret = NRF_SUCCESS;
			break;
		}

        //-----------------------------------------------------------------------------
        case WRITE_HX711_CONVERSION:
			prot->param.meas_result.result.hx711.channel = data[1];
            prot->param.meas_result.result.hx711.samples = data[2];
            ret = NRF_SUCCESS;
			break;

        //-----------------------------------------------------------------------------
		case WRITE_HX711_STATE:
			prot->param.status.statusflag	= data[1];
            prot->param.status.interval		= data[2];
            ret = NRF_SUCCESS;
			break;

        //-----------------------------------------------------------------------------
        case WRITE_AUDIO_ADC_CONFIG:
        {
            AUDIO_CONFIG_s * conf = &prot->param.audio_config;
            conf->channel       = data[1];
            conf->gain          = (data[2] & 0x7F);
            conf->min6dB        = (data[2] & 0x80) ? true : false;
            conf->volume        = data[3];
            conf->fft_count     = data[4];
            conf->fft_start     = data[5];
            conf->fft_stop      = data[6];
            ret = NRF_SUCCESS;
            break;
        }

        //-----------------------------------------------------------------------------
		case WRITE_BUZZER_CUSTOM_TUNE:
		{
			BUZZER_s * buzz = &prot->param.buzz;
            buzz->index			= data[1];
			buzz->freq_100Hz	= data[2];
			buzz->off_time_ms	= uint16_big_decode(&data[3]);
			buzz->on_time_ms	= uint16_big_decode(&data[5]);
			buzz->repeatCount	= data[7];

			// Check inputs
			if(buzz->index == 0 || buzz->freq_100Hz == 0 || buzz->off_time_ms == 0 || buzz->on_time_ms == 0 || buzz->repeatCount == 0)
			{
				return NRF_ERROR_INVALID_PARAM;
			}
            ret = NRF_SUCCESS;
			break;
		}

        //-----------------------------------------------------------------------------
		case WRITE_BUZZER_DEFAULT_TUNE:
		{
			BUZZER_s * buzz = &prot->param.buzz;
            buzz->index		= data[1];
            ret				= NRF_SUCCESS;
			break;
		}

        //-----------------------------------------------------------------------------
        case ALARM_CONFIG_WRITE:
        {
            ALARM_CONFIG_s * config = &prot->param.alarm;
            config->type = data[1];
            switch(config->type)
            {
                case DS18B20:
                    config->thr.ds.Max  = (int16_t)uint16_big_decode(&data[2]);
                    config->thr.ds.Min  = (int16_t)uint16_big_decode(&data[4]);
                    config->thr.ds.Diff = (int16_t)uint16_big_decode(&data[6]);
                    break;

                case nRF_ADC:
                    config->thr.supply.Max  = uint16_big_decode(&data[2]);
                    config->thr.supply.Min  = uint16_big_decode(&data[4]);
                    config->thr.supply.Diff = uint16_big_decode(&data[6]);
                    break;

                case HX711:
                    config->thr.hx.Max  = (int32_t) uint32_big_decode(&data[2]);
                    config->thr.hx.Min  = (int32_t) uint32_big_decode(&data[6]);
                    config->thr.hx.Diff =           uint32_big_decode(&data[10]);
                    break;

                case BME280:
				{
					BME_ALARM_s * BMEthr  = &prot->param.alarm.thr.bme;

					// Temp
					BMEthr->Temp_Max		= (int16_t) uint16_big_decode(&data[2]);
                    BMEthr->Temp_Min		= (int16_t) uint16_big_decode(&data[4]);
                    BMEthr->Temp_Diff		= uint16_big_decode(&data[6]);

					// Humidity
                    BMEthr->humidity_Max	= uint16_big_decode(&data[8]);
                    BMEthr->humidity_Min	= uint16_big_decode(&data[10]);
                    BMEthr->humidity_Diff	= uint16_big_decode(&data[12]);

					// pressure
                    BMEthr->press_Max		= uint16_big_decode(&data[14]);
                    BMEthr->press_Min		= uint16_big_decode(&data[16]);
                    BMEthr->press_Diff		= uint16_big_decode(&data[18]);
					break;
				}

                case AUDIO_ADC:
                default:
                    return NRF_ERROR_INVALID_PARAM;
                    break;
            }
            ret = NRF_SUCCESS;
            break;
        }

        //-----------------------------------------------------------------------------
        case ALARM_CONFIG_READ:
        {
            prot->param.alarm.type  = data[1];
            ret                     = NRF_SUCCESS;
            break;
        }

        //-----------------------------------------------------------------------------
		case READ_DS18B20_CONFIG:
			break;

        //-----------------------------------------------------------------------------
        case READ_ATECC_I2C:
			break;

        //-----------------------------------------------------------------------------
        case READ_BUZZER_STATE:
			break;

        //-----------------------------------------------------------------------------
        case READ_SQ_MIN_STATE:
			break;

        //-----------------------------------------------------------------------------
        case READ_CID_nRF_FLASH:
			break;

        //-----------------------------------------------------------------------------
        case READ_nRF_ADC_CONFIG:
			break;

        //-----------------------------------------------------------------------------
        case READ_APPLICATION_STATE:
			break;
			
		//-----------------------------------------------------------------------------
		default:
			return NRF_ERROR_NOT_FOUND;
			break;
	}

	if(ret == NRF_SUCCESS)
	{
        #if BEEP_PROTOCOL_LOGGING
            beep_protocol_log(prot);
        #endif
	}
	else
	{
        #if BEEP_PROTOCOL_LOGGING
            NRF_LOG_INFO("beep_protocol_decode error: %u/0x%04X", ret, ret);
        #endif
	}

    return ret;
}


/*
 */
uint32_t beep_protocol_encode(bool cmdPrepend, BEEP_protocol_s * prot, uint8_t * pBuffer, uint8_t * msg_lenght, uint8_t maxBufferLenght)
{
    uint8_t data[50] = {0};
	uint8_t offset = 0;


	if(prot == NULL || pBuffer == NULL || msg_lenght == NULL)
	{
		return NRF_ERROR_INVALID_PARAM;
	}

	if(cmdPrepend)
	{
		data[0] = prot->command;
		offset	= 1;
	}

    #if BEEP_PROTOCOL_LOGGING
        NRF_LOG_INFO("Encoding cmd: %u/0x%02X", prot->command, prot->command);
    #endif

	switch(prot->command)
	{
		//-----------------------------------------------------------------------------
		case RESPONSE:
		{
			RESPONSE_s * err = &prot->param.error;

			data[offset] = err->ErrorCmd;
            offset  += 1;
            offset  += uint32_big_encode(err->errorCode, &data[offset]);
			break;
		}

		//-----------------------------------------------------------------------------
		case READ_FIRMWARE_VERSION:
		{
			FH_VERSION_s * version = &prot->param.version;
			offset  += uint16_big_encode(version->major, &data[offset]);
            offset  += uint16_big_encode(version->minor, &data[offset]);
            offset  += uint16_big_encode(version->sub,	 &data[offset]);
			break;
		}

        //-----------------------------------------------------------------------------
		case READ_HARDWARE_VERSION:
		{
			FH_VERSION_s * version = &prot->param.version;
			offset  += uint16_big_encode(version->major, &data[offset]);
            offset  += uint16_big_encode(version->minor, &data[offset]);
            offset  += uint32_big_encode(version->id,	 &data[offset]);
			break;
		}

        //-----------------------------------------------------------------------------
		case READ_DS18B20_STATE:
		{
            offset += uint8_big_encode(prot->param.status.statusflag, &data[offset]);
			break;
		}

        //-----------------------------------------------------------------------------
		case READ_APPLICATION_CONFIG:
		{
            offset += uint8_big_encode(prot->param.status.statusflag, &data[offset]);
            offset += uint16_big_encode(prot->param.status.interval,  &data[offset]);
			break;
		}

		//-----------------------------------------------------------------------------
		case WRITE_DS18B20_CONVERSION:
		{
        	uint8_t i = 0;
			DS18B20_RESULTS_s * ds = &prot->param.meas_result.result.ds18B20;
            offset += uint8_big_encode(ds->devices, &data[offset]);
			offset += uint16_big_encode(ds->temperatures[ds->devices], &data[offset]);
			break;
		}

        //-----------------------------------------------------------------------------
		case READ_DS18B20_CONVERSION:
		{
        	uint8_t i = 0;
			DS18B20_RESULTS_s * ds = &prot->param.meas_result.result.ds18B20;
            offset += uint8_big_encode(ds->devices, &data[offset]);
			for(i=0; i<ds->devices; i++)
			{
				offset  += uint16_big_encode(ds->temperatures[i], &data[offset]);
			}
			break;
		}

        //-----------------------------------------------------------------------------
		case READ_HX711_CONVERSION:
		case WRITE_HX711_CONVERSION:
		{
			uint8_t i, chOffset = 0;
			HX711_CONV_s * conv = &prot->param.meas_result.result.hx711;

            offset += uint8_big_encode(conv->channel, &data[offset]);
			for(i=0; i < 3 ; i++)
			{
				if(conv->channel & (1<<i))
				{
					offset += uint24_big_encode(conv->value[chOffset++], &data[offset]);
				}
			}  
			break;
		}


        //-----------------------------------------------------------------------------
		case READ_ATECC_READ_ID:
		{
			uint8_t i = 0;
			memcpy(&data[offset], &prot->param.atecc_id.ROM, ATECC_ID_LENGHT);
            offset += ATECC_ID_LENGHT;
			break;
		}

        //-----------------------------------------------------------------------------
		case READ_HX711_STATE:
		case WRITE_HX711_STATE:
		{
            offset += uint8_big_encode(prot->param.status.statusflag,			&data[offset]);
            offset += uint8_big_encode((uint8_t)prot->param.status.interval,	&data[offset]);
			break;
		}

        //-----------------------------------------------------------------------------
        case SIZE_MX_FLASH:
            offset      += uint32_big_encode(prot->param.size,	&data[offset]);
            break;

        //-----------------------------------------------------------------------------
        case READ_AUDIO_ADC_CONFIG:
        case WRITE_AUDIO_ADC_CONFIG:
        {
            AUDIO_CONFIG_s * conf = &prot->param.audio_config;
            uint8_t reg = (((uint8_t)conf->gain) & 0x7F) | (conf->min6dB ? 0x80 : 0x00);
            offset += uint8_big_encode(conf->channel,           &data[offset]);
            offset += uint8_big_encode(reg,                     &data[offset]);
            offset += uint8_big_encode(conf->volume,            &data[offset]);
            offset += uint8_big_encode(conf->fft_count,         &data[offset]);
            offset += uint8_big_encode(conf->fft_start,         &data[offset]);
            offset += uint8_big_encode(conf->fft_stop,          &data[offset]);
            break;
        }

        //-----------------------------------------------------------------------------
        case READ_AUDIO_ADC_CONVERSION:
        {
            uint8_t i;
            FFT_RESULTS * result = &prot->param.meas_result.result.fft;

            offset  += uint8_big_encode(result->bins, &data[offset]);
            offset  += uint8_big_encode(result->start, &data[offset]);
            offset  += uint8_big_encode(result->stop, &data[offset]);

            for(i=0; i<result->bins; i++)
            {
                offset  += uint16_big_encode(result->values[i], &data[offset]);
            }
			break;
        }

        //-----------------------------------------------------------------------------
        case READ_LORAWAN_STATE:
			offset  += uint8_big_encode(prot->param.status.statusflag, &data[offset]);
			break;

        //-----------------------------------------------------------------------------
        case READ_LORAWAN_DEVEUI:
        case READ_LORAWAN_APPEUI:
        case READ_LORAWAN_APPKEY:
		{
			prot->param.lorawan_key.lenght = (prot->command == READ_LORAWAN_APPKEY ? 16 : 8); 
			memcpy(&data[offset], prot->param.lorawan_key.data, prot->param.lorawan_key.lenght);
			offset += prot->param.lorawan_key.lenght;
			break;
		}

        //-----------------------------------------------------------------------------
        case READ_PINCODE:
		{
			data[offset++] = prot->param.lorawan_key.lenght;
			memcpy(&data[offset], prot->param.lorawan_key.data, prot->param.lorawan_key.lenght);
			offset += prot->param.lorawan_key.lenght;
			break;
		}

		//-----------------------------------------------------------------------------
        case WRITE_nRF_ADC_CONVERSION:
		case READ_nRF_ADC_CONVERSION:
		{
            ADC_s * saadc = &prot->param.meas_result.result.saadc;
			offset  += uint16_big_encode(saadc->battVoltage_mV, &data[offset]);
			offset  += uint16_big_encode(saadc->vccVoltage_mV,  &data[offset]);
			offset  += uint8_big_encode(saadc->battPercentage,	&data[offset]);
			break;
		}


        //-----------------------------------------------------------------------------
        case READ_BOOT_COUNT:
        case WRITE_BOOT_COUNT:
        {    
            offset += uint32_big_encode(prot->param.status.interval, &data[offset]);
            break;
        }

        //-----------------------------------------------------------------------------
        case ALARM_CONFIG_READ:
        {
            ALARM_CONFIG_s * config = &prot->param.alarm;
            offset  += uint8_big_encode(config->type, &data[offset]);
            switch(config->type)
            {
                case DS18B20:
                    offset += uint16_big_encode((uint16_t) config->thr.ds.Max,  &data[offset]);
                    offset += uint16_big_encode((uint16_t) config->thr.ds.Min,  &data[offset]);
                    offset += uint16_big_encode((uint16_t) config->thr.ds.Diff, &data[offset]);
                    break;

                case nRF_ADC:
                    offset += uint16_big_encode(config->thr.supply.Max,  &data[offset]);
                    offset += uint16_big_encode(config->thr.supply.Min,  &data[offset]);
                    offset += uint16_big_encode(config->thr.supply.Diff, &data[offset]);
                    break;

                case HX711:
                    offset += uint32_big_encode((uint32_t)config->thr.hx.Max,  &data[offset]);
                    offset += uint32_big_encode((uint32_t)config->thr.hx.Min,  &data[offset]);
                    offset += uint32_big_encode((uint32_t)config->thr.hx.Diff, &data[offset]);
                    break;

                case BME280:
				{
					BME_ALARM_s * BMEthr  = &prot->param.alarm.thr.bme;

					// Temp
                    offset += uint16_big_encode((uint16_t) BMEthr->Temp_Max,	&data[offset]);
                    offset += uint16_big_encode((uint16_t) BMEthr->Temp_Min,	&data[offset]);
                    offset += uint16_big_encode(BMEthr->Temp_Diff,				&data[offset]);

					// Humidity
                    offset += uint16_big_encode(BMEthr->humidity_Max,			&data[offset]);
                    offset += uint16_big_encode(BMEthr->humidity_Min,			&data[offset]);
                    offset += uint16_big_encode(BMEthr->humidity_Diff,			&data[offset]);

					// Pressure
                    offset += uint16_big_encode(BMEthr->press_Max,				&data[offset]);
                    offset += uint16_big_encode(BMEthr->press_Min,				&data[offset]);
                    offset += uint16_big_encode(BMEthr->press_Diff,				&data[offset]);
					break;
				}

                case AUDIO_ADC:
                default:
                    *msg_lenght += offset;
                    return NRF_ERROR_INVALID_PARAM;
                    break;
            }
            break;
        }

        //-----------------------------------------------------------------------------
        case ALARM_STATUS_READ:
        {
            STATUS_s * stat = &prot->param.status;
            offset  += uint8_big_encode(stat->statusflag, &data[offset]);
            break;
        }

        //-----------------------------------------------------------------------------
		case BME280_CONVERSION_READ:
		{
			BME280_RESULT_s * bme = &prot->param.meas_result.result.bme280;
			offset  += uint16_big_encode((uint16_t) bme->temperature,	&data[offset]);
			offset  += uint16_big_encode(bme->humidity,					&data[offset]);
			offset  += uint16_big_encode(bme->airPressure,				&data[offset]);		
			break;
		}
        
        //-----------------------------------------------------------------------------
        case READ_CID_nRF_FLASH:
			break;

        //-----------------------------------------------------------------------------
        case READ_nRF_ADC_CONFIG:
			break;

        //-----------------------------------------------------------------------------
        case READ_APPLICATION_STATE:
			break;

        //-----------------------------------------------------------------------------
        case START_AUDIO_ADC_CONVERSION:
			break;

        //-----------------------------------------------------------------------------
        case READ_ATECC_I2C:
			break;

        //-----------------------------------------------------------------------------
        case READ_BUZZER_STATE:
			break;

        //-----------------------------------------------------------------------------
        case READ_SQ_MIN_STATE:
			break;

        //-----------------------------------------------------------------------------
		case READ_DS18B20_CONFIG:
			break;
			
		//-----------------------------------------------------------------------------
		default:
			return NRF_ERROR_NOT_FOUND;
			break;
	}	

    if(*msg_lenght + offset >= maxBufferLenght)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    // Append the encoded data and increment the lenght pointer
    memcpy(&pBuffer[(*msg_lenght)], data, offset);
    (*msg_lenght) += offset;

    #if BEEP_PROTOCOL_LOGGING
        beep_protocol_log(prot);
    #endif
	return NRF_SUCCESS;
}




const char * HX711_channel_str(HX711_GAIN channel)
{
	const char * channelStrings[] = {
		"CH_A_GAIN128",
        "CH_B_GAIN32",
        "CH_A_GAIN64",
        "CH_ERROR",
	};
	switch(channel)
	{
		case CH_A_GAIN128:
			return channelStrings[0];
			break;
        case CH_B_GAIN32:
			return channelStrings[1];	
			break;
        case CH_A_GAIN64:	
			return channelStrings[2];
			break;
		default:
			return channelStrings[3];
			break;
	}
    return  channelStrings[3];
}



