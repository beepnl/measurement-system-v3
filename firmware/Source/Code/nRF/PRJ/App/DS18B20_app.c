#include "OWI.h"
#define NRF_LOG_MODULE_NAME DS_APP
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
#include "DS18B20.h"
#include "DS18B20_app.h"
#include "nrf_delay.h"
#include "utilities.h"
#include "app_timer.h"
#include "gpio-board.h"
#include "power_app.h"
#include "ble_setup.h"
#include "nvm_fs.h"
#include "timeslot.h"

APP_TIMER_DEF(timer);
DS18B20_APP_s app;
static bool reset = false;

static DS_RESOLUTION uint_to_resolution(uint8_t res)
{
	DS_RESOLUTION ret;
	switch((res & 0x0E) >> 1)
	{
        case 1:		ret = TEMP_9BIT_RESOLUTION;			break;
        case 2:		ret = TEMP_10BIT_RESOLUTION;		break;
        case 3:		ret = TEMP_11BIT_RESOLUTION;		break;
        case 4:		ret = TEMP_12BIT_RESOLUTION;		break;
        default:	ret = 0;							break;
			
	}
    return ret;
}

static uint8_t resolution_to_uint(DS_RESOLUTION res)
{
	uint8_t ret = 0;

    switch(res)
	{
        case TEMP_9BIT_RESOLUTION:		ret = 1; break;
        case TEMP_10BIT_RESOLUTION:		ret = 2; break;
        case TEMP_11BIT_RESOLUTION:		ret = 3; break;
        case TEMP_12BIT_RESOLUTION:		ret = 4; break;
		default:						ret = 0; break;
			
	}
    return (ret << 1);
}

static void DS18B20_App_convert(CONTROL_SOURCE source, MEASUREMENT_RESULT_s * conv)
{
	uint16_t temp, i;

	if(conv == NULL)
	{
		return;
	}

	memset(conv, 0, sizeof(MEASUREMENT_RESULT_s));
					
	conv->type						= DS18B20;
	conv->source					= source;
	conv->result.ds18B20.devices	= app.device_count;

	// Add devices temperature to the list of results
	for(i = 0; i < conv->result.ds18B20.devices; i++)
	{
		temp = (uint16_t)(app.dev[i].temp * 100.0);
		conv->result.ds18B20.temperatures[i] = (int16_t) temp;
	}
}


void DS18B20_App_getTemp(MEASUREMENT_RESULT_s * result)
{
	DS18B20_App_convert(INTERNAL_SOURCE, result);
}


void DS18B20_App_NewState(DS18B20_STATES_s newState)
{
	app.state = newState;

	#if 0
		const char * stateStrings[] =
		{
			"STOP",
			"DETECT",
			"SEARCH",
			"READ_CONFIG",
			"SET_CONFIG",
			"IDLE",
            "START_TEMP_CONV",
			"WAIT_TEMP_CONV_COMPLETE",
			"GET_TEMP_CONV_RESULT",
			"UNKNOWN State",
		};
		NRF_LOG_INFO("New DS18B20 App state: 0x%04X = %s", (uint16_t) newState, ((newState >= DS_UNKNOWN_STATE) ? stateStrings[DS_UNKNOWN_STATE] : stateStrings[newState]));
		NRF_LOG_FLUSH();
	#endif
}



uint32_t  DS18B20_App_getStatus(uint8_t * ret_status, uint16_t * interval)
{
	uint8_t status = 0;
	uint8_t resolution = 0;

	if(ret_status == NULL || interval == NULL)
	{
		return NRF_ERROR_NULL;
	}

	status |= (app.on)	? (1<<0) : 0;
	status |= resolution_to_uint(app.resolution);

	*ret_status	= status;
	*interval	= app.interval;
	return NRF_SUCCESS;
}


uint32_t DS18B20_App_setStatus(uint8_t status, uint16_t interval)
{
    BEEP_protocol_s conf;
    DS_RESOLUTION newResSet = 0;

	newResSet = uint_to_resolution(status);
	if(newResSet == 0)
	{
		return NRF_ERROR_INVALID_PARAM;
	}

    app.on			= (status & (1<<0)) ? true : false;

    if(newResSet != app.resolution && app.on || app.on)
	{
		// Reset the statemachine
        reset			= true;
        app.resolution	= newResSet; 
	}

	if(!app.on)
	{
		app_timer_stop(timer);
	}

    conf.command = READ_DS18B20_STATE;
    DS18B20_App_getStatus(&conf.param.status.statusflag, &conf.param.status.interval);
	nvm_fds_eeprom_set(&conf);
	
	return NRF_SUCCESS;
}

static void DS18B20_App_timerCallback(void * p_context)
{
	UNUSED_VARIABLE(p_context);

	switch(app.state)
	{
		default:
		case DS_IDLE:
			app.source		= INTERNAL_SOURCE;
			app.convIndex	= 0xFF;
			DS18B20_App_NewState(DS_START_TEMP_CONV);
			break;

        case DS_WAIT_TEMP_CONV_COMPLETE:
			DS18B20_App_NewState(DS_GET_TEMP_CONV_RESULT);
			break;
	}
}





void DS18B20_clearIndex(uint8_t index)
{
	if(index >= DS18B20_SENSORS_MAX)
	{
		// Clear all indexes
		uint8_t i;
		for(i=0; i < DS18B20_SENSORS_MAX; i++)
		{
			memset(&app.dev[i], 0, sizeof(sDS18B20));
		}
	}
	else
	{
		// Clear a specific index
		memset(&app.dev[index], 0, sizeof(sDS18B20));
	}
}

void DS18B20_clearErrorCount(uint8_t index)
{
	if(index >= DS18B20_SENSORS_MAX)
	{
		uint8_t i;
		for(i=0; i<DS18B20_SENSORS_MAX; i++)
		{
			app.dev[i].errorCount = 0;
		}
		return;
	}
	
	app.dev[index].errorCount = 0;
}

void DS18B20_incErrorCount(uint8_t index, DS_ERROR_CODES status)
{
	if(index >= DS18B20_SENSORS_MAX)
	{
		return;
	}
	
	app.dev[index].errorCount++;
	if(app.dev[index].errorCount > DS18B20_ERROR_COUNT_MAX)
	{
		app.dev[index].temp = -100.0; // Set the temperature of the misbehaving temperature sensor to -100C
		#if DS18B20_DISABLE_SENSOR_ON_ERROR_MAX
			// Disable the device after to many errors.
			app.dev[index].enabled = false;

			// decrement the device count.
			if(app.device_count != 0)
			{
				app.device_count--;
			}
		#else

		#endif
	}

	#if DS18B20_LOG_ENABLE
		const char * errorStrings[] =
		{
			"DS_SUCCESFULL",			//  0
			"DS_INVALID_PARAM",			//  1
			"OWI_PULL_UP_LOW_ERROR",	//  2
			"OWI_PULL_UP_HIGH_ERROR",	//  3
			"DS_NODEVICE",				//  4
			"DS_ROM_CRC_NOK",			//  5
			"DS_CRC_NOK",				//  6
			"DS_TIMEOUT",				//  7
			"DS_UNKNOWN_ERROR",			//  8
		};
		
		NRF_LOG_INFO(" DS18B20[%u] error: 0x%02X = %s, count: %u, status: %s", 
			index, 
			status, 
			((status >= DS_UNKNOWN_ERROR) ? errorStrings[DS_UNKNOWN_ERROR] : errorStrings[status]),
			app.dev[index].errorCount,
			NRF_LOG_PUSH((app.dev[index].enabled) ? "On" : "Off"));
	#endif

	// Retry
	return;
}

static void DS18B20_statemachineReset(void)
{
	reset				= false;
	app.device_count	= 0;
	app.state			= DS_DETECT;
	app.conversionCount	= 0;

	// Clear all parameters.
	DS18B20_clearIndex(DS18B20_SENSORS_MAX);

	// Clear the power App lock bit
    powerApp_Enable(false, PWR_DS18B20);
}



/*
 * Returns false when in the STOP, IDLE or WAIT_TEMP_CONV_COMPLETE state. Returns true when in other states
 */
bool DS18B20_App_busy(void)
{
	return (app.state == DS_STOP || app.state == DS_IDLE || app.state == DS_WAIT_TEMP_CONV_COMPLETE) ? false : true;
}


static uint32_t DS18B20_App_index_conversion(void)
{
	uint32_t ret;
    uint8_t * ROM = NULL;

	// When a valid index value is set, convert just a single sensor.
	if(app.convIndex < DS18B20_SENSORS_MAX && app.dev[app.convIndex].enabled)
	{
		ROM				= app.dev[app.convIndex].ROM;
        app.read_index	= app.convIndex;
	}
	else
	{
		app.convIndex	= 0xFF;
        app.read_index	= 0;
	}


	ret = DS18B20_StartTempConversion(ROM);
	return ret;
}



void DS18B20_App_while(void)
{	
	if(reset)
	{		
        DS18B20_statemachineReset();
	}


	switch(app.state)
	{
		default:
		case DS_STOP:
			// Dead state, disable the lock on the power switch and boost converter if set.
            if(powerApp_getEnabled(PWR_DS18B20))
			{
				powerApp_Enable(false, PWR_DS18B20);
			}
			break;
	
		case DS_DETECT:
		{
			bool pwr = powerApp_getEnabled(PWR_DS18B20);

			// Detect whether devices are present on the one-wire interface
			if(!pwr)
			{
				powerApp_Enable(true, PWR_DS18B20);
				nrf_delay_ms(10);
				return;
			}
			else if(OWI_presenceDetect() && pwr)
			{
				// Clear the search history
				DS18B20_clearSearch();
				DS18B20_App_NewState(DS_SEARCH);
			}
			else
			{
				DS18B20_App_NewState(DS_STOP);
                NRF_LOG_INFO("No presence detected on the OWI bus, go to STOP mode.");
			}
			break;
		}

		case DS_SEARCH:
		{
			bool search_result;
			uint8_t ROM[DS18B20_ROM_LENGHT] = {0};

			// Search for all connected slave devices and increment the count and copy the ROM if one is found.
            search_result = DS18B20_Search(ROM);
			if(search_result)
			{
				// Copy the received ROM
				memcpy(app.dev[app.device_count].ROM, ROM, DS18B20_ROM_LENGHT);
                app.dev[app.device_count].enabled = true;
				app.device_count++;
			}

			// When no new slave devices have been detected return to DETECT.
			if(!search_result || app.device_count >= DS18B20_SENSORS_MAX)
			{
				DS18B20_App_NewState( (app.device_count == 0) ? DS_STOP : DS_READ_CONFIG);
				app.read_index = 0;
                #if DS18B20_LOG_ENABLE
                    NRF_LOG_INFO("%u One wire sensors found", app.device_count);
                #endif
			}
            break;
		}
			

		case DS_READ_CONFIG:
		{
			uint8_t scratchPad[DS18B20_SCRATCHPAD_LENGHT] = {0};
			DS_ERROR_CODES status;
            sDS18B20 * dev;

			// Check the readCount
			if(app.read_index >= DS18B20_SENSORS_MAX)
			{
				app.read_index = DS18B20_SENSORS_MAX - 1;
			}	
            dev = &app.dev[app.read_index];

			// Check if the device is enabled
			if(dev->enabled && dev->errorCount < DS18B20_ERROR_COUNT_MAX)
			{
				// Read each device's scratchpad memory individually
				status =  DS18B20_ReadScratchPad(dev->ROM, scratchPad);

				// When the read operation is succesfull retrieve the parameters.
				if(status == DS_SUCCESFULL)
				{
					// Calculate the temperature
					uint16_t reg;
					reg			= (((uint16_t) scratchPad[1]) << 8) | ((uint16_t) scratchPad[0]);
					dev->temp	= DS18B20_CalculateTemperature(reg);

                    // Get the Alarm configurations
					dev->Thigh	= scratchPad[2];
					dev->Tlow	= scratchPad[3];

					// Read the set temperature resolution
					dev->resolution = (DS_RESOLUTION) scratchPad[4];
                    DS18B20_clearErrorCount(app.read_index);

					#if DS18B20_LOG_ENABLE
						const char * p_res = NULL;
						const char * errorStrings[] =
						{
							"9 bit",
							"10 bit",
							"11 bit",
							"12 bit",
							"Error"
						};

						switch(dev->resolution)
						{
							case TEMP_9BIT_RESOLUTION:		p_res = errorStrings[0];	break;
							case TEMP_10BIT_RESOLUTION:		p_res = errorStrings[1];	break;
							case TEMP_11BIT_RESOLUTION:		p_res = errorStrings[2];	break;
							case TEMP_12BIT_RESOLUTION:		p_res = errorStrings[3];	break;
							default:						p_res = errorStrings[4];	break;
						}


						NRF_LOG_INFO("DS18B20[%u] res: 0x%02X/%s, Thigh: 0x%02X, Tlow: 0x%02X", 
							app.read_index,
							dev->resolution,
                            p_res,
							dev->Thigh,
							dev->Tlow);
					#endif
				}
				else
				{
					// Retry
					DS18B20_incErrorCount(app.read_index, status);
					return;
				}
			}
			
			// Check if all available devices have been read or not.
            app.read_index++;
			if(app.read_index >= app.device_count)
			{
				app.read_index = 0;
				app.settingsChanged = false;
				DS18B20_App_NewState(DS_SET_CONFIG);
			}
            break;
		}
			

		case DS_SET_CONFIG:
		{
			DS_ERROR_CODES status;
            sDS18B20 * dev;

			// Check the readCount
			if(app.read_index >= DS18B20_SENSORS_MAX)
			{
				app.read_index = DS18B20_SENSORS_MAX - 1;
			}	
            dev = &app.dev[app.read_index];

			// Check if the device is enabled and if the resolution is correct.
			if(dev->enabled && dev->resolution != app.resolution && dev->errorCount < DS18B20_ERROR_COUNT_MAX)
			{
				// Write the scratchpad with the new settings.
				status = DS18B20_WriteScratchPad(dev->ROM, dev->Thigh, dev->Tlow, (uint8_t) app.resolution);
				if(status != DS_SUCCESFULL)
				{
					// Increment the error count and Retry
					DS18B20_incErrorCount(app.read_index, status);
					return;
				}

				status = DS18B20_CopyScratchPad(dev->ROM);
                if(status != DS_SUCCESFULL)
				{
					// Increment the error count and Retry
					DS18B20_incErrorCount(app.read_index, status);
					return;
				}

				status = DS18B20_RecalEeprom(dev->ROM);
                if(status != DS_SUCCESFULL)
				{
					// Increment the error count and Retry
					DS18B20_incErrorCount(app.read_index, status);
					return;
				}

				// Clear any communication error after all these command complete succesfully.
                DS18B20_clearErrorCount(app.read_index);

				// Set the flag that a sensor' setting have been changed for a read cycle to determine if the new settigns have been correctly been applied.
				app.settingsChanged = true;
			}

			app.read_index++;
			if(app.read_index >= app.device_count)
			{
				app.read_index = 0;

				// When a device' settings have been changed reread the device's settings
				if(app.settingsChanged)
				{
					DS18B20_App_NewState(DS_READ_CONFIG);
				}
				else
				{
					// Start the conversion by setting the state first to IDlE and then calling the apptimer callback.
                    DS18B20_App_NewState(DS_IDLE);
                    DS18B20_App_timerCallback(NULL);
				}
			}
            break;
		}

		case DS_IDLE:
			break;

        case DS_START_TEMP_CONV:
		{
			bool pwr = powerApp_getEnabled(PWR_DS18B20);
			uint32_t conversionDelay = 0;

			if(!app.on)
			{
				// Return to Idle when not in a valid operation mode.
				DS18B20_App_NewState(DS_IDLE);
                powerApp_Enable(false, PWR_DS18B20);
				return;
			}
			else if(!pwr)
			{
				powerApp_Enable(true, PWR_DS18B20);
                nrf_delay_ms(10);
				return;
			}
			else if(DS18B20_App_index_conversion() == DS_SUCCESFULL && pwr)
			{
				app.conversionCount++;
				DS18B20_App_NewState(DS_WAIT_TEMP_CONV_COMPLETE);

				// Tconv depending on the resolution
				switch(app.resolution)
				{
					default:
					case TEMP_12BIT_RESOLUTION:		conversionDelay = 800;	break; // 750.00 ms
                    case TEMP_11BIT_RESOLUTION:		conversionDelay = 400;	break; // 375.00 ms
                    case TEMP_10BIT_RESOLUTION:		conversionDelay = 200;	break; // 187.50 ms
                    case TEMP_9BIT_RESOLUTION:		conversionDelay = 100;	break; //  93.75 ms
				}

                // Start the timer 
				APP_ERROR_CHECK(app_timer_start(timer, APP_TIMER_TICKS(conversionDelay), NULL));
                app.startTimestamp = app_timer_cnt_get();
			}
			break;
		}

		case DS_WAIT_TEMP_CONV_COMPLETE:
		{
			uint32_t passedTime_ms = app_timer_time_since_start_ms(app.startTimestamp);
			if(passedTime_ms > 3 * 1000 || app.startTimestamp == 0)
			{
				app.startTimestamp = 0;

				// Go to the next state if more than 3 seconds have passed or the timestamp is 0
				DS18B20_App_NewState(DS_GET_TEMP_CONV_RESULT);
			}
			break;
		}

		case DS_GET_TEMP_CONV_RESULT:
		{
			uint8_t scratchPad[DS18B20_SCRATCHPAD_LENGHT] = {0};
			DS_ERROR_CODES status;
            sDS18B20 * dev;

			// Check the readCount
			if(app.read_index >= DS18B20_SENSORS_MAX)
			{
				app.read_index = DS18B20_SENSORS_MAX - 1;
			}	
            dev = &app.dev[app.read_index];

			// Check if the device is enabled
			if(dev->enabled && dev->errorCount < DS18B20_ERROR_COUNT_MAX)
			{
				// Read each device's scratchpad memory individually
				status =  DS18B20_ReadScratchPad(dev->ROM, scratchPad);

				// When the read operation is succesfull retrieve the parameters.
				if(status == DS_SUCCESFULL)
				{
					// Calculate the temperature
					uint16_t reg;
					reg			= (((uint16_t) scratchPad[1]) << 8) | ((uint16_t) scratchPad[0]);
					dev->temp	= DS18B20_CalculateTemperature(reg);
                    DS18B20_clearErrorCount(app.read_index);

					#if DS18B20_LOG_ENABLE
						NRF_LOG_FLUSH();
						NRF_LOG_INFO("%u - DS18B20[%u] temp: 0x%04X/"NRF_LOG_FLOAT_MARKER"C", 
							app.conversionCount,
							app.read_index,
                            reg,
                            NRF_LOG_FLOAT(dev->temp));
						
					#endif
				}
				else
				{
					// Retry
					DS18B20_incErrorCount(app.read_index, status);
					return;
				}
			}
			
			// Check if all available devices have been read or not.			
			app.read_index++;
			if((app.convIndex == 0xFF && app.read_index >= app.device_count) || (app.convIndex != 0xFF && app.convIndex != app.read_index))
			{
				app.read_index = 0;

				// When a result handler is available, fill the result structure with the available data.
				if(app.handler != NULL)
				{
					MEASUREMENT_RESULT_s ds;
                    DS18B20_App_convert(app.source, &ds);
                    app.source	= INTERNAL_SOURCE;
					app.handler(&ds);
				}

				// Start the timer for the next Temperature conversion
				APP_ERROR_CHECK(app_timer_start(timer, APP_TIMER_TICKS(app.interval * 60 * 1000), NULL));
                DS18B20_App_NewState(DS_SHUTDOWN);
			}
			break;
		}

        case DS_SHUTDOWN:
            powerApp_Enable(false, PWR_DS18B20);
            DS18B20_App_NewState(DS_IDLE);
            break;
	}
}


uint32_t DS18B20_App_start_conversion(CONTROL_SOURCE source, uint8_t index)
{
	// Check if the statemachine can start a new conversion
	if(app.state != DS_IDLE || !app.on)
	{
		return NRF_ERROR_INVALID_STATE;
	}

	//
	if(index < DS18B20_SENSORS_MAX )
	{
		if(index >= app.device_count)
		{
			return NRF_ERROR_INVALID_PARAM;
		}
		// else
        app.convIndex = index; // Start a conversion for a specific temperature sensor
	}
	else
	{
		app.convIndex = 0xFF; // Start a conversion for all temperature sensors
	}
	
	app.source = source;
	app_timer_stop(timer);
    DS18B20_App_NewState(DS_START_TEMP_CONV);
	return NRF_SUCCESS;
}


uint32_t DS18B20_App_shutdown(void)
{
    if(app.state == DS_IDLE)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    DS18B20_App_NewState(DS_SHUTDOWN);
    return NRF_SUCCESS;
}




void DS18B20_App_init(uint16_t interval, DS_RESOLUTION	resolution, measurement_callback measurement_handler)
{
	uint32_t ret;
    BEEP_protocol_s conf;

	conf.command = READ_DS18B20_STATE;
    nvm_fds_eeprom_get(&conf);

	app.on				= conf.param.status.statusflag & (1<<0) ? true : false;
    app.resolution		= uint_to_resolution(conf.param.status.statusflag);
	app.handler			= measurement_handler;
	app.interval		= conf.param.status.interval;

	if(app.resolution == 0)
	{
		app.resolution = TEMP_12BIT_RESOLUTION;
	}

	#if 0
		TestTempCalc();
	#endif

	// Reset all parameters to default.
    DS18B20_statemachineReset();

    // Create an app timer.
    ret = app_timer_create(&timer, APP_TIMER_MODE_SINGLE_SHOT, DS18B20_App_timerCallback);
    APP_ERROR_CHECK(ret);

#if 0
	// Check the number of vibration pulses
    ret = app_timer_start(timer, APP_TIMER_TICKS(app.interval * 60 * 1000), NULL);
    APP_ERROR_CHECK(ret);
#endif
}


