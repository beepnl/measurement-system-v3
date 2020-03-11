#define NRF_LOG_MODULE_NAME HXapp
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "HX711_app.h"
#include "HX711.h"
#include "nrf_log_ctrl.h"
#include "nrf_delay.h"
#include "utilities.h"
#include "app_timer.h"
#include "gpio-board.h"
#include "power_app.h"
#include "ble_setup.h"
#include "nvm_fs.h"

HX711_APP_s hxApp;



void clearResults(void)
{
	memset(&hxApp.results, 0, sizeof(HX711_CONV_s));
}


HX711_GAIN getNextGain(void)
{
	HX711_GAIN gain;

	if(hxApp.channels & CH_A_GAIN128)
	{
		gain = CH_A_GAIN128;
	}
    else if(hxApp.channels & CH_B_GAIN32)
	{
		gain = CH_B_GAIN32;
	}
    else if(hxApp.channels & CH_A_GAIN64)
	{
		gain = CH_A_GAIN64;
	}
	else
	{
		// No channels available
		return CH_HX_INVALID;
	}

	// Remove the gain bit from the channels parameters
    hxApp.channels &=~gain;
	return gain;
}

static void HX711_app_newState(HX711_STATES_s newState)
{
	hxApp.state = newState;

	#if 0
		const char * stateStrings[] =
		{
			"INIT state"
            "IDLE state"
            "START SAMPLING state"
            "SAMPLING state"
            "STOP SAMPLING state"
			"UNKNOWN State",
		};
		NRF_LOG_INFO("New HX711 App state: 0x%04X = %s", (uint16_t) newState, ((newState >= HX_UNKNOWN) ? stateStrings[HX_UNKNOWN] : stateStrings[newState]));
		NRF_LOG_FLUSH();
	#endif
}


void HX711_app_result_handler(HX711_CONV_s * result)
{
	uint8_t i, resultIndex = 0;
	#if 0
		// Print the results
		NRF_LOG_INFO("%u - %s, result %u", result->samples, HX711_channel_str(result->gain), result->value);
	#endif
	if(hxApp.state != HX_SAMPLING)
	{
		return;
	}

	if(hxApp.convCount_counter++ == 0)
	{	
		// Discard the first sample
        hxApp.convSum = 0;
		return;
	}
	else if(hxApp.convCount_counter <= hxApp.convCount_treshold)
	{
		// Add the last result to the sum
		hxApp.convSum += (int32_t) result->value[0];
	}
	else
	{
		// Disable the timer
        hxApp.convSum += result->value[0];

		// Calculate the average result
        hxApp.average_result = (hxApp.convSum / hxApp.convCount_treshold);

		#if 0
			NRF_LOG_INFO("%u - Conversion average result %s: %i, sum: %i, gain bit: %u", hxApp.sampleCount, HX711_channel_str(hxApp.gain), hxApp.average_result, hxApp.convSum, hxApp.gain);
		#endif

		// Store the result in the struct
		for(i=0; i < 3; i++)
		{
			// Find a spot in the results array which is still empty
			if(!(hxApp.results.channel & (1<<i)))
			{
				hxApp.results.channel			|= hxApp.gain; // set the bit for the gain configuration
				hxApp.results.value[resultIndex] = hxApp.average_result;
				break;
			}
			else
			{
				resultIndex++;
			}
		}

		hxApp.gain = getNextGain();
		
		// Disable the HX711 when no channel has to be measured.
		if(hxApp.gain != CH_HX_INVALID)
		{	
			HX711_setGain(hxApp.gain);
			hxApp.convCount_counter		= 0;
            hxApp.convSum				= 0;
            hxApp.average_result		= 0;
		}
		else
		{
			hxApp.sampleCount++;

			// Stop sampling and disable interrupt generation
			HX711_enable(false);

            // Call the callback when available
			if(hxApp.handler != NULL)
			{
				MEASUREMENT_RESULT_s main_cb;
				memset(&main_cb, 0, sizeof(main_cb));
				memcpy(&main_cb.result.hx711, &hxApp.results, sizeof(HX711_CONV_s));
				main_cb.type	= HX711;
                main_cb.source	= hxApp.source;
                hxApp.source	= INTERNAL_SOURCE;
				hxApp.handler(&main_cb);
			}

            HX711_app_newState(HX_STOP_SAMPLING);
		}
	}
}

bool HX711_app_busy(void)
{
	return (hxApp.state != HX_INIT && hxApp.state != HX_IDLE) ? true : false;
}


void HX711_app_While(void)
{
	switch(hxApp.state)
	{
		//----------------------------------------------------------------------------------------------------------
        default:
		case HX_INIT:
			break;

        //----------------------------------------------------------------------------------------------------------
		case HX_IDLE:
			break;

        //----------------------------------------------------------------------------------------------------------
		case HX_SAMPLING:
			// Wait for samples.
			break;

		//----------------------------------------------------------------------------------------------------------
		case HX_START_SAMPLING:
		{
			bool pwr = powerApp_getEnabled(PWR_HX711);

			// Check if the power has been turned on.
			if(!pwr)
			{
				// If not enabled yet, start the TPS boost converter and power swith and give the hardware 10ms to power on.
				powerApp_Enable(true, PWR_HX711);
                nrf_delay_ms(10);
			}
			else
			{
				HX711_setGain(hxApp.gain);
                HX711_setRate(true);		// 80 samples per second 
				HX711_enable(true);			// Enable sampling
				HX711_app_newState(HX_SAMPLING);
			}
			break;
		}

		case HX_STOP_SAMPLING:
		{
			bool pwr = powerApp_getEnabled(PWR_HX711);
			if(pwr)
			{
                HX711_setRate(false);
                HX711_enable(false);			// disble sampling incase another sensor still has a claim on the power.
				powerApp_Enable(false, PWR_HX711);
			}
			else
			{
				HX711_app_newState(HX_IDLE);
			}
			break;
		}
	}
}


void HX711_app_Init(measurement_callback measurement_handler)
{
	hxApp.handler		= measurement_handler;
	hxApp.state			= HX_IDLE;
	hxApp.sampleCount	= 0;

    // Initialize the bridge.
    HX711_init(HX711_app_result_handler, CH_A_GAIN64, true);
    HX711_enable(false);

    powerApp_Enable(false, PWR_HX711);
}


void HX711_app_getLastResult(MEASUREMENT_RESULT_s * result)
{
	if(result == NULL)
	{
		return;
	}
    memset(result, 0, sizeof(MEASUREMENT_RESULT_s));
	memcpy(&result->result.hx711, &hxApp.results, sizeof(HX711_CONV_s));
    result->type = HX711;
}



uint32_t HX711_app_start_sampling(CONTROL_SOURCE source, uint16_t Nsamples, uint8_t channel)
{
	// Check if the input variables are valid
	if(Nsamples == 0 || ((channel & CH_HX_BITMASK) == 0))
	{
		return NRF_ERROR_INVALID_PARAM;
	}

	// Only in IDLE state may the sensor be started
	if(hxApp.state != HX_IDLE)
	{
		return NRF_ERROR_INVALID_STATE;
	}

	clearResults();	
    hxApp.results.samples		= Nsamples;
	hxApp.convCount_treshold	= Nsamples;
	hxApp.convCount_counter		= 0;
	hxApp.convSum				= 0;
	hxApp.channels				= channel;
	hxApp.gain					= getNextGain();
	hxApp.source				= source;

	// Change the state to start sampling
    HX711_app_newState(HX_START_SAMPLING);
	return NRF_SUCCESS;
}


uint32_t HX711_app_shutdown(void)
{
    if(hxApp.state == HX_IDLE)
	{
		return NRF_ERROR_INVALID_STATE;
	}
    HX711_app_newState(HX_STOP_SAMPLING);
    return NRF_SUCCESS;
}


