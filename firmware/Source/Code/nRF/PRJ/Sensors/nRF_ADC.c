#include <stdbool.h>
#include <stdint.h>
#include "sdk_config.h"
#include "nRF_ADC.h"
#define NRF_LOG_MODULE_NAME ADC
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
#include "boards.h"
#include "nrf.h"
#include "nrf_drv_saadc.h"
#include "app_timer.h"
#include "beep_types.h"
#include "app_util.h"

APP_TIMER_DEF(saadc_timer);
static nrf_saadc_value_t    m_buffer_pool[2][SAMPLES_IN_BUFFER];

static uint32_t             m_adc_evt_counter;
struct
{
	bool						supplyOk;
	bool						startUp;
	uint16_t					conversionCounter;
	uint16_t					conversionThreshold;
	int32_t						battADCSum;
	int32_t						averageADC;
	float						averageBatt_mV;
	uint32_t					intervalActive_ms;
	uint32_t					intervalStartup_ms;
    CONTROL_SOURCE				source;
}per;

ADC_s saadc;


measurement_callback adc_handler = NULL;



void get_saadc_result(MEASUREMENT_RESULT_s * result)
{
    memset(result, 0, sizeof(MEASUREMENT_RESULT_s));
    memcpy(&result->result.saadc, &saadc, sizeof(ADC_s));
}

bool saadc_VbatOk(void)
{
	return per.supplyOk;
}

bool saadc_startUp(void)
{
	return per.startUp;
}


static void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
	float input_voltage;
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

        saadc.battADC			= p_event->data.done.p_buffer[0];
        saadc.vccADC			= p_event->data.done.p_buffer[1];
		saadc.battVoltage_mV	= (saadc.battADC * 3600) / 4096;
		saadc.vccVoltage_mV		= (saadc.vccADC * 3600) / 4096;

        #if SAADC_LOG_ENABLE
            NRF_LOG_INFO("%u - 0x%04X = %u mVbat", m_adc_evt_counter,	saadc.battADC, saadc.battVoltage_mV);
            NRF_LOG_INFO("%u - 0x%04X = %u mVcc", m_adc_evt_counter,	saadc.vccADC, saadc.vccVoltage_mV);
        #endif
        m_adc_evt_counter++;

		// When the ADC handler pointer is valid, pass the measurement results back to the application
		if(per.conversionCounter++ < per.conversionThreshold)
		{
			per.battADCSum += (int32_t)saadc.battADC;
		}
		else
		{
			// Calculate the average voltage over N samples as a float.
			per.battADCSum			+= (int32_t)saadc.battADC;
			per.averageADC			= per.battADCSum / per.conversionThreshold;
            per.averageBatt_mV		= (((float)per.battADCSum) * 3600.0) / ((float)per.conversionThreshold * 4096.0);

			// Determine if the supply is ok.
			per.supplyOk = (per.averageBatt_mV > BATTERY_CUTT_OFF_mV) ? true : false;


			if(per.averageBatt_mV >= BATTERY_NOMINAL_mV)
			{
				saadc.battPercentage = 100;
			}
			else if(per.averageBatt_mV <= BATTERY_CUTT_OFF_mV)
			{
				saadc.battPercentage = 0;
			}
			else
			{
				// Linear battery percentage calculation
				saadc.battPercentage = ((per.averageBatt_mV - BATTERY_CUTT_OFF_mV) * 100.0) / (BATTERY_NOMINAL_mV - BATTERY_CUTT_OFF_mV);

				if(saadc.battPercentage >= 100)
				{
					saadc.battPercentage = 99;
				}
			}

            #if SAADC_LOG_ENABLE
				NRF_LOG_INFO("Battery sum: %i, samples: %u, average value: %i mV, %u %%, power ok = %s", per.battADCSum, per.conversionCounter, (int32_t) per.averageBatt_mV, saadc.battPercentage, bool_to_str(per.supplyOk));
			#endif

			per.conversionCounter	= 0;
			per.battADCSum			= 0;

			// Disable the fast power on sample interval and apply the slow interval.
			if(per.startUp)
			{
				app_timer_stop(saadc_timer);
				if(per.intervalActive_ms != 0)
				{
					app_timer_start(saadc_timer, APP_TIMER_TICKS(per.intervalActive_ms), NULL);
				}
				per.startUp	= false;
                #if SAADC_LOG_ENABLE
                    NRF_LOG_INFO("new ADC interval %u ms", per.intervalActive_ms);
                #endif
			}
		}

        nrf_drv_saadc_uninit();                                                                   //Unintialize SAADC to disable EasyDMA and save power
        NRF_SAADC->INTENCLR = (SAADC_INTENCLR_END_Clear << SAADC_INTENCLR_END_Pos);               //Disable the SAADC interrupt
        NVIC_ClearPendingIRQ(SAADC_IRQn);                                                         //Clear the SAADC interrupt if set

		if(adc_handler != NULL && !per.startUp)
		{
			MEASUREMENT_RESULT_s adc_data;
			adc_data.type	= nRF_ADC;
            adc_data.source	= per.source;
			memcpy(&adc_data.result.saadc, &saadc, sizeof(ADC_s));
		
			// Call the callback function
			adc_handler(&adc_data);
		}
    }
}




static void saadc_init(void)
{
    ret_code_t err_code;
    nrf_drv_saadc_config_t saadc_config;


	// Configure the ADC peripheral
    saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW;
    saadc_config.low_power_mode		= true;
    saadc_config.oversample			= NRF_SAADC_OVERSAMPLE_DISABLED;
    saadc_config.resolution			= NRF_SAADC_RESOLUTION_12BIT;

    err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);
    APP_ERROR_CHECK(err_code);

	// Configure channel 0
    nrf_saadc_channel_config_t channel_config0 = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN4);
    channel_config0.gain			= NRF_SAADC_GAIN1_6;
    channel_config0.reference		= NRF_SAADC_REFERENCE_INTERNAL;
    channel_config0.burst			= NRF_SAADC_BURST_DISABLED;

	// Configure channel 1
    nrf_saadc_channel_config_t channel_config1 = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);
    channel_config1.gain			= NRF_SAADC_GAIN1_6;
    channel_config1.reference		= NRF_SAADC_REFERENCE_INTERNAL;
    channel_config1.burst			= NRF_SAADC_BURST_DISABLED;

    err_code = nrf_drv_saadc_channel_init(0, &channel_config0);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(1, &channel_config1);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
}



static void SAADC_App_timerCallback(void * p_context)
{
	saadc_start_conversion(INTERNAL_SOURCE);
}


void saadc_start_conversion(CONTROL_SOURCE source)
{
	uint32_t err = 0;
	if(nrf_drv_saadc_is_busy())
	{
		return;
	}

	per.source = source;
    saadc_init();
	err = nrf_drv_saadc_sample();
}


void saadc_start(uint32_t interval_sec, measurement_callback callback)
{
    ret_code_t err_code;

    adc_handler				= callback;
	per.startUp				= true;
    per.supplyOk			= false;
    per.conversionCounter	= 0;
	per.battADCSum			= 0;
	per.intervalStartup_ms	= 100;
	per.conversionThreshold	= SAADC_AVERAGING;

    // Create an app timer.
    APP_ERROR_CHECK(app_timer_create(&saadc_timer, APP_TIMER_MODE_REPEATED, SAADC_App_timerCallback));

    per.intervalActive_ms = interval_sec * 1000;

	if(per.intervalStartup_ms != 0)
	{
		APP_ERROR_CHECK(app_timer_start(saadc_timer, APP_TIMER_TICKS(per.intervalStartup_ms), NULL));
	}
}


