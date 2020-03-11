
#include <stdbool.h>
#include <stdint.h>
#include "SQ.h"
#include "app_timer.h"
#define NRF_LOG_MODULE_NAME SQ
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include "boards.h"

APP_TIMER_DEF(vibTimer);


// Angle sensor
SQ_SEN_645B sen =
{
	.timeCount		= 0,
	#ifdef DEBUG
		.timeThreshold	= 5,
	#else
		.timeThreshold	= 5,
	#endif
};

measurement_callback sq_handler;

bool SQ_startup(void)
{
	return sen.startUp;
}


uint32_t SQ_SEN_645B_pinRead (void)
{
	return nrf_gpio_pin_read(SQ_SEN_645B_PIN);
}

static inline void SQ_Event_Call(SQ_ORIENTATIONS_e eventType)
{
	if(sq_handler != NULL)
	{
		MEASUREMENT_RESULT_s result;
		result.type = SQ_MIN;
		result.result.sq.orientation = eventType;
		sq_handler(&result);
	}
}




static inline void SQ_SEN_645B_Tick(void)
{
	static uint32_t count = 0;
	// Get the current output
	sen.stateNew = SQ_SEN_645B_pinRead();

	if(sen.startUp)
	{
		// On start-up determine the current orientation over a longer time to prevent snap shots.
		if(sen.stateNew)
		{
			if((sen.countHorizontal < UINT8_MAX))
			{
				sen.countHorizontal++;
			}
		}
		else
		{
			if((sen.countVertical < UINT8_MAX))
			{
				sen.countVertical++;
			}
		}

		sen.timeCount++;
		if(sen.timeCount >= sen.timeCount)
		{
			sen.timeCount = 0;
			if(sen.countHorizontal > sen.countVertical)
			{
				sen.stateCurrently = 1;
			}
			else
			{
				sen.stateCurrently = 0;
			}

			// After the start-up period and the current orientation has been detected, call the main application
			sen.startUp = false;
			NRF_LOG_INFO("Startup orientation is %s", (sen.stateCurrently) ? "Horizontal" : "Vertical");
            SQ_Event_Call((sen.stateCurrently) ? SQ_SEN_645B_HORIZONTAL : SQ_SEN_645B_VERTICAL);
		}
	}
	else
	{
		if(sen.stateNew != sen.stateCurrently)
		{
			sen.timeCount++;
		}
		else if(sen.timeCount > 0)
		{
			sen.timeCount--;
		}

		if(sen.timeCount >= sen.timeThreshold)
		{
			sen.timeCount		= 0;
			sen.stateCurrently	= sen.stateNew;
			#if SQ_DEBUG
				NRF_LOG_INFO("%u - SQ-SEN-645B new state: %u", count++, sen.stateCurrently);
			#endif
			SQ_Event_Call((sen.stateCurrently) ? SQ_SEN_645B_HORIZONTAL : SQ_SEN_645B_VERTICAL);
		}
		#if 0
		else
		{
			NRF_LOG_INFO("%u - SQ-SEN-645B pin state: %u, time: %u", count++, sen.stateNew, sen.timeCount);
		}
		#endif
	}
}


static void SQ_MIN_200_timerCallback( void * p_context )
{
	UNUSED_VARIABLE(p_context);
    SQ_SEN_645B_Tick();
}






void SetGPIOinterrupt(uint32_t pin, nrfx_gpiote_evt_handler_t evt_handler)
{
    uint32_t err_code;

    /*  Pin change interrupt event configuration. For low power consumption the accuracy must be set to false and in the interrupt must be checked whether the pin is high or low. */
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
    in_config.hi_accuracy = false;

    err_code = nrf_drv_gpiote_in_init(pin, &in_config, evt_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(pin, true);
}


void SQ_init(measurement_callback handler)
{
	uint32_t ret;

	if(handler != NULL)
	{
		sq_handler = handler;
	}

	sen.startUp			= true;
	sen.timeCount		= 0;
	sen.countHorizontal	= 0;
	sen.countVertical	= 0;

    // Create an app timer.
    ret = app_timer_create(&vibTimer, APP_TIMER_MODE_REPEATED, SQ_MIN_200_timerCallback);
    APP_ERROR_CHECK(ret);
	
	#if 1
		// Check the number of vibration pulses
		ret = app_timer_start(vibTimer, APP_TIMER_TICKS(1000), NULL);
		APP_ERROR_CHECK(ret);
	#endif

	// Set the current pin input value as the current state.
	sen.stateCurrently	= SQ_SEN_645B_pinRead();
}

void SQ_deinit(void)
{
	app_timer_stop(vibTimer);
}




bool SQ_getOrientation(void)
{
	return (sen.stateCurrently) ? true : false;
}


