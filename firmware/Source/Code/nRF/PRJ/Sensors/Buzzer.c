#include <stdbool.h>
#include <stdint.h>
#include "Buzzer.h"
#include "app_timer.h"
#define NRF_LOG_MODULE_NAME BUZZER
#include "nrf_log.h"
#include "app_util.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_gpio.h"
#include "boards.h"
#include "power_app.h"
#include "pwm.h"
#include "nRF_ADC.h"

APP_TIMER_DEF(buzzTimer);
#define TURN_ON_DELAY_ms 10

struct
{
	volatile bool on;
	bool ONperiod;	
	float dutyCycle;
	uint32_t	freq;
	uint32_t	onPeriodTicks;
	uint16_t	onTime;
	uint16_t	offtime;
	uint16_t	repeatCount;
	uint16_t	repeatThreshold;
}buzz;



static void Buzzer_timerCallback( void * p_context )
{
	uint32_t ret;
	UNUSED_VARIABLE(p_context);

	if(!powerApp_getEnabled(PWR_BUZZER))
	{
		buzz.on = false;
		powerApp_Enable(false, PWR_BUZZER);
	}
	else
	{
		if(buzz.ONperiod)
		{
			buzz.ONperiod = false;
			PWM_event(buzz.dutyCycle, buzz.freq, buzz.onPeriodTicks);
            ret = app_timer_start(buzzTimer, APP_TIMER_TICKS(buzz.onTime), NULL);
		}
		else
		{
			buzz.ONperiod = true;
			if(buzz.repeatCount++ < buzz.repeatThreshold)
			{
				ret = app_timer_start(buzzTimer, APP_TIMER_TICKS(buzz.offtime), NULL);
			}
			else
			{
				//	Release power lock
                buzz.on = false;
                powerApp_Enable(false, PWR_BUZZER);

				// Start the ADC
                saadc_start_conversion(INTERNAL_SOURCE);
			}
		}

        #if 0
			static uint32_t count = 0;
			NRF_LOG_INFO("%u - repeatCount=%u, ON=%s", count++, buzz.repeatCount, bool_to_str(buzz.ONperiod));
		#endif
	}
}

bool BuzzerIsOn(void)
{
	return buzz.on;
}

void Buzzer_init(const uint32_t interval)
{
	uint32_t ret;

    // Create an app timer.
    ret = app_timer_create(&buzzTimer, APP_TIMER_MODE_SINGLE_SHOT, Buzzer_timerCallback);
    APP_ERROR_CHECK(ret);

	// Start a periodic timer for debugging
	if(interval != 0)
	{
		ret = app_timer_start(buzzTimer, APP_TIMER_TICKS(interval * 1000), NULL);
		APP_ERROR_CHECK(ret);
	}
}

uint32_t Buzzer_sound(const uint8_t dutyCycle, const uint32_t freq, const uint16_t onTime, const uint16_t offtime, const uint16_t repeat)
{
	uint32_t ret;

	#if !BUZZER_ENABLE
		return NRF_ERROR_INVALID_STATE;
	#endif

    if(buzz.on)
	{
		return NRF_ERROR_INVALID_STATE;
	}

	if(dutyCycle == 0 || dutyCycle >= 100 || onTime == 0 || offtime == 0 || repeat == 0)
	{
		return NRF_ERROR_INVALID_PARAM;
	}
    buzz.on                 = true;
	buzz.dutyCycle			= dutyCycle;
	buzz.freq				= freq;
	buzz.onTime				= onTime;
	buzz.offtime			= offtime;
	buzz.repeatCount		= 0;
	buzz.repeatThreshold	= repeat;
	buzz.ONperiod			= false;
	buzz.onPeriodTicks		= ((buzz.freq * buzz.onTime) / 1000);

	// Enable the boost converter for the buzzer.
	powerApp_Enable(true, PWR_BUZZER);

	ret = app_timer_start(buzzTimer, APP_TIMER_TICKS(TURN_ON_DELAY_ms), NULL);
	return ret;
}


uint32_t Buzzer_default_tune(uint8_t TuneIndex)
{
	uint32_t ret;
	switch(TuneIndex)
	{
		case 0:
			ret = Buzzer_sound(50, 2800, 100, 1000, 4);
			break;
		case 1:
			ret = Buzzer_sound(50, 2800, 1000, 1, 1);
			break;
		case 2:
			ret = Buzzer_sound(50, 2800, 50, 100, 2);
			break;

		default:
			ret = NRF_ERROR_INVALID_PARAM;
			break;
	}
	return ret;
}


