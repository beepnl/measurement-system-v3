
#include <stdbool.h>
#include <stdint.h>
#include "pwm.h"
#include "app_timer.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "gpio-board.h"
#include "nrf_drv_pwm.h"
#include "nrf_drv_clock.h"
#include "nrf_log.h"
#include "app_util.h"

static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);
static bool clearPWM = false;


bool PWM_clear(void)
{
	if(clearPWM)
	{
		clearPWM = false;
		return true;
	}
	return false;
}

void PWM_shutdown(void)
{
	if(!nrfx_pwm_is_uninit(&m_pwm0))
	{
		nrf_drv_pwm_uninit(&m_pwm0);
	}
}


static void PWM_handler(nrf_drv_pwm_evt_type_t event_type)
{
    if (event_type == NRF_DRV_PWM_EVT_FINISHED)
    {
        nrf_drv_pwm_uninit(&m_pwm0);
        #if PWM_LOG_EN
        NRF_LOG_INFO("PWM handler unint");
        #endif
        clearPWM = true;
    }
    #if PWM_LOG_EN
        NRF_LOG_INFO("PWM handler %u/0x%02X, uninit %s", event_type, event_type, bool_to_str(nrfx_pwm_is_uninit(&m_pwm0)));
    #endif
}


void PWM_event(uint8_t dutycycle, uint16_t frequency, uint16_t nRepeat)
{
    uint32_t err; 
    static uint32_t regVal, topvalue;

	// Ensure that the boost converter is enabled before attempting to sounds the buzzer.
	#if 0
	if(!GpioBoostConverterGet())
	{
		return;
	}
	#endif

    if(frequency > 10000)
	{
       frequency = 10000;
    }

    if(frequency < 1000)
	{
       frequency = 1000;
    }

    topvalue = 1250000 / frequency;

    if(dutycycle > 100)
	{
      dutycycle = 50;
    }

    // Calculate the PWM register value from the maximum top value and the given duty Cycle.
    regVal = (topvalue * dutycycle) / 1000;
    topvalue /= 10;

    nrf_drv_pwm_config_t const config0 =
    {
        .output_pins =
        {
            BUZZER_PWM ,
            NRF_DRV_PWM_PIN_NOT_USED,
            NRF_DRV_PWM_PIN_NOT_USED,
            NRF_DRV_PWM_PIN_NOT_USED,
        },
        .irq_priority = APP_IRQ_PRIORITY_LOWEST,
        .base_clock   = NRF_PWM_CLK_125kHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = topvalue,
        .load_mode    = NRF_PWM_LOAD_COMMON,
        .step_mode    = NRF_PWM_STEP_AUTO
    };

	// Check whether the pwm peripheral is already initialized.
	bool pwmInit = nrfx_pwm_is_uninit(&m_pwm0);
    #if PWM_LOG_EN
        NRF_LOG_INFO("PWM start, uninit=%s", bool_to_str(pwmInit));
    #endif

    if(!pwmInit)
    {
        nrf_drv_pwm_uninit(&m_pwm0);
        #if PWM_LOG_EN
            NRF_LOG_INFO("PWM start unint");
        #endif
    }

    err = nrf_drv_pwm_init(&m_pwm0, &config0, PWM_handler);
    APP_ERROR_CHECK(err);


    // This array cannot be allocated on stack (hence "static") and it must
    // be in RAM (hence no "const", though its content is not changed).
    static uint16_t /*const*/ seq_values[1];
    seq_values[0] = regVal;

    nrf_pwm_sequence_t const seq =
    {
        .values.p_common = seq_values,
        .length          = NRF_PWM_VALUES_LENGTH(seq_values),
        .repeats         = 0,
        .end_delay       = 0
    };

    (void)nrf_drv_pwm_simple_playback(&m_pwm0, &seq, nRepeat, NRF_DRV_PWM_FLAG_STOP);
}


