
#include <stdbool.h>
#include <stdint.h>
#include "HX711_SPI.h"
#include "HX711.h"
#include "app_timer.h"

#define NRF_LOG_MODULE_NAME HX711
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
#include "nrf_delay.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include "boards.h"
#include "gpio-board.h"



HX711_s	hx;

// Private functions declarations
int32_t HX711_read(uint32_t clkPulses);


/*
 * 
 */
void HX711_DOUTevent(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	uint32_t clkPulses = 0;

	// DOUT pin will go low when data is available. Since it's a toggle event, do not continue when the pin is high.
	if(nrf_gpio_pin_read(pin) != 0)
	{
		return;
	}

	switch(hx.gain)
	{
		default:
		case CH_A_GAIN128:
			clkPulses = CLK_CYCLES_CH_A_GAIN128;
			break;
		case CH_B_GAIN32:
			clkPulses = CLK_CYCLES_CH_B_GAIN32;
			break;
		case CH_A_GAIN64:
			clkPulses = CLK_CYCLES_CH_A_GAIN64;
			break;
	}

	hx.last_result = HX711_read(clkPulses);

	// Return the sample result to the application level.
	if(hx.handler != NULL)
	{
		HX711_CONV_s	result;
		memset(&result, 0, sizeof(HX711_CONV_s));
        result.value[0]	= hx.last_result;
        result.samples	= hx.sample_count;
        result.channel	= hx.gain;
		hx.handler(&result);

        #if 0
			NRF_LOG_INFO("%u - ch: %u, clk pulses: %u, result: %i, 0x%06X", hx.sample_count, hx.gain, clkPulses, hx.last_result, hx.last_result);
		#endif
	}
}

void HX711_DOUTeventEnable(bool enable)
{
	// Check whether the pin is used by the gpiote. If not, return.
	if(!pin_in_use_by_gpiote(HX711_DOUT))
	{
		return;
	}

	if(enable)
	{
		nrfx_gpiote_in_event_enable(HX711_DOUT, true);
	}
	else
	{
		nrfx_gpiote_in_event_disable(HX711_DOUT);
	}
}

void HX711_DOUTeventInit(void)
{
	uint32_t err_code;

    /*  Pin change interrupt event configuration. For low power consumption the accuracy must be set to false and in the event there must be checked whether the pin is high or low. */
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
    in_config.hi_accuracy = false;

    err_code = nrf_drv_gpiote_in_init(HX711_DOUT, &in_config, HX711_DOUTevent);
    APP_ERROR_CHECK(err_code);

    HX711_DOUTeventEnable(false);
}


int32_t HX711_read(uint32_t clkPulses)
{
	uint8_t nested, i;
	uint32_t result = 0, in;

    HX711_DOUTeventEnable(false);

    app_util_critical_region_enter(&nested);

	#if 0
	// Protect against interrupts to prevent timing issues.
	

	for(i=0; i < 24; i++)
	{
		// Set the clock pin high
		nrf_gpio_pin_set(HX711_PD_SCK);
		nrf_delay_us(1);
		in = nrf_gpio_pin_read(HX711_DOUT);
		result |= (in << (23 - i));

		// Set the clock pin low
        nrf_gpio_pin_clear(HX711_PD_SCK);
        nrf_delay_us(1);
	}

	// Clock the additional bits to set the next channel conversion with the new gain.
	while(i < gain)
	{
		nrf_gpio_pin_set(HX711_PD_SCK);
		nrf_delay_us(2);
		nrf_gpio_pin_clear(HX711_PD_SCK);
        nrf_delay_us(2);
		i++;
	}

	

	// If the 23th sign bit is set, append the nex 8 bit to convert the 24 bit result to 32 bit int.
	if(result & (1<<23))
	{
		result |= 0xFF000000;
	}
	#else
		// Init the SPI interface
		hx_spi_init(NULL);

        hx_spi_read(&result, clkPulses);

		//deinit the SPI interface
        hx_spi_deinit();
	#endif

	// Increment the sample count.
	hx.sample_count++;

	nrf_gpio_cfg_input(HX711_DOUT, NRF_GPIO_PIN_NOPULL);

    app_util_critical_region_exit(nested);

	// Re-enable the generation of an event when the DOUT pin goes high.
    HX711_DOUTeventEnable(true);

	return (int32_t) result;
}



void HX711_setGain(HX711_GAIN gain)
{
	hx.gain = gain;
}

/*
 *	@Brief: function to enable the HX711 by setting the PD_SCK pin. The HX711 is powered down when the PD_SCK is low for more then 60us. 
 */
void HX711_enable(bool enable)
{
	if(enable)
	{
		nrf_gpio_pin_clear(HX711_PD_SCK);	
	}
	else
	{
        nrf_gpio_pin_set(HX711_PD_SCK);
	}

	HX711_DOUTeventEnable(enable);
}



void HX711_setRate(bool highSampleRate)
{
	hx.rate = highSampleRate;
	if(hx.rate)
	{
		nrf_gpio_pin_set(HX711_RATE);
	}
	else
	{
		nrf_gpio_pin_clear(HX711_RATE);
	}
}


void hx_spi_event(nrf_drv_spi_evt_t const * p_event, void * p_context)
{
	
}


void HX711_init	(HX711_callback callback, HX711_GAIN gain, bool highSampleRate)
{
	hx.sample_count	= 0;
    hx.last_result	= 0;
	hx.handler		= callback;
    NRF_LOG_FLUSH();


	// Configure the IO
    nrf_gpio_cfg_output_state(HX711_DOUT,	0);
	nrf_gpio_cfg_output_state(HX711_PD_SCK,	0);
    nrf_gpio_cfg_output_state(HX711_RATE,	0);

	// Set Gain and sample rate
    HX711_setGain(gain);
	HX711_setRate(highSampleRate);
    HX711_enable(false);

	// Configure event generation on DOUT pin state changes. (Which needs to be disabled when reading the conversion result.)
    HX711_DOUTeventInit();
}
