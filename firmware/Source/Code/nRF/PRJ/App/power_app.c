#define NRF_LOG_MODULE_NAME PWR
#include "power_app.h"
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
#include "nrf_delay.h"
#include "app_timer.h"
#include "gpio-board.h"

/*
 * Lock variables to create a mutex around the boost converter and power switch while the sensor statemachines handle
 */
static volatile uint8_t boostConverterLock = 0;
static volatile uint8_t powerSwitchLock    = 0;


bool powerApp_getEnabled(power_t index)
{
	if(index > PWR_ALL)
	{
		return false;
	}
	else if(index == PWR_ALL)
	{
		return ((boostConverterLock & UINT8_MAX || powerSwitchLock & UINT8_MAX) ? true : false);
	}
	return ((boostConverterLock & (1<<index)  || powerSwitchLock & (1<<index)) ? true : false);
}



void powerApp_Enable(bool enable, power_t index)
{
    uint32_t boostPin, switchPin;

    const volatile uint8_t boostConverterLock_prev = boostConverterLock;
	const volatile uint8_t powerSwitchLock_prev    = powerSwitchLock;

	if(index > PWR_ALL)
	{
		return;
	}
	else if(index == PWR_ALL)
	{
		// Clear or enable all locks on the boost converter and power switch
		boostConverterLock = (enable) ? UINT8_MAX : 0;
        powerSwitchLock	   = (enable) ? UINT8_MAX : 0;
	}
	else
	{
		// Clear or enable for a specific sensor
		switch(index)
		{
			// sensor who need both the boost converter and power switch
			case PWR_DS18B20:
			case PWR_BME280:
			case PWR_HX711:
			case PWR_AUDIO:
			case PWR_ATECC:
			case PWR_BUZZER:

				if(enable)
				{
					boostConverterLock	|= (1<<index);
                    powerSwitchLock		|= (1<<index);
				}
				else
				{
					boostConverterLock	&= ~(1<<index);
                    powerSwitchLock		&= ~(1<<index);
				}
				break;

			// The RFM only requires the boost converter, power switch is don't care
			case PWR_RFM:

				if(enable)
				{
					boostConverterLock	|= (1<<index);
				}
				else
				{
					boostConverterLock	&= ~(1<<index) ;
				}
				break;
			default:
				break;
		}
	}


	// Check if the boost converter needs to be enabled of is no longer required
	if(boostConverterLock == 0 && boostConverterLock_prev != 0)
	{
		// Disable the bosot converter
        GpioBoostConverterSet(false);
	}
	else if(boostConverterLock != 0 && boostConverterLock_prev == 0)
	{
		// Enable the boost converter
        GpioBoostConverterSet(true);
	}

    // Check if the boost converter needs to be enabled of is no longer required
	if(powerSwitchLock == 0 && powerSwitchLock_prev != 0)
	{
		// Disable the power switch
        GpioPowerSwitchSet(false);

        // Set the GPIO to the new configuration now that the power's off.
        GPIO_switchOFF();
	}
	else if(powerSwitchLock != 0 && powerSwitchLock_prev == 0)
	{
		// Enable the power switch
        GpioPowerSwitchSet(true);

		// Set the GPIO to the new configuration now that the power's on.
        GPIO_switchON();

		// Reset the TLV320ADC3100 into a low power mode 
        GPIO_TLVreset();
	}

	#if 0
		const char * deviceStrings[] =
		{
			"DS18B20",
			"BME280",
			"HX711",
			"AUDIO ADC",
			"ATECC",
			"BUZZER",
			"RFM95",
			"ALL devices",
		};

		/*
		 * Disable logging for certain devices to reduce the logging overhead
		 */
		if( (index == PWR_DS18B20	&& 1) ||
			(index == PWR_BME280	&& 0) ||
			(index == PWR_HX711		&& 0) ||
			(index == PWR_AUDIO		&& 0) ||
			(index == PWR_ATECC		&& 0) ||
			(index == PWR_BUZZER	&& 0) ||
			(index == PWR_RFM		&& 0) ||
			(index == PWR_ALL		&& 0))
		{
			return;
		}

		NRF_LOG_FLUSH();
		NRF_LOG_INFO("Device: %s=%s - boost:0x%02X=%s, switch:0x%02X=%s",
			((index >= PWR_ALL)			? deviceStrings[PWR_ALL] : deviceStrings[index]),
			((enable)					? "On":"Off"),
			boostConverterLock,
			((GpioBoostConverterGet())	? "On":"Off"),
            powerSwitchLock,
            ((GpioPowerSwitchGet())		? "On":"Off"));
	#endif
}



