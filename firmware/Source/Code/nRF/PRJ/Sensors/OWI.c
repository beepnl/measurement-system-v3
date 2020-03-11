#include "OWI.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include "boards.h"

__inline void OWI_Power(void)
{
	nrf_gpio_cfg_output_state(OWI_PIN, 1);
}


__inline void OWI_High (void)
{
	nrf_gpio_cfg_input(OWI_PIN, NRF_GPIO_PIN_NOPULL);
}

/* Set the pull-up Low to cutt-off power for the one-Wire interface and the DS18B20. */
void OWI_Low (void)
{
	nrf_gpio_cfg_output_state(OWI_PIN, 0);
}

/*
*****************************************************************************************
 * \brief	Write Bit function
*****************************************************************************************
*/
// Set the OWI high to send a bit. Write LOW time: min 1 us - max 15 us 
__inline void OWI_Write1 (void)
{
    OWI_Low();
	nrf_delay_us(TIME_A);
	OWI_High();
    nrf_delay_us(TIME_B);
}

// Clear the OWI pin to write a bit. Write LOW time: min 60 us - max 120 us
__inline void OWI_Write0 (void)
{
    OWI_Low();
	nrf_delay_us(TIME_B);
	OWI_High();
	nrf_delay_us(TIME_A);
}

void OWI_write(uint8_t data)
{
	uint8_t i;

	for(i=0; i<8; i++)
	{
		// Write least significant bit first.
		if(data & (1 << i))
		{
			OWI_Write1();
		}
		else
		{
			OWI_Write0();
		}
	}
}

__inline uint32_t OWI_readBit(void)
{
	uint32_t read;
	OWI_Low();
	nrf_delay_us(TIME_A);
	OWI_High();
    nrf_delay_us(TIME_A);

    read = nrf_gpio_pin_read(OWI_PIN);
    nrf_delay_us(TIME_C);

	return read;
}

uint8_t OWI_readByte(void)
{
	uint32_t i;
	uint8_t retval = 0;

	for(i=0; i<8; i++)
	{
		retval |= (OWI_readBit() << i);
	}
	return retval;
}


void OWI_reset(void)
{
	OWI_High();
	nrf_delay_us(TIME_RESET);
	OWI_Low();
	nrf_delay_us(TIME_RESET);
    OWI_High();
}


bool OWI_presenceDetect(void)
{
	bool pressence = false;
	uint32_t pin_read;

	OWI_reset();
	nrf_delay_us(80);

    pin_read = nrf_gpio_pin_read(OWI_PIN);
	if(!pin_read)
	{
		pressence = true;
	}
    nrf_delay_us(240);

	return pressence;
}









