
#include <stdbool.h>
#include <stdint.h>
#include "HX711_SPI.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "boards.h"
#include "nrf_drv_spi.h"
#include "nrf_spi.h"
#include "nrfx_spim.h"


const nrfx_spim_t HX_SPI  = NRFX_SPIM_INSTANCE(3);
static uint8_t clkData[HX_BUFFER_SIZE] = {
	0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 
	0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 
	0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
	0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
	0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
    0xF0, 0xF0};
static uint8_t rxData[HX_BUFFER_SIZE] = {0};


/*@Brief: Function to read an array from a given register
*/
void hx_spi_read(uint32_t * value, uint8_t size)
{
    uint32_t err_code, i;
	uint32_t result = 0;

	if(size < 25 || size > 27)
	{
		return;
	}

	memset(rxData, 0, HX_BUFFER_SIZE);

    nrfx_spim_xfer_desc_t transfer;

    transfer.p_tx_buffer	= clkData;
    transfer.tx_length		= size;
    transfer.p_rx_buffer	= rxData;
    transfer.rx_length		= size;

    //	Receive the data.
    err_code = nrfx_spim_xfer(&HX_SPI, &transfer, 0);
    APP_ERROR_CHECK(err_code);

	// Only the first 24 bytes contain data
	for(i=0; i<24; i++)
	{
		// Set the bit high when the array index isn't zero.
		if((rxData[i] & 0x0F) != 0)
		{
			result |= (1<<(23-i));
		}
	}

    // If the 23th sign bit is set, append the nex 8 bit to convert the 24 bit result to 32 bit int.
	if(result & (1<<23))
	{
		result |= 0xFF000000;
	}
	*value = result;
}





void hx_spi_deinit(void)
{
	nrfx_spim_uninit(&HX_SPI);
}

void hx_spi_init(nrf_drv_spi_evt_handler_t handler)
{
    uint32_t err_code;
    nrfx_spim_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
		spi_config.mode         = NRF_SPIM_MODE_0;
		spi_config.frequency    = NRF_SPIM_FREQ_8M;
		spi_config.ss_pin       = NRFX_SPIM_PIN_NOT_USED;
		spi_config.miso_pin     = HX711_DOUT; 
		spi_config.mosi_pin     = HX711_PD_SCK; 
		spi_config.sck_pin      = HX711_CLK_NC;
		spi_config.irq_priority = 6;
		spi_config.orc			= 0; // use zero padding
        spi_config.bit_order	= NRF_SPIM_BIT_ORDER_MSB_FIRST;

    err_code = nrfx_spim_init(&HX_SPI, &spi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);
}




