#define NRF_LOG_MODULE_NAME I2C2
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
#include "I2C2.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_twi.h"
#include "boards.h"


/* TWI instance. */
static const nrf_drv_twi_t m_twi2 = NRF_DRV_TWI_INSTANCE(1);

/**
 * @brief TWI events handler.
 */
void twi2_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
            }
            break;
        default:
            break;
    }
}



ret_code_t I2C2_write(const uint8_t address, const uint8_t reg, uint8_t data)
{
	ret_code_t err_code;
	uint8_t msg[2];

	if(reg == 0xFF)
	{
		msg[0] = data;
	}
	else
	{
		msg[0] = reg;
		msg[1] = data;
	}
	err_code = nrf_drv_twi_tx(&m_twi2, address, msg, (reg == 0xFF) ? 1 : 2, false);
    return err_code;
}



ret_code_t I2C2_writeArray(const uint8_t address, const uint8_t reg, uint8_t * data, uint16_t lenght)
{
	ret_code_t err_code;
	static uint8_t txTemp[32];
	memset(txTemp, 0, 32);

	if(lenght > 31)
	{
		lenght = 31;
	}

	txTemp[0] = reg;
	memcpy(&txTemp[1], data, lenght);

	err_code = nrf_drv_twi_tx(&m_twi2, address, txTemp, lenght+1, false);
	return err_code;
}


ret_code_t I2C2_read(const uint8_t address, const uint8_t reg, uint8_t *retval)
{
	ret_code_t err_code;
	uint8_t data[1];

	if(reg != 0xFF)
	{
		err_code = nrf_drv_twi_tx(&m_twi2, address, &reg, 1, true);
		if(err_code != NRF_SUCCESS)
		{
			return err_code;
		}
	}

	err_code = nrf_drv_twi_rx(&m_twi2, address, data, 1);
    if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}

	if(retval != NULL)
	{
		*retval = data[0];
	}

	return err_code;
}


ret_code_t I2C2_readArray(const uint8_t address, const uint8_t reg, uint8_t * data, uint16_t lenght)
{
	ret_code_t err_code;

	if(reg != 0xFF)
	{
		err_code = nrf_drv_twi_tx(&m_twi2, address, &reg, 1, true);
		if(err_code != NRF_SUCCESS)
		{
			return err_code;
		}
	}
    err_code = nrf_drv_twi_rx(&m_twi2, address, data, lenght);
	return err_code;
}


nrfx_drv_state_t I2C2_state(void)
{
    return nrf_drv_twi_enabled(&m_twi2);
}



uint32_t I2C2_init(void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = SCL_EXT,
       .sda                = SDA_EXT,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_LOW,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi2, &twi_config, NULL, NULL); // twi_handler
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi2);

	#if 0 // Find slave devices on the bus.
		nrf_delay_ms(10);
		I2C_searchSlaves();
	#endif
    return err_code;
}

void I2C2_uninit(void)
{
	nrf_drv_twi_disable(&m_twi2);
    nrf_drv_twi_uninit(&m_twi2);
}

void I2C2_searchSlaves(void)
{
	ret_code_t err_code;
	uint8_t i, slaveCount = 0;

	for(i=1; i< 127; i++)
	{
		err_code = I2C2_read(i, 0, NULL);
		if(err_code == NRF_SUCCESS)
		{
			NRF_LOG_INFO("Found slave device %u on address: %u/0x%02X", slaveCount++, i, i);
			NRF_LOG_FLUSH();
		}
	}

    NRF_LOG_INFO("Found %u slave devices on the I2C bus", slaveCount);
	NRF_LOG_FLUSH();
}

