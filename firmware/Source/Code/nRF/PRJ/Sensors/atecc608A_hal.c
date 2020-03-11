#define NRF_LOG_MODULE_NAME ATECC_HAL
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
#include "atecc608A_hal.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_twi.h"
#include "boards.h"
#include "I2C.h"
#include "atca_hal.h"
#include "nrf_delay.h"


#define	ATECC_ADDRESS	0x60


void atca_delay_us(uint32_t delay)
{
	nrf_delay_us(delay);
}

void atca_delay_10us(uint32_t delay)
{
	nrf_delay_us(10 * delay);
}

void atca_delay_ms(uint32_t delay)
{
	nrf_delay_ms(delay);
}


ATCA_STATUS hal_i2c_init(void *hal, ATCAIfaceCfg *cfg)
{
	I2C_init();
	return ATCA_SUCCESS;
}

ATCA_STATUS hal_i2c_uninit(void *hal, ATCAIfaceCfg *cfg)
{
	I2C_init();
	return ATCA_SUCCESS;
}


ATCA_STATUS hal_i2c_post_init(ATCAIface iface)
{
	return ATCA_SUCCESS;
}

ATCA_STATUS hal_i2c_send(ATCAIface iface, uint8_t *txdata, int txlength)
{
	uint32_t ret;
	ATCAIfaceCfg *cfg = atgetifacecfg(iface);

    // for this implementation of I2C with CryptoAuth chips, txdata is assumed to have ATCAPacket format
    // other device types that don't require i/o tokens on the front end of a command need a different hal_i2c_send and wire it up instead of this one
    // this covers devices such as ATSHA204A and ATECCx08A that require a word address value pre-pended to the packet
    // txdata[0] is using _reserved byte of the ATCAPacket
    txdata[0] = 0x03;   // insert the Word Address Value, Command token
    txlength++;         // account for word address value byte.
	ret = I2C_writeArray(ATECC_ADDRESS, txdata, txlength); // cfg->atcai2c.slave_address
	if(ret != NRF_SUCCESS)
	{
		return ATCA_COMM_FAIL;
	}
	return ATCA_SUCCESS;
}

ATCA_STATUS hal_i2c_receive(ATCAIface iface, uint8_t *rxdata, uint16_t *rxlength)
{
	ret_code_t ret;
	uint8_t msgLenght = 0, rxBufferLenght;
	ATCAIfaceCfg *cfg = atgetifacecfg(iface);

    rxBufferLenght = *rxlength;
    *rxlength = 0;
    if (rxBufferLenght < 1)
    {
        return ATCA_SMALL_BUFFER;
    }

	// Read the lenght of the message
	ret = I2C_read(ATECC_ADDRESS, 0xFF, rxdata); // cfg->atcai2c.slave_address
	if(ret != NRF_SUCCESS)
	{
		return ATCA_COMM_FAIL;
	}

	// Check if the RX buffer is able to hold the message
    if (rxdata[0] < ATCA_RSP_SIZE_MIN)
    {
        return ATCA_INVALID_SIZE;
    }
    if (rxdata[0] > rxBufferLenght)
    {
        return ATCA_SMALL_BUFFER;
    }

	msgLenght = rxdata[0] - 1;
	ret = I2C_readArray(ATECC_ADDRESS, 0xFF, &rxdata[1], msgLenght); // cfg->atcai2c.slave_address
	if(ret != NRF_SUCCESS)
	{
		return ATCA_COMM_FAIL;
	}

	*rxlength = rxdata[0];
	return ATCA_SUCCESS;
}


ATCA_STATUS hal_i2c_wake(ATCAIface iface)
{
	uint32_t ret;
	uint8_t rxData[4] = {0};
	ATCAIfaceCfg *cfg = atgetifacecfg(iface);

	// Send 0x00 as wake pulse
	ret = I2C_write(ATECC_ADDRESS, 0xFF, 0x00);
    atca_delay_ms(3);   // wait tWHI + tWLO which is configured based on device type and configuration structure

	// Read four bytes from the device
    ret = I2C_readArray(ATECC_ADDRESS, 0xFF, rxData, 4);

	return hal_check_wake(rxData, 4);
}

ATCA_STATUS hal_i2c_idle(ATCAIface iface)
{
	return ATCA_SUCCESS;
}

ATCA_STATUS hal_i2c_sleep(ATCAIface iface)
{
	return ATCA_SUCCESS;
}

ATCA_STATUS hal_i2c_release(void *hal_data)
{
	return ATCA_SUCCESS;
}
ATCA_STATUS hal_i2c_discover_buses(int i2c_buses[], int max_buses)
{
	return ATCA_SUCCESS;
}

ATCA_STATUS hal_i2c_discover_devices(int bus_num, ATCAIfaceCfg *cfg, int *found)
{
	return ATCA_SUCCESS;
}




