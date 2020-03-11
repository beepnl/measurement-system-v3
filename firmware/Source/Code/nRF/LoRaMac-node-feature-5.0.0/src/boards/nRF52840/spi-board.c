/*!
 * \file      spi-board.c
 *
 * \brief     Target board SPI driver implementation
 *
 * \author    Marten Lootsma(TWTG) on behalf of Microchip/Atmel (c)2017
 */
#include "boards.h"
#include "nrf_drv_spi.h"
#include "nrf_spi.h"
#include "spi-board.h"

const nrf_drv_spi_t SPI_A  = NRF_DRV_SPI_INSTANCE(2);
uint32_t nss_pin;

void SpiInit( Spi_t *obj, SpiId_t spiId, PinNames mosi, PinNames miso, PinNames sclk, PinNames nss )
{
    uint32_t err_code;
    nss_pin = nss;
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.mode         = NRF_DRV_SPI_MODE_0;
    spi_config.frequency    = NRF_DRV_SPI_FREQ_8M;
    spi_config.ss_pin       = NRF_DRV_SPI_PIN_NOT_USED;
    spi_config.miso_pin     = miso;
    spi_config.mosi_pin     = mosi;
    spi_config.sck_pin      = sclk;
    spi_config.irq_priority = 6;
    spi_config.orc			= 0; // use zero padding
    err_code				= nrf_drv_spi_init(&SPI_A, &spi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);
}

void SpiDeInit( Spi_t *obj )
{

}

uint16_t SpiInOut( Spi_t *obj, uint16_t outData )
{
    uint32_t err_code;
    uint8_t m_rx_buf[1] = {0};
    uint8_t m_tx_buf[1] = {0};
    m_tx_buf[0] = (uint8_t) outData;

    err_code = nrf_drv_spi_transfer(&SPI_A, m_tx_buf, 1, m_rx_buf, 1);
    APP_ERROR_CHECK(err_code);
}


void SpiWriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint32_t err_code;

    //NSS = 0;
    nrf_gpio_pin_clear(nss_pin);

    // Transmit the register address 
    addr |= 0x80;
    err_code = nrf_drv_spi_transfer(&SPI_A, &addr, 1, 0, 0);
    APP_ERROR_CHECK(err_code);

    //	Transmit the data.
    err_code = nrf_drv_spi_transfer(&SPI_A, buffer, size, 0, 0);
    APP_ERROR_CHECK(err_code);

    //NSS = 1;
    nrf_gpio_pin_set(nss_pin);
}

/*@Brief: Function to read an array from a given register
*/
void SpiReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint32_t err_code;

    //NSS = 0;
    nrf_gpio_pin_clear(nss_pin);

    //	Transmit the register address
    err_code = nrf_drv_spi_transfer(&SPI_A, &addr, 1, 0, 0);
    APP_ERROR_CHECK(err_code);

    //	Receive the data.
    err_code = nrf_drv_spi_transfer(&SPI_A, 0, 0, buffer, size);
    APP_ERROR_CHECK(err_code);

    //NSS = 1;
    nrf_gpio_pin_set(nss_pin);
}





