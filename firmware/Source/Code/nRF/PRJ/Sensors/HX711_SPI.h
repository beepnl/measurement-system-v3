/*!
 * \file      HX711_SPI.h
 *
 * \brief     HX711 SPI communication driver
 *
 */
#ifndef __HX711_SPI_H__
#define __HX711_SPI_H__

	#include <stdbool.h>
	#include <stdint.h>
	#include "nrf_drv_spi.h"

	#define HX_BUFFER_SIZE 27

	void	hx_spi_read		(uint32_t * value, uint8_t size);
	void	hx_spi_deinit	(void);
	void	hx_spi_init		(nrf_drv_spi_evt_handler_t handler);


#endif // __HX711_SPI_H__
