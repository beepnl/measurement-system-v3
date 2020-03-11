/*!
 * \file      spi-board.h
 *
 * \brief     Target board SPI driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#ifndef __SPI_BOARD_H__
#define __SPI_BOARD_H__

    #include <stdint.h>
    #include "spi.h"
    #include "gpio.h"

    void SpiInit	( Spi_t *obj, SpiId_t spiId, PinNames mosi, PinNames miso, PinNames sclk, PinNames nss );
    void SpiWriteBuffer	( uint8_t addr, uint8_t *buffer, uint8_t size );
    void SpiReadBuffer	( uint8_t addr, uint8_t *buffer, uint8_t size );

// An Spi.c file has to be implmented under system directory.

#endif // __SPI_BOARD_H__
