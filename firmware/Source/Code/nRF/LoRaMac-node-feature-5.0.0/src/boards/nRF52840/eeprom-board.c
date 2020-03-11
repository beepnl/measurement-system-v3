/*!
 * \file      eeprom-board.c
 *
 * \brief     Target board EEPROM driver implementation
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
#include "nrf.h"
#include "utilities.h"
#include "eeprom-board.h"
#include "nvm_fs.h"

uint8_t EepromMcuWriteBuffer( uint16_t addr, uint8_t *buffer, uint16_t size )
{
	nvm_fds_eeprom_wrapper_write( addr, buffer, size);
    nvm_fds_changed();
    return SUCCESS;
}

uint8_t EepromMcuReadBuffer( uint16_t addr, uint8_t *buffer, uint16_t size )
{
	nvm_fds_eeprom_wrapper_read( addr, buffer, size);;
    return SUCCESS;
}

void EepromMcuSetDeviceAddr( uint8_t addr )
{
//    assert_param( FAIL );
}

uint8_t EepromMcuGetDeviceAddr( void )
{
//    assert_param( FAIL );
    return 0;
}
