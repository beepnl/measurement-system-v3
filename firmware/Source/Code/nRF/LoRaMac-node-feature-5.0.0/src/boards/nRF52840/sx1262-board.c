/*!
 * \file      sx1262-board.c
 *
 * \brief     Target board implementation
 */
#include <stdlib.h>
#include "utilities.h"
#include "boards.h"
#include "board.h"
#include "delay.h"
#include "radio.h"
#include "sx126x-board.h"

/*!
 * Antenna switch GPIO pins objects
 */
Gpio_t AntPow;
Gpio_t DeviceSel;

/*!
 * Debug GPIO pins objects
 */
#if defined( USE_RADIO_DEBUG )
Gpio_t DbgPinTx;
Gpio_t DbgPinRx;
#endif

void SX126xIoInit( void )
{

}

void SX126xIoIrqInit( DioIrqHandler dioIrq )
{

}

void SX126xIoDeInit( void )
{

}

void SX126xIoDbgInit( void )
{

}

void SX126xIoTcxoInit( void )
{

}

uint32_t SX126xGetBoardTcxoWakeupTime( void )
{

}

void SX126xReset( void )
{

}

void SX126xWaitOnBusy( void )
{

}

void SX126xWakeup( void )
{

}

void SX126xWriteCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{

}

uint8_t SX126xReadCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{

}

void SX126xWriteRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{

}

void SX126xWriteRegister( uint16_t address, uint8_t value )
{

}

void SX126xReadRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{

}

uint8_t SX126xReadRegister( uint16_t address )
{

}

void SX126xWriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{

}

void SX126xReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{

}

void SX126xSetRfTxPower( int8_t power )
{
}

uint8_t SX126xGetDeviceId( void )
{

}

void SX126xAntSwOn( void )
{

}

void SX126xAntSwOff( void )
{

}

bool SX126xCheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return true;
}

#if defined( USE_RADIO_DEBUG )
void SX126xDbgPinTxWrite( uint8_t state )
{

}

void SX126xDbgPinRxWrite( uint8_t state )
{

}
#endif
