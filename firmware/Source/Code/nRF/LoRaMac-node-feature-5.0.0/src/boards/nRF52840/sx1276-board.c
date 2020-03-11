/*!
 * \file      sx1276-board.c
 *
 * \brief     Target board SX1276 driver implementation
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
 *
 * \author    Marten Lootsma(TWTG) on behalf of Microchip/Atmel (c)2017
 */
#include <string.h>
#include "boards.h"
#include "delay.h"
#include "radio.h"
#include "sx1276-board.h"
#include "spi-board.h"
#include "gpio-board.h"

/*!
 * \brief Gets the board PA selection configuration
 *
 * \param [IN] channel Channel frequency in Hz
 * \retval PaSelect RegPaConfig PaSelect value
 */
static uint8_t SX1276GetPaSelect( uint32_t channel );

/*!
 * Flag used to set the RF switch control pins in low power mode when the radio is not active.
 */
static bool RadioIsActive = false;

/*!
 * Radio driver structure initialization
 */
const struct Radio_s Radio =
{
    SX1276Init,
    SX1276GetStatus,
    SX1276SetModem,
    SX1276SetChannel,
    SX1276IsChannelFree,
    SX1276Random,
    SX1276SetRxConfig,
    SX1276SetTxConfig,
    SX1276CheckRfFrequency,
    SX1276GetTimeOnAir,
    SX1276Send,
    SX1276SetSleep,
    SX1276SetStby,
    SX1276SetRx,
    SX1276StartCad,
    SX1276SetTxContinuousWave,
    SX1276ReadRssi,
    SX1276Write,
    SX1276Read,
    SX1276WriteBuffer,
    SX1276ReadBuffer,
    SX1276SetMaxPayloadLength,
    SX1276SetPublicNetwork,
    SX1276GetWakeupTime,
    NULL, // void ( *IrqProcess )( void )
    NULL, // void ( *RxBoosted )( uint32_t timeout ) - SX126x Only
    NULL, // void ( *SetRxDutyCycle )( uint32_t rxTime, uint32_t sleepTime ) - SX126x Only
};

/*!
 * Debug GPIO pins objects
 */
#if defined( USE_RADIO_DEBUG )
Gpio_t DbgPinTx;
Gpio_t DbgPinRx;
#endif

void SX1276IoInit( void )
{
//    GpioInit( &SX1276.Spi.Nss, RADIO_NSS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
////
////    ext_irq_init( );
////
//    GpioInit( &SX1276.DIO0, RADIO_DIO_0, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//    GpioInit( &SX1276.DIO1, RADIO_DIO_1, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//    GpioInit( &SX1276.DIO2, RADIO_DIO_2, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//    GpioInit( &SX1276.DIO3, RADIO_DIO_3, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//    GpioInit( &SX1276.DIO4, RADIO_DIO_4, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//    GpioInit( &SX1276.DIO5, RADIO_DIO_5, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
}


static Gpio_t *DioIrqs[] = {
    &SX1276.DIO0,
    &SX1276.DIO1,
    &SX1276.DIO2,
    &SX1276.DIO3,
    &SX1276.DIO4,
    &SX1276.DIO5
};
//typedef void (*ext_irq_cb_t) (void);
//static ext_irq_cb_t ExtIrqHandlers[] = {
//    Dio0IrqHandler,
//    Dio1IrqHandler,
//    Dio2IrqHandler,
//    Dio3IrqHandler,
//    Dio4IrqHandler,
//    Dio5IrqHandler
//};

static void DioIrqHanlderProcess( uint8_t index )
{
    if( ( DioIrqs[index] != NULL ) && ( DioIrqs[index]->IrqHandler != NULL ) )
    {
        DioIrqs[index]->IrqHandler( DioIrqs[index]->Context );
    }
}


void dio_evt_handler (nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    uint32_t pin_state = nrf_gpio_pin_read(pin);

    // Only handle falling edge cases.
    if(!pin_state){
		return;
    }

    switch(pin)
    {
	case RFM_A_DIO_0_PIN:
	    DioIrqHanlderProcess( 0 );
	    break;
	case RFM_A_DIO_1_PIN:
	    DioIrqHanlderProcess( 1 );
	    break;

#ifdef RFM_A_DIO_2_PIN
	case RFM_A_DIO_2_PIN:
		DioIrqHanlderProcess( 2 );
		break;
#endif

#ifdef RFM_A_DIO_3_PIN	
	case RFM_A_DIO_3_PIN:
		DioIrqHanlderProcess( 3 );
		break;
#endif

#ifdef RFM_A_DIO_4_PIN	
	case RFM_A_DIO_4_PIN:
		DioIrqHanlderProcess( 4 );
		break;
#endif

    case RFM_A_DIO_5_PIN:
	    DioIrqHanlderProcess( 5 );
	    break;
	default:
	    break;
    }
}

static void IoIrqInit( uint8_t index, DioIrqHandler *irqHandler )
{
    DioIrqs[index]->IrqHandler = irqHandler;
    //ext_irq_register( DioIrqs[index]->pin, ExtIrqHandlers[index] );
}

void SX1276IoIrqInit( DioIrqHandler **irqHandlers )
{
    // Connect the Callbacks to the GPIO pins
    for( int8_t i = 0; i < 5; i++ )
    {
        IoIrqInit( i, irqHandlers[i] );
    }

    GpioMcuSetInterrupt(RFM_A_DIO_0_PIN, dio_evt_handler);
    GpioMcuSetInterrupt(RFM_A_DIO_1_PIN, dio_evt_handler);

#ifdef RFM_A_DIO_2_PIN
	GpioMcuSetInterrupt(RFM_A_DIO_2_PIN, dio_evt_handler);
#endif

#ifdef RFM_A_DIO_3_PIN
	GpioMcuSetInterrupt(RFM_A_DIO_3_PIN, dio_evt_handler);
#endif

#ifdef RFM_A_DIO_4_PIN
	GpioMcuSetInterrupt(RFM_A_DIO_4_PIN, dio_evt_handler);
#endif

    GpioMcuSetInterrupt(RFM_A_DIO_5_PIN, dio_evt_handler);
}

void SX1276IoDeInit( void )
{
    GpioMcuRemoveInterrupt(RFM_A_DIO_0_PIN);
    GpioMcuRemoveInterrupt(RFM_A_DIO_1_PIN);

#ifdef RFM_A_DIO_2_PIN
	GpioMcuRemoveInterrupt(RFM_A_DIO_2_PIN);
#endif

#ifdef RFM_A_DIO_3_PIN
	GpioMcuRemoveInterrupt(RFM_A_DIO_3_PIN);
#endif

#ifdef RFM_A_DIO_4_PIN
	GpioMcuRemoveInterrupt(RFM_A_DIO_4_PIN);
#endif

    GpioMcuRemoveInterrupt(RFM_A_DIO_5_PIN);
}

void SX1276IoDbgInit( void )
{
#if defined( USE_RADIO_DEBUG )
    GpioInit( &DbgPinTx, RADIO_DBG_PIN_TX, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &DbgPinRx, RADIO_DBG_PIN_RX, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
#endif
}

void SX1276IoTcxoInit( void )
{
    // No TCXO component available on this board design.
}

void SX1276SetBoardTcxo( uint8_t state )
{
    // No TCXO component available on this board design.
#if 0
    if( state == true )
    {
        TCXO_ON( );
        DelayMs( BOARD_TCXO_WAKEUP_TIME );
    }
    else
    {
        TCXO_OFF( );
    }
#endif
}

#define BOARD_TCXO_WAKEUP_TIME 0
uint32_t SX1276GetBoardTcxoWakeupTime( void )
{
    return BOARD_TCXO_WAKEUP_TIME;
}

void SX1276Reset( void )
{
	SX1276IoDeInit();

    // Enables the TCXO if available on the board design
    SX1276SetBoardTcxo( true );

    // Set RESET pin to 0
    nrf_gpio_pin_clear(RFM_A_RST_PIN);

    // Wait 1 ms
    DelayMs( 1 );

    // Configure RESET as input
    nrf_gpio_pin_set(RFM_A_RST_PIN);

    // Wait 6 ms
    DelayMs( 6 );
}

void SX1276SetRfTxPower( int8_t power )
{
    uint8_t paConfig = 0;
    uint8_t paDac = 0;

    paConfig = SX1276Read( REG_PACONFIG );
    paDac = SX1276Read( REG_PADAC );

    paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | SX1276GetPaSelect( SX1276.Settings.Channel );

    if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
    {
        if( power > 17 )
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON;
        }
        else
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;
        }
        if( ( paDac & RF_PADAC_20DBM_ON ) == RF_PADAC_20DBM_ON )
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
        }
        else
        {
            if( power < 2 )
            {
                power = 2;
            }
            if( power > 17 )
            {
                power = 17;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else
    {
        if( power > 0 )
        {
            if( power > 15 )
            {
                power = 15;
            }
            paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( 7 << 4 ) | ( power );
        }
        else
        {
            if( power < -4 )
            {
                power = -4;
            }
            paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( 0 << 4 ) | ( power + 4 );
        }
    }
    SX1276Write( REG_PACONFIG, paConfig );
    SX1276Write( REG_PADAC, paDac );
}

static uint8_t SX1276GetPaSelect( uint32_t channel )
{
    return RF_PACONFIG_PASELECT_PABOOST;
}

void SX1276SetAntSwLowPower( bool status )
{
    // No antenna switch available.
    // Just control the TCXO if available.
    if( RadioIsActive != status )
    {
        RadioIsActive = status;
    }
}

void SX1276SetAntSw( uint8_t opMode )
{
    // No antenna switch available
}

bool SX1276CheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return true;
}

#if defined( USE_RADIO_DEBUG )
void SX1276DbgPinTxWrite( uint8_t state )
{
    GpioWrite( &DbgPinTx, state );
}

void SX1276DbgPinRxWrite( uint8_t state )
{
    GpioWrite( &DbgPinRx, state );
}
#endif
