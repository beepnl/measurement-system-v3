/*!
 * \file      gpio-board.h
 *
 * \brief     Target board GPIO driver implementation
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
#ifndef __GPIO_BOARD_H__
#define __GPIO_BOARD_H__

#include "gpio.h"
#include "nrf_gpiote.h"
#include "nrf_drv_gpiote.h"

/*!
 * \brief Initializes the given GPIO object
 *
 * \param [IN] obj    Pointer to the GPIO object
 * \param [IN] pin    Pin name ( please look in pinName-board.h file )
 * \param [IN] mode   Pin mode [PIN_INPUT, PIN_OUTPUT,
 *                              PIN_ALTERNATE_FCT, PIN_ANALOGIC]
 * \param [IN] config Pin config [PIN_PUSH_PULL, PIN_OPEN_DRAIN]
 * \param [IN] type   Pin type [PIN_NO_PULL, PIN_PULL_UP, PIN_PULL_DOWN]
 * \param [IN] value  Default output value at initialization
 */
void GpioMcuInit( void );



void GpioIntInit(void);


/*!
 * \brief Sets a user defined object pointer
 *
 * \param [IN] context User defined data object pointer to pass back
 *                     on IRQ handler callback
 */
 void		GPIO_TLVreset		(void);
 void		GPIO_switchOFF		(void);
 void		GPIO_switchON		(void);

// RFM bit bang mode to power down the RFM 
void		RFM_bitBangWrite    (const uint8_t reg, const uint8_t value);
uint8_t		RFM_bitBangRead     (const uint8_t reg);
void        flash_deepPowerDown (void);
void		RFM_LowPowerMode	(void);
uint8_t		RFM_readMode		(void);
void		RFM_reset			(void);
void		RFM_setMode			(uint8_t newMode);
void        TLV_reset           (void);
uint16_t	flash_ReadStatusRegister(void);


// Boost converter functions
void		GpioBoostConverterSet(uint32_t enable);
uint32_t	GpioBoostConverterGet(void);

// Power switch functions
void		GpioPowerSwitchSet(uint32_t enable);
uint32_t	GpioPowerSwitchGet(void);

void GpioMcuSetContext( Gpio_t *obj, void* context );

/*!
 * \brief GPIO IRQ Initialization
 *
 * \param [IN] obj         Pointer to the GPIO object
 * \param [IN] irqHandler  Callback function pointer
 */
void GpioMcuSetInterrupt( uint32_t pin, nrfx_gpiote_evt_handler_t evt_handler);

/*!
 * \brief Removes the interrupt from the object
 *
 * \param [IN] obj Pointer to the GPIO object
 */
void GpioMcuRemoveInterrupt(uint32_t pin);


/*!
 * \brief Writes the given value to the GPIO output
 *
 * \param [IN] obj   Pointer to the GPIO object
 * \param [IN] value New GPIO output value
 */
void GpioMcuWrite( Gpio_t *obj, uint32_t value );

/*!
 * \brief Toggle the value to the GPIO output
 *
 * \param [IN] obj   Pointer to the GPIO object
 */
void GpioMcuToggle( Gpio_t *obj );

/*!
 * \brief Reads the current GPIO input value
 *
 * \param [IN] obj Pointer to the GPIO object
 * \retval value   Current GPIO input value
 */
uint32_t GpioMcuRead( Gpio_t *obj );

#endif // __GPIO_BOARD_H__
