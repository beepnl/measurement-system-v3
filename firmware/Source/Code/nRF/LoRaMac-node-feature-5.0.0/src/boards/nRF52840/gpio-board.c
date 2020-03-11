/*!
 * \file      gpio-board.c
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
 *
 * \author    Marten Lootsma(TWTG) on behalf of Microchip/Atmel (c)2017
 */
#include "sdk_config.h"
#include "gpio-board.h"
#include "boards.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"
#include "nrf_drv_gpiote.h"
#include "nrf_delay.h"



void RFM_bitbangSPI(uint8_t * tx, uint8_t * rx, const uint8_t bytes)
{
	uint8_t i, j;
	uint32_t miso;

	if(tx == NULL || rx == NULL || bytes == 0){
		return;
	}
    nrf_gpio_cfg_input(RFM_A_MISO_PIN, GPIO_PIN_CNF_PULL_Disabled);
    nrf_gpio_cfg_output_state(RFM_A_MOSI_PIN, 0);
    nrf_gpio_cfg_output_state(RFM_A_SCK_PIN, 0);
    nrf_gpio_pin_set(RFM_A_NSS_PIN);
	nrf_delay_us(100);


	// Enable transmission by setting the NSS pin low
    nrf_gpio_pin_clear(RFM_A_SCK_PIN);
	nrf_gpio_pin_clear(RFM_A_NSS_PIN);

	// Repeat for the number of bytes
	for(i = 0; i < bytes; i++)
	{
		for(j=0; j < 8; j++)
		{
			// Set the bit to transfer on the MOSI pin MSbit first
			if(tx[i] & (1<<(7-j)))
			{
				nrf_gpio_pin_set(RFM_A_MOSI_PIN);
			}
			else
			{
				nrf_gpio_pin_clear(RFM_A_MOSI_PIN);
			}
			nrf_delay_us(10);
			nrf_gpio_pin_set(RFM_A_SCK_PIN);
			nrf_delay_us(10);

			// Read the miso pin and 
			miso = nrf_gpio_pin_read(RFM_A_MISO_PIN);
            rx[i] |= (miso<<(7-j));
			nrf_gpio_pin_clear(RFM_A_SCK_PIN);
		}
	}

	// Disable transmission by setting the NSS pin high
	nrf_gpio_pin_set(RFM_A_NSS_PIN);

    nrf_gpio_cfg_output_state(RFM_A_MOSI_PIN, 1);
}

#define RFM_SLEEP			0x00
#define RFM_STANDBY			0x01
#define RFM_LORA_SLEEP		0x80
#define RFM_LORA_STANDBY	0x81
void RFM_LowPowerMode(void)
{
	uint8_t mode, dioreg0, dioreg1;
	RFM_reset();
    RFM_bitBangWrite(0x01, 0x00);
    nrf_delay_ms(10);
    RFM_bitBangWrite(0x01, 0x80);
    nrf_delay_ms(10);

    RFM_setMode(RFM_LORA_STANDBY);
	nrf_delay_ms(10);


    RFM_bitBangWrite(0x40, 0);
    RFM_bitBangWrite(0x41, 0);

	RFM_setMode(RFM_LORA_SLEEP);
	nrf_delay_ms(10);
	nrf_gpio_cfg_output_state(RFM_A_MISO_PIN,		0);	
}

void RFM_setMode(uint8_t newMode)
{
	uint8_t tx[2] = {0x81, 0};
	uint8_t rx[2] = {0};
	tx[1] = newMode;
	RFM_bitbangSPI(tx, rx, 2);
}

void RFM_bitBangWrite(const uint8_t reg, const uint8_t value)
{
	uint8_t tx[2] = {0};
	uint8_t rx[2] = {0};
	tx[0] = reg | 0x80;
	tx[1] = value;
	RFM_bitbangSPI(tx, rx, 2);
}

uint8_t RFM_bitBangRead(const uint8_t reg)
{
	uint8_t tx[2] = {0};
	uint8_t rx[2] = {0};
	tx[0] = reg;
	RFM_bitbangSPI(tx, rx, 2);
	return rx[1];
}


void RFM_reset(void)
{
	nrf_gpio_pin_clear(RFM_A_RST_PIN);
	nrf_delay_ms(10);
    nrf_gpio_pin_set(RFM_A_RST_PIN);
    nrf_delay_ms(10);
}


uint8_t RFM_readMode(void)
{
	uint8_t tx[2] = {0x01, 0x00};
	uint8_t rx[2] = {0};
	RFM_bitbangSPI(tx, rx, 2);
	return rx[1];
}

void flash_ReleasePowerDown(void)
{
	nrf_gpio_pin_set(BSP_QSPI_CSN_PIN);
	nrf_delay_us(100);
	nrf_gpio_pin_clear(BSP_QSPI_CSN_PIN);
    nrf_delay_us(100);
    nrf_gpio_pin_set(BSP_QSPI_CSN_PIN);
    nrf_delay_us(100);
}


void flash_sendInstructionCode(const uint8_t instCode)
{
	uint32_t i;

	nrf_gpio_pin_clear(BSP_QSPI_SCK_PIN);
    nrf_gpio_pin_clear(BSP_QSPI_IO0_PIN);
	nrf_gpio_pin_clear(BSP_QSPI_CSN_PIN);
    flash_ReleasePowerDown();

	// Clear the CS pin to start the data transfer.
    nrf_gpio_pin_clear(BSP_QSPI_CSN_PIN);

	for(i=0; i<8; i++)
	{
		if(instCode & (1<<(7 - i)))
		{
			nrf_gpio_pin_set(BSP_QSPI_IO0_PIN);
		}
		else
		{
			nrf_gpio_pin_clear(BSP_QSPI_IO0_PIN);
		}
        nrf_delay_us(10);
		nrf_gpio_pin_set(BSP_QSPI_SCK_PIN);
        nrf_delay_us(10);
        nrf_gpio_pin_clear(BSP_QSPI_SCK_PIN);
	}

    nrf_gpio_pin_set(BSP_QSPI_CSN_PIN);
}

uint16_t flash_ReadStatusRegister(void)
{
	uint32_t i, pinInput;
    uint16_t ret = 0;
    const uint8_t instCode = 0x05;
	
	nrf_gpio_pin_clear(BSP_QSPI_SCK_PIN);
    nrf_gpio_pin_clear(BSP_QSPI_IO0_PIN);
	nrf_gpio_pin_clear(BSP_QSPI_CSN_PIN);
	nrf_gpio_cfg_output(BSP_QSPI_IO0_PIN);
	nrf_gpio_cfg_input(BSP_QSPI_IO1_PIN, NRF_GPIO_PIN_NOPULL);
    flash_ReleasePowerDown();

	// Clear the CS pin to start the data transfer.
    nrf_gpio_pin_clear(BSP_QSPI_CSN_PIN);

    // Write the instruction
	for(i=0; i<8; i++)
	{
		if(instCode & (1<<(7 - i)))
		{
			nrf_gpio_pin_set(BSP_QSPI_IO0_PIN);
		}
		else
		{
			nrf_gpio_pin_clear(BSP_QSPI_IO0_PIN);
		}
        nrf_delay_us(10);
		nrf_gpio_pin_set(BSP_QSPI_SCK_PIN);
        nrf_delay_us(10);
        nrf_gpio_pin_clear(BSP_QSPI_SCK_PIN);
	}

    nrf_gpio_pin_clear(BSP_QSPI_IO0_PIN);

    // Read the satus register
    for(i=0; i<16; i++)
	{
        pinInput = nrf_gpio_pin_read(BSP_QSPI_IO1_PIN);
		if(pinInput)
		{
			ret |= (1<<(15-i));
		}

        // Clock the next bit
        nrf_delay_us(10);
		nrf_gpio_pin_set(BSP_QSPI_SCK_PIN);
        nrf_delay_us(10);
        nrf_gpio_pin_clear(BSP_QSPI_SCK_PIN);
	}

    nrf_gpio_pin_set(BSP_QSPI_CSN_PIN);
	return ret;
}


void flash_deepPowerDown(void)
{
	flash_sendInstructionCode(0xB9);
}


void flash_erase_all(void)
{

}


void GpioMcuInit( void )
{
	// Power control
	nrf_gpio_cfg_output_state(TPS_PWR_ENABLE,		0);
	nrf_gpio_cfg_output_state(TPS_SW_ENABLE,		0);
    nrf_gpio_cfg_input(BAT_MEAS_PIN, GPIO_PIN_CNF_PULL_Disabled);

    // Configure the IO of the RFM module: NSS and RST pins as output and high, DIO as inputs.
	#if RFM_ENABLE
		nrf_gpio_cfg_input(RFM_A_DIO_0_PIN, GPIO_PIN_CNF_PULL_Disabled);
		nrf_gpio_cfg_input(RFM_A_DIO_1_PIN, GPIO_PIN_CNF_PULL_Disabled);
		nrf_gpio_cfg_input(RFM_A_DIO_2_PIN, GPIO_PIN_CNF_PULL_Disabled);
		nrf_gpio_cfg_input(RFM_A_DIO_3_PIN, GPIO_PIN_CNF_PULL_Disabled);
		nrf_gpio_cfg_input(RFM_A_DIO_4_PIN, GPIO_PIN_CNF_PULL_Disabled);
		nrf_gpio_cfg_input(RFM_A_DIO_5_PIN, GPIO_PIN_CNF_PULL_Disabled);

		// SPI mode 0: SCK active high, sample on leading edge.
		nrf_gpio_cfg_output_state(RFM_A_SCK_PIN,		0);
		nrf_gpio_cfg_output_state(RFM_A_NSS_PIN,		1);

		// Reset the RFM to a known state.
		nrf_gpio_cfg_output_state(RFM_A_RST_PIN,		1);
		RFM_reset();
	#else
        nrf_gpio_cfg_output_state(RFM_A_DIO_0_PIN,	0);
		nrf_gpio_cfg_output_state(RFM_A_DIO_1_PIN,	0);
		nrf_gpio_cfg_output_state(RFM_A_DIO_2_PIN,	0);
		nrf_gpio_cfg_output_state(RFM_A_DIO_3_PIN,	0);
		nrf_gpio_cfg_output_state(RFM_A_DIO_4_PIN,	0);
		nrf_gpio_cfg_output_state(RFM_A_DIO_5_PIN,	0);
		nrf_gpio_cfg_output_state(RFM_A_SCK_PIN,	0);
        nrf_gpio_cfg_output_state(RFM_A_MOSI_PIN,	0);
        nrf_gpio_cfg_output_state(RFM_A_MISO_PIN,	0);
		nrf_gpio_cfg_output_state(RFM_A_NSS_PIN,	0);
		nrf_gpio_cfg_output_state(RFM_A_RST_PIN,	0);
		RFM_reset();
	#endif

	// Reedswitch
    nrf_gpio_cfg_input(BUTTON_1, GPIO_PIN_CNF_PULL_Pullup);


	// Buzzer PWM pin
    nrf_gpio_cfg_output_state(BUZZER_PWM,			0);

    // DS18B20
	nrf_gpio_cfg_output_state(OWI_PIN,				0);

    // SQ movement sensors
	nrf_gpio_cfg_input(SQ_SEN_645B_PIN, GPIO_PIN_CNF_PULL_Disabled);

	// I2C extern: BME280
	nrf_gpio_cfg_output_state(SCL_EXT,				0);
    nrf_gpio_cfg_output_state(SDA_EXT,				0);

    // I2C intern: TLV320ADC3100, ATEC6088A
	nrf_gpio_cfg_output_state(SCL_INT,				0);
    nrf_gpio_cfg_output_state(SDA_INT,				0);

	// TLV320ADC3100
    nrf_gpio_cfg_output_state(I2S_MCLK,				0);
    nrf_gpio_cfg_output_state(I2S_BCLK,				0);
    nrf_gpio_cfg_output_state(I2S_LRCLK,			0);
    nrf_gpio_cfg_output_state(I2S_DOUT,				0);
    nrf_gpio_cfg_output_state(ADC_RST,				0);

	// HX711
	nrf_gpio_cfg_output_state(HX711_RATE,			0);
	nrf_gpio_cfg_output_state(HX711_DOUT,			0);
	nrf_gpio_cfg_output_state(HX711_PD_SCK,			0);

	// Flash chip
	#if MX_FLASH_ENABLE
		nrf_gpio_cfg_output_state(BSP_QSPI_SCK_PIN,		0);
		nrf_gpio_cfg_output_state(BSP_QSPI_CSN_PIN,		1);
		nrf_gpio_cfg_output_state(BSP_QSPI_IO0_PIN,		0);
		nrf_gpio_cfg_input(BSP_QSPI_IO1_PIN,			GPIO_PIN_CNF_PULL_Disabled);
		nrf_gpio_cfg_output_state(BSP_QSPI_IO2_PIN,		1);
		nrf_gpio_cfg_output_state(BSP_QSPI_IO3_PIN,		1);

		flash_deepPowerDown();
	#else

		nrf_gpio_cfg_output_state(BSP_QSPI_SCK_PIN,		0);
		nrf_gpio_cfg_output_state(BSP_QSPI_CSN_PIN,		0);
		nrf_gpio_cfg_output_state(BSP_QSPI_IO0_PIN,		0);
		nrf_gpio_cfg_output_state(BSP_QSPI_IO1_PIN,		0);
		nrf_gpio_cfg_output_state(BSP_QSPI_IO2_PIN,		0);
		nrf_gpio_cfg_output_state(BSP_QSPI_IO3_PIN,		0);
	#endif
}


void GPIO_switchOFF(void)
{
	// I2C extern: BME280
	nrf_gpio_cfg_output_state(SCL_EXT,		0);
    nrf_gpio_cfg_output_state(SDA_EXT,		0);

    // I2C intern: TLV320ADC3100, ATEC6088A
	nrf_gpio_cfg_output_state(SCL_INT,		0);
    nrf_gpio_cfg_output_state(SDA_INT,		0);

	// I2S: TLV320ADC3100
    nrf_gpio_cfg_output_state(ADC_RST,		0);

    // DS18B20
	nrf_gpio_cfg_output_state(OWI_PIN,		0);

	// HX711
    nrf_gpio_cfg_output_state(HX711_DOUT,	0);
	nrf_gpio_cfg_output_state(HX711_PD_SCK,	0);
    nrf_gpio_cfg_output_state(HX711_RATE,	0);
}

void GPIO_TLVreset(void)
{
    nrf_gpio_cfg_output_state(ADC_RST,		1);
	nrf_delay_us(100);
    nrf_gpio_cfg_output_state(ADC_RST,		0);
    nrf_delay_us(100);
    nrf_gpio_cfg_output_state(ADC_RST,		1);
}

void TLV_reset(void)
{
    nrf_gpio_cfg_output_state(ADC_RST,		1);
	nrf_delay_ms(1);
    nrf_gpio_cfg_output_state(ADC_RST,		0);
    nrf_delay_ms(1);
    nrf_gpio_cfg_output_state(ADC_RST,		1);
}


void GPIO_switchON(void)
{
	// I2C extern: BME280
	nrf_gpio_cfg_input(SCL_EXT,				GPIO_PIN_CNF_PULL_Disabled);
    nrf_gpio_cfg_input(SDA_EXT,				GPIO_PIN_CNF_PULL_Disabled);

    // I2C intern: TLV320ADC3100, ATEC6088A
	nrf_gpio_cfg_input(SCL_INT,				GPIO_PIN_CNF_PULL_Disabled);
    nrf_gpio_cfg_input(SDA_INT,				GPIO_PIN_CNF_PULL_Disabled);

	// I2S: TLV320ADC3100
    TLV_reset();

    // DS18B20
	nrf_gpio_cfg_input(OWI_PIN,				GPIO_PIN_CNF_PULL_Disabled);

	// HX711 in off mode
    nrf_gpio_cfg_input(HX711_DOUT, NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_output_state(HX711_PD_SCK,	1);
    nrf_gpio_cfg_output_state(HX711_RATE,	0);
}

void GpioBoostConverterSet(uint32_t enable)
{
	if(enable)
	{
		nrf_gpio_pin_set(TPS_PWR_ENABLE);
	}
	else
	{
		nrf_gpio_pin_clear(TPS_PWR_ENABLE);
	}
}

uint32_t GpioBoostConverterGet(void)
{
	nrf_gpio_pin_out_read(TPS_PWR_ENABLE);
}

void GpioPowerSwitchSet(uint32_t enable)
{
	if(enable)
	{
		nrf_gpio_pin_set(TPS_SW_ENABLE);
	}
	else
	{
		nrf_gpio_pin_clear(TPS_SW_ENABLE);
	}
}

uint32_t GpioPowerSwitchGet(void)
{
	nrf_gpio_pin_out_read(TPS_SW_ENABLE);
}

void GpioIntInit(void)
{
    uint32_t err_code;
    if(nrf_drv_gpiote_is_init())
    {
		return;
    }

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);
}

void GpioMcuSetContext( Gpio_t *obj, void* context )
{
//    obj->Context = context;
}

void GpioMcuSetInterrupt( uint32_t pin, nrfx_gpiote_evt_handler_t evt_handler)
{
    uint32_t err_code;

    /*  Pin change interrupt event configuration. For low power consumption the accuracy must be set to false and in the interrupt must be checked whether the pin is high or low. */
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
    in_config.hi_accuracy = false;

	// Only enable the GPIO pin interrupt when the pin isn' already in use.
	if(!pin_in_use_by_gpiote(pin))
	{
		err_code = nrf_drv_gpiote_in_init(pin, &in_config, evt_handler);
		APP_ERROR_CHECK(err_code);
	}

    nrf_drv_gpiote_in_event_enable(pin, true);
}



void GpioMcuRemoveInterrupt( uint32_t pin )
{
	if(pin_in_use_by_gpiote(pin))
	{
		nrfx_gpiote_in_event_disable(pin);
	}
}

void GpioMcuWrite( Gpio_t *obj, uint32_t value )
{
//
//    if( obj == NULL )
//    {
//        //assert_param( FAIL );
//        while( 1 );
//    }
//    // Check if pin is not connected
//    if( obj->pin == NC )
//    {
//        return;
//    }
//    gpio_set_pin_level( obj->pin, value );
}

void GpioMcuToggle( Gpio_t *obj )
{
//    if( obj == NULL )
//    {
//        //assert_param( FAIL );
//        while( 1 );
//    }
//
//    // Check if pin is not connected
//    if( obj->pin == NC )
//    {
//        return;
//    }
//    gpio_toggle_pin_level( obj->pin );
}

uint32_t GpioMcuRead( Gpio_t *obj )
{
//    if( obj == NULL )
//    {
//        //assert_param( FAIL );
//        while( 1 );
//    }
//    // Check if pin is not connected
//    if( obj->pin == NC )
//    {
//        return 0;
//    }
//    return ( uint32_t )gpio_get_pin_level( obj->pin );
}
