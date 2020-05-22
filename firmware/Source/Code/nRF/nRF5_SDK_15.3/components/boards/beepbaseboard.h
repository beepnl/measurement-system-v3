#ifndef BEEPBASEBOARD_H
#define BEEPBASEBOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_gpio.h"



// LEDs definitions
#define LEDS_NUMBER					0
#define LED_1						NRF_GPIO_PIN_MAP(0,13)
#define LED_2						NRF_GPIO_PIN_MAP(0,14)
#define LED_3						NRF_GPIO_PIN_MAP(0,15)
#define LED_4						NRF_GPIO_PIN_MAP(0,16)
#define LED_START					LED_1
#define LED_STOP					LED_4
#define LEDS_ACTIVE_STATE			0
#define LEDS_LIST { }
#define LEDS_INV_MASK				LEDS_MASK

// Button definitions
#define BUTTONS_NUMBER				1
#define BUTTON_1					NRF_GPIO_PIN_MAP(0, 11)	// Reed switch. Requires pull-up
#define BUTTON_PULL					NRF_GPIO_PIN_PULLUP
#define BUTTONS_ACTIVE_STATE		0
#define BUTTONS_LIST				{BUTTON_1}
#define BSP_BUTTON_0				BUTTON_1

#define RX_PIN_NUMBER				NRF_GPIO_PIN_MAP(0, 8)
#define TX_PIN_NUMBER				NRF_GPIO_PIN_MAP(0, 6)
#define CTS_PIN_NUMBER				NRF_GPIO_PIN_MAP(0, 7)
#define RTS_PIN_NUMBER				NRF_GPIO_PIN_MAP(0, 5)
#define HWFC						true


// Power control
#define TPS_PWR_ENABLE				NRF_GPIO_PIN_MAP(0, 25)
#define TPS_SW_ENABLE				NRF_GPIO_PIN_MAP(1, 5)
#define BAT_MEAS_PIN				NRF_GPIO_PIN_MAP(0,	28)

// Buzzer PWM pin
#define BUZZER_PWM					NRF_GPIO_PIN_MAP(0, 24)

// DS18B20
#define OWI_PIN						NRF_GPIO_PIN_MAP(1, 12)

// SQ-SEN-645B
#define SQ_SEN_645B_PIN				NRF_GPIO_PIN_MAP(0, 12)

// I2C extern: BME280
#define SCL_EXT						NRF_GPIO_PIN_MAP(1, 9)
#define SDA_EXT						NRF_GPIO_PIN_MAP(1, 8)

// I2C intern: TLV320ADC3100, ATEC6088A
#define SCL_INT						NRF_GPIO_PIN_MAP(0, 26)
#define SDA_INT						NRF_GPIO_PIN_MAP(0, 27)

// TLV320ADC3100
#define I2S_MCLK					NRF_GPIO_PIN_MAP(1, 13)
#define I2S_BCLK					NRF_GPIO_PIN_MAP(0, 6)
#define I2S_LRCLK					NRF_GPIO_PIN_MAP(0, 7)
#define I2S_DOUT					NRF_GPIO_PIN_MAP(0, 8)
#define ADC_RST						NRF_GPIO_PIN_MAP(0, 9)

// HX711
#define HX711_CLK_NC				NRF_GPIO_PIN_MAP(0, 3)
#define HX711_RATE					NRF_GPIO_PIN_MAP(0, 4)
#define HX711_DOUT					NRF_GPIO_PIN_MAP(0, 5)
#define HX711_PD_SCK				NRF_GPIO_PIN_MAP(0, 10)


// Flash chip
#define BSP_QSPI_SCK_PIN			NRF_GPIO_PIN_MAP(0, 19)
#define BSP_QSPI_CSN_PIN			NRF_GPIO_PIN_MAP(0, 17)
#define BSP_QSPI_IO0_PIN			NRF_GPIO_PIN_MAP(0, 20)
#define BSP_QSPI_IO1_PIN			NRF_GPIO_PIN_MAP(0, 21)
#define BSP_QSPI_IO2_PIN			NRF_GPIO_PIN_MAP(0, 22)
#define BSP_QSPI_IO3_PIN			NRF_GPIO_PIN_MAP(0, 23)


// RFM pin definitions
#define RFM_A_MISO_PIN      		NRF_GPIO_PIN_MAP(0, 16)
#define RFM_A_MOSI_PIN      		NRF_GPIO_PIN_MAP(0, 15)
#define RFM_A_SCK_PIN       		NRF_GPIO_PIN_MAP(0, 14)
#define RFM_A_NSS_PIN       		NRF_GPIO_PIN_MAP(0, 13)
#define RFM_A_RST_PIN       		NRF_GPIO_PIN_MAP(1, 15)
#define RFM_A_DIO_0_PIN     		NRF_GPIO_PIN_MAP(1, 2)
#define RFM_A_DIO_1_PIN     		NRF_GPIO_PIN_MAP(1, 1)
#define RFM_A_DIO_2_PIN     		NRF_GPIO_PIN_MAP(1, 0)
#define RFM_A_DIO_3_PIN     		NRF_GPIO_PIN_MAP(1, 4)
#define RFM_A_DIO_4_PIN     		NRF_GPIO_PIN_MAP(1, 3)
#define RFM_A_DIO_5_PIN     		NRF_GPIO_PIN_MAP(1, 14)




#ifdef __cplusplus
}
#endif

#endif // BEEPBASEBOARD_H
