/**
 * Copyright (c) 2017 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */



#ifndef SDK_CONFIG_H
#define SDK_CONFIG_H

#include <stdbool.h>

// <<< Use Configuration Wizard in Context Menu >>>\n
#ifdef USE_APP_CONFIG
#include "app_config.h"
#endif

// Options
#define RFM_ENABLE                      1
#define MX_FLASH_ENABLE                 1
#define BUZZER_ENABLE                   1
#define BUZZER_START_UP_MELODY_EN       1
#define BLE_ADV_START_UP_ENABLE         1
#define BLE_ADV_START_STOP              0
#define BLE_ADV_ENABLE                  1
#define ATECC_ENABLED                   1
#define BLE_PASSKEY_ENABLED             1
#define LORAWAN_JOIN_ENABLE             true
#define TLV_ENABLE                      1
#define BME280_ENABLE                   1
#define SEND_FIRMWARE_VERSION_ON_JOIN   1


// <h> nRF_Log 
//==========================================================
// <e> NRF_LOG_ENABLED - nrf_log - Logger
//==========================================================
#ifdef DEBUG
	#define NRF_LOG_ENABLED							1
    #define NRF_LOG_BACKEND_UART_ENABLED			0
    #define NRF_LOG_BACKEND_RTT_ENABLED				1
    #define NRF_LOG_USES_TIMESTAMP					1
	#define	NRF_LOG_HIGH_ACCURACY_TIMESTAMP			0
#else
	#define NRF_LOG_ENABLED							0
    #define NRF_LOG_BACKEND_UART_ENABLED			0
    #define NRF_LOG_BACKEND_RTT_ENABLED				0
    #define NRF_LOG_USES_TIMESTAMP					0
	#define NRF_LOG_HIGH_ACCURACY_TIMESTAMP			0
#endif



#define NRF_LOG_BACKEND_RTT_TEMP_BUFFER_SIZE        256
#define NRF_LOG_BACKEND_RTT_TX_RETRY_DELAY_MS       1
#define NRF_LOG_BACKEND_RTT_TX_RETRY_CNT            3

#define NRF_LOG_BACKEND_UART_TX_PIN                 6
#define NRF_LOG_BACKEND_UART_BAUDRATE               30801920	    // 115200 baud
#define NRF_LOG_BACKEND_UART_TEMP_BUFFER_SIZE       64



// <o> NRF_LOG_TIMESTAMP_DEFAULT_FREQUENCY - Default frequency of the timestamp (in Hz) or 0 to use app_timer frequency. 
#if NRF_LOG_HIGH_ACCURACY_TIMESTAMP
	#define NRF_LOG_TIMESTAMP_DEFAULT_FREQUENCY 1E6
#else
	#define NRF_LOG_TIMESTAMP_DEFAULT_FREQUENCY 0 // app_timer frequency
#endif


// <h> segger_rtt - SEGGER RTT

//==========================================================
// <o> SEGGER_RTT_CONFIG_BUFFER_SIZE_UP - Size of upstream buffer. 
// <i> Note that either @ref NRF_LOG_BACKEND_RTT_OUTPUT_BUFFER_SIZE
// <i> or this value is actually used. It depends on which one is bigger.
#define SEGGER_RTT_CONFIG_BUFFER_SIZE_UP		512
#define SEGGER_RTT_CONFIG_MAX_NUM_UP_BUFFERS	2
#define SEGGER_RTT_CONFIG_BUFFER_SIZE_DOWN		16
#define SEGGER_RTT_CONFIG_MAX_NUM_DOWN_BUFFERS	2
#define SEGGER_RTT_CONFIG_DEFAULT_MODE			0


// <h> nRF_Drivers 


// <e> NRF_CLOCK_ENABLED - nrf_drv_clock - CLOCK peripheral driver - legacy layer
//==========================================================
#define NRF_CLOCK_ENABLED						1
#define CLOCK_CONFIG_LF_SRC						1
#define CLOCK_CONFIG_IRQ_PRIORITY				6

#define APP_TIMER_ENABLED						1
#define APP_TIMER_CONFIG_RTC_FREQUENCY			0 // 32768 Hz
#define APP_TIMER_CONFIG_IRQ_PRIORITY			6
#define APP_TIMER_CONFIG_OP_QUEUE_SIZE			10
#define APP_TIMER_CONFIG_USE_SCHEDULER			0
#define APP_TIMER_KEEPS_RTC_ACTIVE				1 // <i> If option is enabled RTC is kept running even if there is no active timers.
#define APP_TIMER_SAFE_WINDOW_MS				300000
#define APP_TIMER_WITH_PROFILER					0
#define APP_TIMER_CONFIG_SWI_NUMBER				0



// <e> SPI_ENABLED - nrf_drv_spi - SPI/SPIM peripheral driver - legacy layer
//==========================================================
#define SPI_ENABLED								1
#define SPI_DEFAULT_CONFIG_IRQ_PRIORITY			6
#define NRF_SPI_DRV_MISO_PULLUP_CFG				1
#define SPI0_ENABLED							0
#define SPI0_USE_EASY_DMA						0
#define SPI1_ENABLED							0
#define SPI1_USE_EASY_DMA						0
#define SPI2_ENABLED							1
#define SPI2_USE_EASY_DMA						1
#define NRFX_SPIM_ENABLED						1
#define NRFX_SPIM0_ENABLED						0
#define NRFX_SPIM1_ENABLED						0
#define NRFX_SPIM2_ENABLED						1
#define NRFX_SPIM3_ENABLED						1
#define NRFX_SPIM_EXTENDED_ENABLED				0
#define NRFX_SPIM_MISO_PULL_CFG					1
#define NRFX_SPIM_DEFAULT_CONFIG_IRQ_PRIORITY	6
#define NRFX_SPIM_CONFIG_LOG_ENABLED			0
#define NRFX_SPIM_CONFIG_LOG_LEVEL				3
#define NRFX_SPIM_CONFIG_INFO_COLOR				0
#define NRFX_SPIM_CONFIG_DEBUG_COLOR			0
#define NRFX_SPI_ENABLED						1
#define NRFX_SPI0_ENABLED						0
#define NRFX_SPI1_ENABLED						0
#define NRFX_SPI2_ENABLED						0
#define NRFX_SPI_MISO_PULL_CFG					1
#define NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY	6
#define NRFX_SPI_CONFIG_LOG_ENABLED				0
#define NRFX_SPI_CONFIG_LOG_LEVEL				3
#define NRFX_SPI_CONFIG_INFO_COLOR				0
#define NRFX_SPI_CONFIG_DEBUG_COLOR				0


// <e> NRFX_RNG_ENABLED - nrfx_rng - RNG peripheral driver
//==========================================================
#define NRFX_RNG_ENABLED						1
#define NRFX_RNG_CONFIG_ERROR_CORRECTION		1
#define NRFX_RNG_CONFIG_IRQ_PRIORITY			6
#define NRFX_RNG_CONFIG_LOG_ENABLED				0
#define NRFX_RNG_CONFIG_LOG_LEVEL				3
#define NRFX_RNG_CONFIG_INFO_COLOR				0
#define NRFX_RNG_CONFIG_DEBUG_COLOR				0


// <e> RNG_ENABLED - nrf_drv_rng - RNG peripheral driver - legacy layer
//==========================================================
#define RNG_ENABLED								1
#define RNG_CONFIG_ERROR_CORRECTION				1
#define RNG_CONFIG_POOL_SIZE					64
#define RNG_CONFIG_IRQ_PRIORITY					6

// <e> NRF_QUEUE_ENABLED - nrf_queue - Queue module
//==========================================================
#define NRF_QUEUE_ENABLED						1
#define NRF_QUEUE_CLI_CMDS						0
#define NRF_QUEUE_CONFIG_LOG_ENABLED			0
#define NRF_QUEUE_CONFIG_LOG_LEVEL				3
#define NRF_QUEUE_CONFIG_LOG_INIT_FILTER_LEVEL	3
#define NRF_QUEUE_CONFIG_INFO_COLOR				0
#define NRF_QUEUE_CONFIG_DEBUG_COLOR			0


// <e> NRF_PWR_MGMT_ENABLED - nrf_pwr_mgmt - Power management module
//==========================================================
#define NRF_PWR_MGMT_ENABLED							1
#define NRF_PWR_MGMT_CONFIG_DEBUG_PIN_ENABLED			0
#define NRF_PWR_MGMT_SLEEP_DEBUG_PIN					31
#define NRF_PWR_MGMT_CONFIG_CPU_USAGE_MONITOR_ENABLED	0
#define NRF_PWR_MGMT_CONFIG_STANDBY_TIMEOUT_ENABLED		0
#define NRF_PWR_MGMT_CONFIG_STANDBY_TIMEOUT_S			3
#define NRF_PWR_MGMT_CONFIG_FPU_SUPPORT_ENABLED			1
#define NRF_PWR_MGMT_CONFIG_AUTO_SHUTDOWN_RETRY			1
#define NRF_PWR_MGMT_CONFIG_USE_SCHEDULER				0
#define NRF_PWR_MGMT_CONFIG_HANDLER_PRIORITY_COUNT		3


// <e> UART_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#define UART_CONFIG_LOG_ENABLED							0
#define UART_CONFIG_LOG_LEVEL							3
#define UART_CONFIG_INFO_COLOR							0
#define UART_CONFIG_DEBUG_COLOR							0


// <e> NRFX_UART_ENABLED - nrfx_uart - UART peripheral driver
//==========================================================
#define NRFX_UART_ENABLED								1
#define NRFX_UART0_ENABLED								1
#define NRFX_UART_DEFAULT_CONFIG_HWFC					0
#define NRFX_UART_DEFAULT_CONFIG_PARITY					0
#define NRFX_UART_DEFAULT_CONFIG_BAUDRATE				30924800
#define NRFX_UART_DEFAULT_CONFIG_IRQ_PRIORITY			6
#define NRFX_UART_CONFIG_LOG_ENABLED					0
#define NRFX_UART_CONFIG_LOG_LEVEL						3
#define NRFX_UART_CONFIG_INFO_COLOR						0
#define NRFX_UART_CONFIG_DEBUG_COLOR					0

#define UART_ENABLED									1
#define UART_DEFAULT_CONFIG_HWFC						0
#define UART_DEFAULT_CONFIG_PARITY						0
#define UART_DEFAULT_CONFIG_BAUDRATE					30801920    // <30801920=> 115200 baud 
#define UART_DEFAULT_CONFIG_IRQ_PRIORITY				6
#define UART_EASY_DMA_SUPPORT							0
#define UART_LEGACY_SUPPORT								1


//==========================================================
// <e> GPIOTE_ENABLED - nrf_drv_gpiote - GPIOTE peripheral driver - legacy layer
//==========================================================
#define GPIOTE_ENABLED									1
#define GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS			10
#define GPIOTE_CONFIG_IRQ_PRIORITY						6
#define NRFX_GPIOTE_ENABLED								1
#define NRFX_GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS		1
#define NRFX_GPIOTE_CONFIG_IRQ_PRIORITY					6
#define NRFX_GPIOTE_CONFIG_LOG_ENABLED					0
#define NRFX_GPIOTE_CONFIG_LOG_LEVEL					3
#define NRFX_GPIOTE_CONFIG_INFO_COLOR					0
#define NRFX_GPIOTE_CONFIG_DEBUG_COLOR					0



// <e> FDS_ENABLED - fds - Flash data storage module
//==========================================================
#define FDS_ENABLED										1
#define FDS_VIRTUAL_PAGES								6
#define FDS_VIRTUAL_PAGE_SIZE							1024
#define FDS_VIRTUAL_PAGES_RESERVED						0
#define FDS_BACKEND										2
#define FDS_OP_QUEUE_SIZE								6
#define FDS_CRC_CHECK_ON_READ							1
#define FDS_CRC_CHECK_ON_WRITE							0
#define FDS_MAX_USERS									4


// <e> NRF_BALLOC_ENABLED - nrf_balloc - Block allocator module
//==========================================================
#define NRF_BALLOC_ENABLED								1
#define NRF_BALLOC_CONFIG_DEBUG_ENABLED					0
#define NRF_BALLOC_CONFIG_HEAD_GUARD_WORDS				1
#define NRF_BALLOC_CONFIG_TAIL_GUARD_WORDS				1
#define NRF_BALLOC_CONFIG_BASIC_CHECKS_ENABLED			0
#define NRF_BALLOC_CONFIG_DOUBLE_FREE_CHECK_ENABLED		0
#define NRF_BALLOC_CONFIG_DATA_TRASHING_CHECK_ENABLED	0
#define NRF_BALLOC_CLI_CMDS								0 // <q> NRF_BALLOC_CLI_CMDS  - Enable CLI commands specific to the module

// </h> 
//==========================================================

// <h> nrf_fstorage_sd - Implementation using the SoftDevice

// <i> Configuration options for the fstorage implementation using the SoftDevice
//==========================================================
// <o> NRF_FSTORAGE_SD_QUEUE_SIZE - Size of the internal queue of operations 
// <i> Increase this value if API calls frequently return the error @ref NRF_ERROR_NO_MEM.
#define CRC16_ENABLED									1
#define NRF_FSTORAGE_ENABLED							1
#define NRF_FSTORAGE_PARAM_CHECK_DISABLED				0
#define NRF_FSTORAGE_SD_QUEUE_SIZE						4
#define NRF_FSTORAGE_SD_MAX_RETRIES						8
#define NRF_FSTORAGE_SD_MAX_WRITE_SIZE					4096
#define NRF_MEMOBJ_ENABLED								1



// <e> NRFX_PWM_ENABLED - nrfx_pwm - PWM peripheral driver
//==========================================================
#define NRFX_PWM_ENABLED							1
#define NRFX_PWM0_ENABLED							1
#define NRFX_PWM1_ENABLED							0
#define NRFX_PWM2_ENABLED							0
#define NRFX_PWM3_ENABLED							0

// <o> NRFX_PWM_DEFAULT_CONFIG_OUT0_PIN - Out0 pin  <0-31> 
#define NRFX_PWM_DEFAULT_CONFIG_OUT0_PIN			31
#define NRFX_PWM_DEFAULT_CONFIG_OUT1_PIN			31
#define NRFX_PWM_DEFAULT_CONFIG_OUT2_PIN			31
#define NRFX_PWM_DEFAULT_CONFIG_OUT3_PIN			31

#define NRFX_PWM_DEFAULT_CONFIG_BASE_CLOCK			4
#define NRFX_PWM_DEFAULT_CONFIG_COUNT_MODE			0
#define NRFX_PWM_DEFAULT_CONFIG_TOP_VALUE			1000
#define NRFX_PWM_DEFAULT_CONFIG_LOAD_MODE			0
#define NRFX_PWM_DEFAULT_CONFIG_STEP_MODE			0
#define NRFX_PWM_DEFAULT_CONFIG_IRQ_PRIORITY		6


// <e> NRFX_PWM_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#define NRFX_PWM_CONFIG_LOG_ENABLED					0
#define NRFX_PWM_CONFIG_LOG_LEVEL					3
#define NRFX_PWM_CONFIG_INFO_COLOR					0
#define NRFX_PWM_CONFIG_DEBUG_COLOR					0

// <e> NRFX_TWIM_ENABLED - nrfx_twim - TWIM peripheral driver
//==========================================================
#define NRFX_TWIM_ENABLED							1
#define NRFX_TWIM0_ENABLED							0
#define NRFX_TWIM1_ENABLED							0
#define NRFX_TWIM_DEFAULT_CONFIG_FREQUENCY			26738688  // <26738688=> 100k, <67108864=> 250k, <104857600=> 400k 
#define NRFX_TWIM_DEFAULT_CONFIG_HOLD_BUS_UNINIT	0
#define NRFX_TWIM_DEFAULT_CONFIG_IRQ_PRIORITY		6


// <e> NRFX_TWIM_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#define NRFX_TWIM_CONFIG_LOG_ENABLED				0
#define NRFX_TWIM_CONFIG_LOG_LEVEL					3
#define NRFX_TWIM_CONFIG_INFO_COLOR					0
#define NRFX_TWIM_CONFIG_DEBUG_COLOR				0


// <e> NRFX_TWI_ENABLED - nrfx_twi - TWI peripheral driver
//==========================================================
#define NRFX_TWI_ENABLED							1
#define NRFX_TWI0_ENABLED							0
#define NRFX_TWI1_ENABLED							0
#define NRFX_TWI_DEFAULT_CONFIG_FREQUENCY			26738688  // <26738688=> 100k, <67108864=> 250k, <104857600=> 400k 
#define NRFX_TWI_DEFAULT_CONFIG_HOLD_BUS_UNINIT		0
#define NRFX_TWI_DEFAULT_CONFIG_IRQ_PRIORITY		6


// <e> NRFX_TWI_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#define NRFX_TWI_CONFIG_LOG_ENABLED					0
#define NRFX_TWI_CONFIG_LOG_LEVEL					3
#define NRFX_TWI_CONFIG_INFO_COLOR					0
#define NRFX_TWI_CONFIG_DEBUG_COLOR					0


// <e> TWI_ENABLED - nrf_drv_twi - TWI/TWIM peripheral driver - legacy layer
//==========================================================
#define TWI_ENABLED									1
#define TWI_DEFAULT_CONFIG_FREQUENCY				26738688 // <26738688=> 100k, <67108864=> 250k, <104857600=> 400k 
#define TWI_DEFAULT_CONFIG_CLR_BUS_INIT				0
#define TWI_DEFAULT_CONFIG_HOLD_BUS_UNINIT			0
#define TWI_DEFAULT_CONFIG_IRQ_PRIORITY				6
#define TWI0_ENABLED								1
#define TWI0_USE_EASY_DMA							1
#define TWI1_ENABLED								1
#define TWI1_USE_EASY_DMA							1



// <h> nRF_DFU 

//==========================================================
// <h> ble_dfu - Device Firmware Update

//==========================================================
// <q> BLE_DFU_ENABLED  - Enable DFU Service.
 

#ifndef BLE_DFU_ENABLED
#define BLE_DFU_ENABLED 1
#endif

// <q> NRF_DFU_BLE_BUTTONLESS_SUPPORTS_BONDS  - Buttonless DFU supports bonds.
 

#ifndef NRF_DFU_BLE_BUTTONLESS_SUPPORTS_BONDS
#define NRF_DFU_BLE_BUTTONLESS_SUPPORTS_BONDS 0
#endif

// <e> APP_SDCARD_ENABLED - app_sdcard - SD/MMC card support using SPI
//==========================================================
#ifndef APP_SDCARD_ENABLED
#define APP_SDCARD_ENABLED 0
#endif
// <o> APP_SDCARD_SPI_INSTANCE  - SPI instance used
 
// <0=> 0 
// <1=> 1 
// <2=> 2 

#ifndef APP_SDCARD_SPI_INSTANCE
#define APP_SDCARD_SPI_INSTANCE 0
#endif

// <o> APP_SDCARD_FREQ_INIT  - SPI frequency
 
// <33554432=> 125 kHz 
// <67108864=> 250 kHz 
// <134217728=> 500 kHz 
// <268435456=> 1 MHz 
// <536870912=> 2 MHz 
// <1073741824=> 4 MHz 
// <2147483648=> 8 MHz 

#ifndef APP_SDCARD_FREQ_INIT
#define APP_SDCARD_FREQ_INIT 67108864
#endif

// <o> APP_SDCARD_FREQ_DATA  - SPI frequency
 
// <33554432=> 125 kHz 
// <67108864=> 250 kHz 
// <134217728=> 500 kHz 
// <268435456=> 1 MHz 
// <536870912=> 2 MHz 
// <1073741824=> 4 MHz 
// <2147483648=> 8 MHz 

#ifndef APP_SDCARD_FREQ_DATA
#define APP_SDCARD_FREQ_DATA 1073741824
#endif

// </e>


//==========================================================
// <e> I2S_ENABLED - nrf_drv_i2s - I2S peripheral driver - legacy layer
//==========================================================
#ifndef I2S_ENABLED
#define I2S_ENABLED 1
#endif
// <o> I2S_CONFIG_SCK_PIN - SCK pin  <0-31> 


#ifndef I2S_CONFIG_SCK_PIN
#define I2S_CONFIG_SCK_PIN 31
#endif

// <o> I2S_CONFIG_LRCK_PIN - LRCK pin  <1-31> 


#ifndef I2S_CONFIG_LRCK_PIN
#define I2S_CONFIG_LRCK_PIN 30
#endif

// <o> I2S_CONFIG_MCK_PIN - MCK pin 
#ifndef I2S_CONFIG_MCK_PIN
#define I2S_CONFIG_MCK_PIN 255
#endif

// <o> I2S_CONFIG_SDOUT_PIN - SDOUT pin  <0-31> 


#ifndef I2S_CONFIG_SDOUT_PIN
#define I2S_CONFIG_SDOUT_PIN 29
#endif

// <o> I2S_CONFIG_SDIN_PIN - SDIN pin  <0-31> 


#ifndef I2S_CONFIG_SDIN_PIN
#define I2S_CONFIG_SDIN_PIN 28
#endif

// <o> I2S_CONFIG_MASTER  - Mode
 
// <0=> Master 
// <1=> Slave 

#ifndef I2S_CONFIG_MASTER
#define I2S_CONFIG_MASTER 0
#endif

// <o> I2S_CONFIG_FORMAT  - Format
 
// <0=> I2S 
// <1=> Aligned 

#ifndef I2S_CONFIG_FORMAT
#define I2S_CONFIG_FORMAT 0
#endif

// <o> I2S_CONFIG_ALIGN  - Alignment
 
// <0=> Left 
// <1=> Right 

#ifndef I2S_CONFIG_ALIGN
#define I2S_CONFIG_ALIGN 0
#endif

// <o> I2S_CONFIG_SWIDTH  - Sample width (bits)
 
// <0=> 8 
// <1=> 16 
// <2=> 24 

#ifndef I2S_CONFIG_SWIDTH
#define I2S_CONFIG_SWIDTH 1
#endif

// <o> I2S_CONFIG_CHANNELS  - Channels
 
// <0=> Stereo 
// <1=> Left 
// <2=> Right 

#ifndef I2S_CONFIG_CHANNELS
#define I2S_CONFIG_CHANNELS 1
#endif

// <o> I2S_CONFIG_MCK_SETUP  - MCK behavior
 
// <0=> Disabled 
// <2147483648=> 32MHz/2 
// <1342177280=> 32MHz/3 
// <1073741824=> 32MHz/4 
// <805306368=> 32MHz/5 
// <671088640=> 32MHz/6 
// <536870912=> 32MHz/8 
// <402653184=> 32MHz/10 
// <369098752=> 32MHz/11 
// <285212672=> 32MHz/15 
// <268435456=> 32MHz/16 
// <201326592=> 32MHz/21 
// <184549376=> 32MHz/23 
// <142606336=> 32MHz/30 
// <138412032=> 32MHz/31 
// <134217728=> 32MHz/32 
// <100663296=> 32MHz/42 
// <68157440=> 32MHz/63 
// <34340864=> 32MHz/125 

#ifndef I2S_CONFIG_MCK_SETUP
#define I2S_CONFIG_MCK_SETUP 536870912
#endif

// <o> I2S_CONFIG_RATIO  - MCK/LRCK ratio
 
// <0=> 32x 
// <1=> 48x 
// <2=> 64x 
// <3=> 96x 
// <4=> 128x 
// <5=> 192x 
// <6=> 256x 
// <7=> 384x 
// <8=> 512x 

#ifndef I2S_CONFIG_RATIO
#define I2S_CONFIG_RATIO 2000
#endif

// <o> I2S_CONFIG_IRQ_PRIORITY  - Interrupt priority
 

// <i> Priorities 0,2 (nRF51) and 0,1,4,5 (nRF52) are reserved for SoftDevice
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 

#ifndef I2S_CONFIG_IRQ_PRIORITY
#define I2S_CONFIG_IRQ_PRIORITY 6
#endif

// <e> I2S_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef I2S_CONFIG_LOG_ENABLED
#define I2S_CONFIG_LOG_ENABLED 0
#endif
// <o> I2S_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef I2S_CONFIG_LOG_LEVEL
#define I2S_CONFIG_LOG_LEVEL 3
#endif

// <o> I2S_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef I2S_CONFIG_INFO_COLOR
#define I2S_CONFIG_INFO_COLOR 0
#endif

// <o> I2S_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef I2S_CONFIG_DEBUG_COLOR
#define I2S_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// </e>

// <e> NRFX_I2S_ENABLED - nrfx_i2s - I2S peripheral driver
//==========================================================
#ifndef NRFX_I2S_ENABLED
#define NRFX_I2S_ENABLED 1
#endif
// <o> NRFX_I2S_CONFIG_SCK_PIN - SCK pin  <0-31> 


#ifndef NRFX_I2S_CONFIG_SCK_PIN
#define NRFX_I2S_CONFIG_SCK_PIN 31
#endif

// <o> NRFX_I2S_CONFIG_LRCK_PIN - LRCK pin  <1-31> 


#ifndef NRFX_I2S_CONFIG_LRCK_PIN
#define NRFX_I2S_CONFIG_LRCK_PIN 30
#endif

// <o> NRFX_I2S_CONFIG_MCK_PIN - MCK pin 
#ifndef NRFX_I2S_CONFIG_MCK_PIN
#define NRFX_I2S_CONFIG_MCK_PIN 255
#endif

// <o> NRFX_I2S_CONFIG_SDOUT_PIN - SDOUT pin  <0-31> 


#ifndef NRFX_I2S_CONFIG_SDOUT_PIN
#define NRFX_I2S_CONFIG_SDOUT_PIN 29
#endif

// <o> NRFX_I2S_CONFIG_SDIN_PIN - SDIN pin  <0-31> 


#ifndef NRFX_I2S_CONFIG_SDIN_PIN
#define NRFX_I2S_CONFIG_SDIN_PIN 28
#endif

// <o> NRFX_I2S_CONFIG_MASTER  - Mode
 
// <0=> Master 
// <1=> Slave 

#ifndef NRFX_I2S_CONFIG_MASTER
#define NRFX_I2S_CONFIG_MASTER 0
#endif

// <o> NRFX_I2S_CONFIG_FORMAT  - Format
 
// <0=> I2S 
// <1=> Aligned 

#ifndef NRFX_I2S_CONFIG_FORMAT
#define NRFX_I2S_CONFIG_FORMAT 0
#endif

// <o> NRFX_I2S_CONFIG_ALIGN  - Alignment
 
// <0=> Left 
// <1=> Right 

#ifndef NRFX_I2S_CONFIG_ALIGN
#define NRFX_I2S_CONFIG_ALIGN 0
#endif

// <o> NRFX_I2S_CONFIG_SWIDTH  - Sample width (bits)
 
// <0=> 8 
// <1=> 16 
// <2=> 24 

#ifndef NRFX_I2S_CONFIG_SWIDTH
#define NRFX_I2S_CONFIG_SWIDTH 1
#endif

// <o> NRFX_I2S_CONFIG_CHANNELS  - Channels
 
// <0=> Stereo 
// <1=> Left 
// <2=> Right 

#ifndef NRFX_I2S_CONFIG_CHANNELS
#define NRFX_I2S_CONFIG_CHANNELS 1
#endif

// <o> NRFX_I2S_CONFIG_MCK_SETUP  - MCK behavior
 
// <0=> Disabled 
// <2147483648=> 32MHz/2 
// <1342177280=> 32MHz/3 
// <1073741824=> 32MHz/4 
// <805306368=> 32MHz/5 
// <671088640=> 32MHz/6 
// <536870912=> 32MHz/8 
// <402653184=> 32MHz/10 
// <369098752=> 32MHz/11 
// <285212672=> 32MHz/15 
// <268435456=> 32MHz/16 
// <201326592=> 32MHz/21 
// <184549376=> 32MHz/23 
// <142606336=> 32MHz/30 
// <138412032=> 32MHz/31 
// <134217728=> 32MHz/32 
// <100663296=> 32MHz/42 
// <68157440=> 32MHz/63 
// <34340864=> 32MHz/125 

#ifndef NRFX_I2S_CONFIG_MCK_SETUP
#define NRFX_I2S_CONFIG_MCK_SETUP 536870912
#endif

// <o> NRFX_I2S_CONFIG_RATIO  - MCK/LRCK ratio
 
// <0=> 32x 
// <1=> 48x 
// <2=> 64x 
// <3=> 96x 
// <4=> 128x 
// <5=> 192x 
// <6=> 256x 
// <7=> 384x 
// <8=> 512x 

#ifndef NRFX_I2S_CONFIG_RATIO
#define NRFX_I2S_CONFIG_RATIO 2000
#endif

// <o> NRFX_I2S_CONFIG_IRQ_PRIORITY  - Interrupt priority
 
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 

#ifndef NRFX_I2S_CONFIG_IRQ_PRIORITY
#define NRFX_I2S_CONFIG_IRQ_PRIORITY 6
#endif

// <e> NRFX_I2S_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRFX_I2S_CONFIG_LOG_ENABLED
#define NRFX_I2S_CONFIG_LOG_ENABLED 0
#endif
// <o> NRFX_I2S_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRFX_I2S_CONFIG_LOG_LEVEL
#define NRFX_I2S_CONFIG_LOG_LEVEL 3
#endif

// <o> NRFX_I2S_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRFX_I2S_CONFIG_INFO_COLOR
#define NRFX_I2S_CONFIG_INFO_COLOR 0
#endif

// <o> NRFX_I2S_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRFX_I2S_CONFIG_DEBUG_COLOR
#define NRFX_I2S_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// </e>

// <e> NRFX_POWER_ENABLED - nrfx_power - POWER peripheral driver
//==========================================================
#ifndef NRFX_POWER_ENABLED
#define NRFX_POWER_ENABLED 1
#endif
// <o> NRFX_POWER_CONFIG_IRQ_PRIORITY  - Interrupt priority
 
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 

#ifndef NRFX_POWER_CONFIG_IRQ_PRIORITY
#define NRFX_POWER_CONFIG_IRQ_PRIORITY 6
#endif

// <q> NRFX_POWER_CONFIG_DEFAULT_DCDCEN  - The default configuration of main DCDC regulator
 

// <i> This settings means only that components for DCDC regulator are installed and it can be enabled.

#ifndef NRFX_POWER_CONFIG_DEFAULT_DCDCEN
#define NRFX_POWER_CONFIG_DEFAULT_DCDCEN 0
#endif

// <q> NRFX_POWER_CONFIG_DEFAULT_DCDCENHV  - The default configuration of High Voltage DCDC regulator
 

// <i> This settings means only that components for DCDC regulator are installed and it can be enabled.

#ifndef NRFX_POWER_CONFIG_DEFAULT_DCDCENHV
#define NRFX_POWER_CONFIG_DEFAULT_DCDCENHV 0
#endif

// </e>
// <e> NRFX_PRS_ENABLED - nrfx_prs - Peripheral Resource Sharing module
//==========================================================
#ifndef NRFX_PRS_ENABLED
#define NRFX_PRS_ENABLED 1
#endif
// <q> NRFX_PRS_BOX_0_ENABLED  - Enables box 0 in the module.
 

#ifndef NRFX_PRS_BOX_0_ENABLED
#define NRFX_PRS_BOX_0_ENABLED 0
#endif

// <q> NRFX_PRS_BOX_1_ENABLED  - Enables box 1 in the module.
 

#ifndef NRFX_PRS_BOX_1_ENABLED
#define NRFX_PRS_BOX_1_ENABLED 0
#endif

// <q> NRFX_PRS_BOX_2_ENABLED  - Enables box 2 in the module.
 

#ifndef NRFX_PRS_BOX_2_ENABLED
#define NRFX_PRS_BOX_2_ENABLED 0
#endif

// <q> NRFX_PRS_BOX_3_ENABLED  - Enables box 3 in the module.
 

#ifndef NRFX_PRS_BOX_3_ENABLED
#define NRFX_PRS_BOX_3_ENABLED 0
#endif

// <q> NRFX_PRS_BOX_4_ENABLED  - Enables box 4 in the module.
 

#ifndef NRFX_PRS_BOX_4_ENABLED
#define NRFX_PRS_BOX_4_ENABLED 1
#endif

// <e> NRFX_PRS_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRFX_PRS_CONFIG_LOG_ENABLED
#define NRFX_PRS_CONFIG_LOG_ENABLED 0
#endif
// <o> NRFX_PRS_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRFX_PRS_CONFIG_LOG_LEVEL
#define NRFX_PRS_CONFIG_LOG_LEVEL 3
#endif

// <o> NRFX_PRS_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRFX_PRS_CONFIG_INFO_COLOR
#define NRFX_PRS_CONFIG_INFO_COLOR 0
#endif

// <o> NRFX_PRS_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRFX_PRS_CONFIG_DEBUG_COLOR
#define NRFX_PRS_CONFIG_DEBUG_COLOR 0
#endif


// <e> POWER_ENABLED - nrf_drv_power - POWER peripheral driver - legacy layer
//==========================================================
#ifndef POWER_ENABLED
#define POWER_ENABLED 1
#endif
// <o> POWER_CONFIG_IRQ_PRIORITY  - Interrupt priority
 

// <i> Priorities 0,2 (nRF51) and 0,1,4,5 (nRF52) are reserved for SoftDevice
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 

#ifndef POWER_CONFIG_IRQ_PRIORITY
#define POWER_CONFIG_IRQ_PRIORITY 6
#endif

// <q> POWER_CONFIG_DEFAULT_DCDCEN  - The default configuration of main DCDC regulator
 

// <i> This settings means only that components for DCDC regulator are installed and it can be enabled.

#ifndef POWER_CONFIG_DEFAULT_DCDCEN
#define POWER_CONFIG_DEFAULT_DCDCEN 0
#endif

// <q> POWER_CONFIG_DEFAULT_DCDCENHV  - The default configuration of High Voltage DCDC regulator
 

// <i> This settings means only that components for DCDC regulator are installed and it can be enabled.

#ifndef POWER_CONFIG_DEFAULT_DCDCENHV
#define POWER_CONFIG_DEFAULT_DCDCENHV 0
#endif

// </e>

// <e> QSPI_ENABLED - nrf_drv_qspi - QSPI peripheral driver - legacy layer
//==========================================================
#ifndef QSPI_ENABLED
#define QSPI_ENABLED 1
#endif
// <o> QSPI_CONFIG_SCK_DELAY - tSHSL, tWHSL and tSHWL in number of 16 MHz periods (62.5 ns).  <0-255> 


#ifndef QSPI_CONFIG_SCK_DELAY
#define QSPI_CONFIG_SCK_DELAY 1
#endif

// <o> QSPI_CONFIG_XIP_OFFSET - Address offset in the external memory for Execute in Place operation. 
#ifndef QSPI_CONFIG_XIP_OFFSET
#define QSPI_CONFIG_XIP_OFFSET 0
#endif

// <o> QSPI_CONFIG_READOC  - Number of data lines and opcode used for reading.
 
// <0=> FastRead 
// <1=> Read2O 
// <2=> Read2IO 
// <3=> Read4O 
// <4=> Read4IO 

#ifndef QSPI_CONFIG_READOC
#define QSPI_CONFIG_READOC 0
#endif

// <o> QSPI_CONFIG_WRITEOC  - Number of data lines and opcode used for writing.
 
// <0=> PP 
// <1=> PP2O 
// <2=> PP4O 
// <3=> PP4IO 

#ifndef QSPI_CONFIG_WRITEOC
#define QSPI_CONFIG_WRITEOC 0
#endif

// <o> QSPI_CONFIG_ADDRMODE  - Addressing mode.
 
// <0=> 24bit 
// <1=> 32bit 

#ifndef QSPI_CONFIG_ADDRMODE
#define QSPI_CONFIG_ADDRMODE 0
#endif

// <o> QSPI_CONFIG_MODE  - SPI mode.
 
// <0=> Mode 0 
// <1=> Mode 1 

#ifndef QSPI_CONFIG_MODE
#define QSPI_CONFIG_MODE 0
#endif

// <o> QSPI_CONFIG_FREQUENCY  - Frequency divider.
 
// <0=> 32MHz/1 
// <1=> 32MHz/2 
// <2=> 32MHz/3 
// <3=> 32MHz/4 
// <4=> 32MHz/5 
// <5=> 32MHz/6 
// <6=> 32MHz/7 
// <7=> 32MHz/8 
// <8=> 32MHz/9 
// <9=> 32MHz/10 
// <10=> 32MHz/11 
// <11=> 32MHz/12 
// <12=> 32MHz/13 
// <13=> 32MHz/14 
// <14=> 32MHz/15 
// <15=> 32MHz/16 

#ifndef QSPI_CONFIG_FREQUENCY
#define QSPI_CONFIG_FREQUENCY 15
#endif

// <s> QSPI_PIN_SCK - SCK pin value.
#ifndef QSPI_PIN_SCK
#define QSPI_PIN_SCK NRF_QSPI_PIN_NOT_CONNECTED
#endif

// <s> QSPI_PIN_CSN - CSN pin value.
#ifndef QSPI_PIN_CSN
#define QSPI_PIN_CSN NRF_QSPI_PIN_NOT_CONNECTED
#endif

// <s> QSPI_PIN_IO0 - IO0 pin value.
#ifndef QSPI_PIN_IO0
#define QSPI_PIN_IO0 NRF_QSPI_PIN_NOT_CONNECTED
#endif

// <s> QSPI_PIN_IO1 - IO1 pin value.
#ifndef QSPI_PIN_IO1
#define QSPI_PIN_IO1 NRF_QSPI_PIN_NOT_CONNECTED
#endif

// <s> QSPI_PIN_IO2 - IO2 pin value.
#ifndef QSPI_PIN_IO2
#define QSPI_PIN_IO2 NRF_QSPI_PIN_NOT_CONNECTED
#endif

// <s> QSPI_PIN_IO3 - IO3 pin value.
#ifndef QSPI_PIN_IO3
#define QSPI_PIN_IO3 NRF_QSPI_PIN_NOT_CONNECTED
#endif

// <o> QSPI_CONFIG_IRQ_PRIORITY  - Interrupt priority
 

// <i> Priorities 0,2 (nRF51) and 0,1,4,5 (nRF52) are reserved for SoftDevice
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 

#ifndef QSPI_CONFIG_IRQ_PRIORITY
#define QSPI_CONFIG_IRQ_PRIORITY 6
#endif

// </e>

// </e>

// <e> NRFX_QSPI_ENABLED - nrfx_qspi - QSPI peripheral driver
//==========================================================
#ifndef NRFX_QSPI_ENABLED
#define NRFX_QSPI_ENABLED 1
#endif
// <o> NRFX_QSPI_CONFIG_SCK_DELAY - tSHSL, tWHSL and tSHWL in number of 16 MHz periods (62.5 ns).  <0-255> 


#ifndef NRFX_QSPI_CONFIG_SCK_DELAY
#define NRFX_QSPI_CONFIG_SCK_DELAY 1
#endif

// <o> NRFX_QSPI_CONFIG_XIP_OFFSET - Address offset in the external memory for Execute in Place operation. 
#ifndef NRFX_QSPI_CONFIG_XIP_OFFSET
#define NRFX_QSPI_CONFIG_XIP_OFFSET 0
#endif

// <o> NRFX_QSPI_CONFIG_READOC  - Number of data lines and opcode used for reading.
 
// <0=> FastRead 
// <1=> Read2O 
// <2=> Read2IO 
// <3=> Read4O 
// <4=> Read4IO 

#ifndef NRFX_QSPI_CONFIG_READOC
#define NRFX_QSPI_CONFIG_READOC 0
#endif

// <o> NRFX_QSPI_CONFIG_WRITEOC  - Number of data lines and opcode used for writing.
 
// <0=> PP 
// <1=> PP2O 
// <2=> PP4O 
// <3=> PP4IO 

#ifndef NRFX_QSPI_CONFIG_WRITEOC
#define NRFX_QSPI_CONFIG_WRITEOC 0
#endif

// <o> NRFX_QSPI_CONFIG_ADDRMODE  - Addressing mode.
 
// <0=> 24bit 
// <1=> 32bit 

#ifndef NRFX_QSPI_CONFIG_ADDRMODE
#define NRFX_QSPI_CONFIG_ADDRMODE 0
#endif

// <o> NRFX_QSPI_CONFIG_MODE  - SPI mode.
 
// <0=> Mode 0 
// <1=> Mode 1 

#ifndef NRFX_QSPI_CONFIG_MODE
#define NRFX_QSPI_CONFIG_MODE 0
#endif

// <o> NRFX_QSPI_CONFIG_FREQUENCY  - Frequency divider.
 
// <0=> 32MHz/1 
// <1=> 32MHz/2 
// <2=> 32MHz/3 
// <3=> 32MHz/4 
// <4=> 32MHz/5 
// <5=> 32MHz/6 
// <6=> 32MHz/7 
// <7=> 32MHz/8 
// <8=> 32MHz/9 
// <9=> 32MHz/10 
// <10=> 32MHz/11 
// <11=> 32MHz/12 
// <12=> 32MHz/13 
// <13=> 32MHz/14 
// <14=> 32MHz/15 
// <15=> 32MHz/16 

#ifndef NRFX_QSPI_CONFIG_FREQUENCY
#define NRFX_QSPI_CONFIG_FREQUENCY 15
#endif

// <s> NRFX_QSPI_PIN_SCK - SCK pin value.
#ifndef NRFX_QSPI_PIN_SCK
#define NRFX_QSPI_PIN_SCK NRF_QSPI_PIN_NOT_CONNECTED
#endif

// <s> NRFX_QSPI_PIN_CSN - CSN pin value.
#ifndef NRFX_QSPI_PIN_CSN
#define NRFX_QSPI_PIN_CSN NRF_QSPI_PIN_NOT_CONNECTED
#endif

// <s> NRFX_QSPI_PIN_IO0 - IO0 pin value.
#ifndef NRFX_QSPI_PIN_IO0
#define NRFX_QSPI_PIN_IO0 NRF_QSPI_PIN_NOT_CONNECTED
#endif

// <s> NRFX_QSPI_PIN_IO1 - IO1 pin value.
#ifndef NRFX_QSPI_PIN_IO1
#define NRFX_QSPI_PIN_IO1 NRF_QSPI_PIN_NOT_CONNECTED
#endif

// <s> NRFX_QSPI_PIN_IO2 - IO2 pin value.
#ifndef NRFX_QSPI_PIN_IO2
#define NRFX_QSPI_PIN_IO2 NRF_QSPI_PIN_NOT_CONNECTED
#endif

// <s> NRFX_QSPI_PIN_IO3 - IO3 pin value.
#ifndef NRFX_QSPI_PIN_IO3
#define NRFX_QSPI_PIN_IO3 NRF_QSPI_PIN_NOT_CONNECTED
#endif

// <o> NRFX_QSPI_CONFIG_IRQ_PRIORITY  - Interrupt priority
 
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 

#ifndef NRFX_QSPI_CONFIG_IRQ_PRIORITY
#define NRFX_QSPI_CONFIG_IRQ_PRIORITY 6
#endif

// <h> nRF_BLE 

//==========================================================
// <q> BLE_ADVERTISING_ENABLED  - ble_advertising - Advertising module
 

#ifndef BLE_ADVERTISING_ENABLED
#define BLE_ADVERTISING_ENABLED 1
#endif

// <q> BLE_DTM_ENABLED  - ble_dtm - Module for testing RF/PHY using DTM commands
 

#ifndef BLE_DTM_ENABLED
#define BLE_DTM_ENABLED 0
#endif

// <q> BLE_RACP_ENABLED  - ble_racp - Record Access Control Point library
 

#ifndef BLE_RACP_ENABLED
#define BLE_RACP_ENABLED 0
#endif

// <e> NRF_BLE_CONN_PARAMS_ENABLED - ble_conn_params - Initiating and executing a connection parameters negotiation procedure
//==========================================================
#ifndef NRF_BLE_CONN_PARAMS_ENABLED
#define NRF_BLE_CONN_PARAMS_ENABLED 1
#endif
// <o> NRF_BLE_CONN_PARAMS_MAX_SLAVE_LATENCY_DEVIATION - The largest acceptable deviation in slave latency. 
// <i> The largest deviation (+ or -) from the requested slave latency that will not be renegotiated.

#ifndef NRF_BLE_CONN_PARAMS_MAX_SLAVE_LATENCY_DEVIATION
#define NRF_BLE_CONN_PARAMS_MAX_SLAVE_LATENCY_DEVIATION 499
#endif

// <o> NRF_BLE_CONN_PARAMS_MAX_SUPERVISION_TIMEOUT_DEVIATION - The largest acceptable deviation (in 10 ms units) in supervision timeout. 
// <i> The largest deviation (+ or -, in 10 ms units) from the requested supervision timeout that will not be renegotiated.

#ifndef NRF_BLE_CONN_PARAMS_MAX_SUPERVISION_TIMEOUT_DEVIATION
#define NRF_BLE_CONN_PARAMS_MAX_SUPERVISION_TIMEOUT_DEVIATION 65535
#endif

// </e>

// <q> NRF_BLE_GATT_ENABLED  - nrf_ble_gatt - GATT module
 

#ifndef NRF_BLE_GATT_ENABLED
#define NRF_BLE_GATT_ENABLED 1
#endif

// <q> NRF_BLE_LESC_ENABLED  - nrf_ble_lesc - Le Secure Connection
 
#ifndef NRF_BLE_LESC_ENABLED
#define NRF_BLE_LESC_ENABLED 1
#endif

// <e> NRF_BLE_QWR_ENABLED - nrf_ble_qwr - Queued writes support module (prepare/execute write)
//==========================================================
#ifndef NRF_BLE_QWR_ENABLED
#define NRF_BLE_QWR_ENABLED 1
#endif
// <o> NRF_BLE_QWR_MAX_ATTR - Maximum number of attribute handles that can be registered. This number must be adjusted according to the number of attributes for which Queued Writes will be enabled. If it is zero, the module will reject all Queued Write requests. 
#ifndef NRF_BLE_QWR_MAX_ATTR
#define NRF_BLE_QWR_MAX_ATTR 0
#endif

// </e>

// <e> PEER_MANAGER_ENABLED - peer_manager - Peer Manager
//==========================================================
#ifndef PEER_MANAGER_ENABLED
#define PEER_MANAGER_ENABLED 1
#endif
// <o> PM_MAX_REGISTRANTS - Number of event handlers that can be registered. 
#ifndef PM_MAX_REGISTRANTS
#define PM_MAX_REGISTRANTS 3
#endif
																   
// <o> PM_FLASH_BUFFERS - Number of internal buffers for flash operations. 
// <i> Decrease this value to lower RAM usage.

#ifndef PM_FLASH_BUFFERS
#define PM_FLASH_BUFFERS 4
#endif

// <q> PM_CENTRAL_ENABLED  - Enable/disable central-specific Peer Manager functionality.
 

// <i> Enable/disable central-specific Peer Manager functionality.

#ifndef PM_CENTRAL_ENABLED
#define PM_CENTRAL_ENABLED 0
#endif

// <q> PM_SERVICE_CHANGED_ENABLED  - Enable/disable the service changed management for GATT server in Peer Manager.
 

// <i> If not using a GATT server, or using a server wihout a service changed characteristic,
// <i> disable this to save code space.

#ifndef PM_SERVICE_CHANGED_ENABLED
#define PM_SERVICE_CHANGED_ENABLED 1
#endif

// <q> PM_PEER_RANKS_ENABLED  - Enable/disable the peer rank management in Peer Manager.
 

// <i> Set this to false to save code space if not using the peer rank API.

#ifndef PM_PEER_RANKS_ENABLED
#define PM_PEER_RANKS_ENABLED 1
#endif

// <q> PM_LESC_ENABLED  - Enable/disable LESC support in Peer Manager.
 

// <i> If set to true, you need to call nrf_ble_lesc_request_handler() in the main loop to respond to LESC-related BLE events. If LESC support is not required, set this to false to save code space.

#ifndef PM_LESC_ENABLED
#define PM_LESC_ENABLED 1
#endif

// <e> PM_RA_PROTECTION_ENABLED - Enable/disable protection against repeated pairing attempts in Peer Manager.
//==========================================================
#ifndef PM_RA_PROTECTION_ENABLED
#define PM_RA_PROTECTION_ENABLED 0
#endif
// <o> PM_RA_PROTECTION_TRACKED_PEERS_NUM - Maximum number of peers whose authorization status can be tracked. 
#ifndef PM_RA_PROTECTION_TRACKED_PEERS_NUM
#define PM_RA_PROTECTION_TRACKED_PEERS_NUM 8
#endif
										  

// <o> PM_RA_PROTECTION_MIN_WAIT_INTERVAL - Minimum waiting interval (in ms) before a new pairing attempt can be initiated. 
#ifndef PM_RA_PROTECTION_MIN_WAIT_INTERVAL
#define PM_RA_PROTECTION_MIN_WAIT_INTERVAL 4000
#endif

// <o> PM_RA_PROTECTION_MAX_WAIT_INTERVAL - Maximum waiting interval (in ms) before a new pairing attempt can be initiated. 
#ifndef PM_RA_PROTECTION_MAX_WAIT_INTERVAL
#define PM_RA_PROTECTION_MAX_WAIT_INTERVAL 64000
#endif

// <o> PM_RA_PROTECTION_REWARD_PERIOD - Reward period (in ms). 
// <i> The waiting interval is gradually decreased when no new failed pairing attempts are made during reward period.

#ifndef PM_RA_PROTECTION_REWARD_PERIOD
#define PM_RA_PROTECTION_REWARD_PERIOD 10000
#endif

// </e>

// <o> PM_HANDLER_SEC_DELAY_MS - Delay before starting security. 
// <i>  This might be necessary for interoperability reasons, especially as peripheral.

#ifndef PM_HANDLER_SEC_DELAY_MS
#define PM_HANDLER_SEC_DELAY_MS 0
#endif

// </e>

// </h> 
//==========================================================

// <h> nRF_BLE_Services 

//==========================================================
// <q> BLE_ANCS_C_ENABLED  - ble_ancs_c - Apple Notification Service Client
 

#ifndef BLE_ANCS_C_ENABLED
#define BLE_ANCS_C_ENABLED 0
#endif

// <q> BLE_ANS_C_ENABLED  - ble_ans_c - Alert Notification Service Client
 

#ifndef BLE_ANS_C_ENABLED
#define BLE_ANS_C_ENABLED 0
#endif

// <q> BLE_BAS_C_ENABLED  - ble_bas_c - Battery Service Client
 

#ifndef BLE_BAS_C_ENABLED
#define BLE_BAS_C_ENABLED 0
#endif

// <e> BLE_BAS_ENABLED - ble_bas - Battery Service
//==========================================================
#ifndef BLE_BAS_ENABLED						
#define BLE_BAS_ENABLED 1
#endif
// <e> BLE_BAS_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
								
										   
										
#ifndef BLE_BAS_CONFIG_LOG_ENABLED
#define BLE_BAS_CONFIG_LOG_ENABLED 0
#endif
										  
										
// <o> BLE_BAS_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef BLE_BAS_CONFIG_LOG_LEVEL
#define BLE_BAS_CONFIG_LOG_LEVEL 3
#endif

// <o> BLE_BAS_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef BLE_BAS_CONFIG_INFO_COLOR
#define BLE_BAS_CONFIG_INFO_COLOR 0
#endif

// <o> BLE_BAS_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef BLE_BAS_CONFIG_DEBUG_COLOR
#define BLE_BAS_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// </e>
   
	  

// <q> BLE_DIS_ENABLED  - ble_dis - Device Information Service
 

#ifndef BLE_DIS_ENABLED
#define BLE_DIS_ENABLED 1  				   
#endif
   

// </e>

// </h> 
//==========================================================

// <h> nRF_Core 

//==========================================================
// <e> NRF_MPU_LIB_ENABLED - nrf_mpu_lib - Module for MPU
//==========================================================
#ifndef NRF_MPU_LIB_ENABLED
#define NRF_MPU_LIB_ENABLED 0
#endif
// <q> NRF_MPU_LIB_CLI_CMDS  - Enable CLI commands specific to the module.
 

#ifndef NRF_MPU_LIB_CLI_CMDS
#define NRF_MPU_LIB_CLI_CMDS 0
#endif

// </e>

// <e> NRF_STACK_GUARD_ENABLED - nrf_stack_guard - Stack guard
//==========================================================
#ifndef NRF_STACK_GUARD_ENABLED
#define NRF_STACK_GUARD_ENABLED 0
#endif
// <o> NRF_STACK_GUARD_CONFIG_SIZE  - Size of the stack guard.
 
// <5=> 32 bytes 
// <6=> 64 bytes 
// <7=> 128 bytes 
// <8=> 256 bytes 
// <9=> 512 bytes 
// <10=> 1024 bytes 
// <11=> 2048 bytes 
// <12=> 4096 bytes 

#ifndef NRF_STACK_GUARD_CONFIG_SIZE
#define NRF_STACK_GUARD_CONFIG_SIZE 7
#endif

// </e>

// </h> 
//==========================================================

// <h> nRF_Crypto 

//==========================================================
// <e> NRF_CRYPTO_ENABLED - nrf_crypto - Cryptography library.
//==========================================================
#ifndef NRF_CRYPTO_ENABLED
#define NRF_CRYPTO_ENABLED 1
#endif
// <o> NRF_CRYPTO_ALLOCATOR  - Memory allocator
 

// <i> Choose memory allocator used by nrf_crypto. Default is alloca if possible or nrf_malloc otherwise. If 'User macros' are selected, the user has to create 'nrf_crypto_allocator.h' file that contains NRF_CRYPTO_ALLOC, NRF_CRYPTO_FREE, and NRF_CRYPTO_ALLOC_ON_STACK.
// <0=> Default 
// <1=> User macros 
// <2=> On stack (alloca) 
// <3=> C dynamic memory (malloc) 
// <4=> SDK Memory Manager (nrf_malloc) 

#ifndef NRF_CRYPTO_ALLOCATOR
#define NRF_CRYPTO_ALLOCATOR 0
#endif

// <e> NRF_CRYPTO_BACKEND_CC310_BL_ENABLED - Enable the ARM Cryptocell CC310 reduced backend.

// <i> The CC310 hardware-accelerated cryptography backend with reduced functionality and footprint (only available on nRF52840).
//==========================================================
#ifndef NRF_CRYPTO_BACKEND_CC310_BL_ENABLED
#define NRF_CRYPTO_BACKEND_CC310_BL_ENABLED 0
#endif
// <q> NRF_CRYPTO_BACKEND_CC310_BL_ECC_SECP224R1_ENABLED  - Enable the secp224r1 elliptic curve support using CC310_BL.
 

#ifndef NRF_CRYPTO_BACKEND_CC310_BL_ECC_SECP224R1_ENABLED
#define NRF_CRYPTO_BACKEND_CC310_BL_ECC_SECP224R1_ENABLED 0
#endif

// <q> NRF_CRYPTO_BACKEND_CC310_BL_ECC_SECP256R1_ENABLED  - Enable the secp256r1 elliptic curve support using CC310_BL.
 

#ifndef NRF_CRYPTO_BACKEND_CC310_BL_ECC_SECP256R1_ENABLED
#define NRF_CRYPTO_BACKEND_CC310_BL_ECC_SECP256R1_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_CC310_BL_HASH_SHA256_ENABLED  - CC310_BL SHA-256 hash functionality.
 

// <i> CC310_BL backend implementation for hardware-accelerated SHA-256.

#ifndef NRF_CRYPTO_BACKEND_CC310_BL_HASH_SHA256_ENABLED
#define NRF_CRYPTO_BACKEND_CC310_BL_HASH_SHA256_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_CC310_BL_HASH_AUTOMATIC_RAM_BUFFER_ENABLED  - nrf_cc310_bl buffers to RAM before running hash operation
 

// <i> Enabling this makes hashing of addresses in FLASH range possible. Size of buffer allocated for hashing is set by NRF_CRYPTO_BACKEND_CC310_BL_HASH_AUTOMATIC_RAM_BUFFER_SIZE

#ifndef NRF_CRYPTO_BACKEND_CC310_BL_HASH_AUTOMATIC_RAM_BUFFER_ENABLED
#define NRF_CRYPTO_BACKEND_CC310_BL_HASH_AUTOMATIC_RAM_BUFFER_ENABLED 0
#endif

// <o> NRF_CRYPTO_BACKEND_CC310_BL_HASH_AUTOMATIC_RAM_BUFFER_SIZE - nrf_cc310_bl hash outputs digests in little endian 
// <i> Makes the nrf_cc310_bl hash functions output digests in little endian format. Only for use in nRF SDK DFU!

#ifndef NRF_CRYPTO_BACKEND_CC310_BL_HASH_AUTOMATIC_RAM_BUFFER_SIZE
#define NRF_CRYPTO_BACKEND_CC310_BL_HASH_AUTOMATIC_RAM_BUFFER_SIZE 4096
#endif

// <q> NRF_CRYPTO_BACKEND_CC310_BL_INTERRUPTS_ENABLED  - Enable Interrupts while support using CC310 bl.
 

// <i> Select a library version compatible with the configuration. When interrupts are disable, a version named _noint must be used

#ifndef NRF_CRYPTO_BACKEND_CC310_BL_INTERRUPTS_ENABLED
#define NRF_CRYPTO_BACKEND_CC310_BL_INTERRUPTS_ENABLED 1
#endif

// </e>

// <e> NRF_CRYPTO_BACKEND_CC310_ENABLED - Enable the ARM Cryptocell CC310 backend.

// <i> The CC310 hardware-accelerated cryptography backend (only available on nRF52840).
//==========================================================
#ifndef NRF_CRYPTO_BACKEND_CC310_ENABLED
#define NRF_CRYPTO_BACKEND_CC310_ENABLED 1
#endif
// <q> NRF_CRYPTO_BACKEND_CC310_AES_CBC_ENABLED  - Enable the AES CBC mode using CC310.
 

#ifndef NRF_CRYPTO_BACKEND_CC310_AES_CBC_ENABLED
#define NRF_CRYPTO_BACKEND_CC310_AES_CBC_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_CC310_AES_CTR_ENABLED  - Enable the AES CTR mode using CC310.
 

#ifndef NRF_CRYPTO_BACKEND_CC310_AES_CTR_ENABLED
#define NRF_CRYPTO_BACKEND_CC310_AES_CTR_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_CC310_AES_ECB_ENABLED  - Enable the AES ECB mode using CC310.
 

#ifndef NRF_CRYPTO_BACKEND_CC310_AES_ECB_ENABLED
#define NRF_CRYPTO_BACKEND_CC310_AES_ECB_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_CC310_AES_CBC_MAC_ENABLED  - Enable the AES CBC_MAC mode using CC310.
 

#ifndef NRF_CRYPTO_BACKEND_CC310_AES_CBC_MAC_ENABLED
#define NRF_CRYPTO_BACKEND_CC310_AES_CBC_MAC_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_CC310_AES_CMAC_ENABLED  - Enable the AES CMAC mode using CC310.
 

#ifndef NRF_CRYPTO_BACKEND_CC310_AES_CMAC_ENABLED
#define NRF_CRYPTO_BACKEND_CC310_AES_CMAC_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_CC310_AES_CCM_ENABLED  - Enable the AES CCM mode using CC310.
 

#ifndef NRF_CRYPTO_BACKEND_CC310_AES_CCM_ENABLED
#define NRF_CRYPTO_BACKEND_CC310_AES_CCM_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_CC310_AES_CCM_STAR_ENABLED  - Enable the AES CCM* mode using CC310.
 

#ifndef NRF_CRYPTO_BACKEND_CC310_AES_CCM_STAR_ENABLED
#define NRF_CRYPTO_BACKEND_CC310_AES_CCM_STAR_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_CC310_CHACHA_POLY_ENABLED  - Enable the CHACHA-POLY mode using CC310.
 

#ifndef NRF_CRYPTO_BACKEND_CC310_CHACHA_POLY_ENABLED
#define NRF_CRYPTO_BACKEND_CC310_CHACHA_POLY_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_CC310_ECC_SECP160R1_ENABLED  - Enable the secp160r1 elliptic curve support using CC310.
 

#ifndef NRF_CRYPTO_BACKEND_CC310_ECC_SECP160R1_ENABLED
#define NRF_CRYPTO_BACKEND_CC310_ECC_SECP160R1_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_CC310_ECC_SECP160R2_ENABLED  - Enable the secp160r2 elliptic curve support using CC310.
 

#ifndef NRF_CRYPTO_BACKEND_CC310_ECC_SECP160R2_ENABLED
#define NRF_CRYPTO_BACKEND_CC310_ECC_SECP160R2_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_CC310_ECC_SECP192R1_ENABLED  - Enable the secp192r1 elliptic curve support using CC310.
 

#ifndef NRF_CRYPTO_BACKEND_CC310_ECC_SECP192R1_ENABLED
#define NRF_CRYPTO_BACKEND_CC310_ECC_SECP192R1_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_CC310_ECC_SECP224R1_ENABLED  - Enable the secp224r1 elliptic curve support using CC310.
 

#ifndef NRF_CRYPTO_BACKEND_CC310_ECC_SECP224R1_ENABLED
#define NRF_CRYPTO_BACKEND_CC310_ECC_SECP224R1_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_CC310_ECC_SECP256R1_ENABLED  - Enable the secp256r1 elliptic curve support using CC310.
 

#ifndef NRF_CRYPTO_BACKEND_CC310_ECC_SECP256R1_ENABLED
#define NRF_CRYPTO_BACKEND_CC310_ECC_SECP256R1_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_CC310_ECC_SECP384R1_ENABLED  - Enable the secp384r1 elliptic curve support using CC310.
 

#ifndef NRF_CRYPTO_BACKEND_CC310_ECC_SECP384R1_ENABLED
#define NRF_CRYPTO_BACKEND_CC310_ECC_SECP384R1_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_CC310_ECC_SECP521R1_ENABLED  - Enable the secp521r1 elliptic curve support using CC310.
 

#ifndef NRF_CRYPTO_BACKEND_CC310_ECC_SECP521R1_ENABLED
#define NRF_CRYPTO_BACKEND_CC310_ECC_SECP521R1_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_CC310_ECC_SECP160K1_ENABLED  - Enable the secp160k1 elliptic curve support using CC310.
 

#ifndef NRF_CRYPTO_BACKEND_CC310_ECC_SECP160K1_ENABLED
#define NRF_CRYPTO_BACKEND_CC310_ECC_SECP160K1_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_CC310_ECC_SECP192K1_ENABLED  - Enable the secp192k1 elliptic curve support using CC310.
 

#ifndef NRF_CRYPTO_BACKEND_CC310_ECC_SECP192K1_ENABLED
#define NRF_CRYPTO_BACKEND_CC310_ECC_SECP192K1_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_CC310_ECC_SECP224K1_ENABLED  - Enable the secp224k1 elliptic curve support using CC310.
 

#ifndef NRF_CRYPTO_BACKEND_CC310_ECC_SECP224K1_ENABLED
#define NRF_CRYPTO_BACKEND_CC310_ECC_SECP224K1_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_CC310_ECC_SECP256K1_ENABLED  - Enable the secp256k1 elliptic curve support using CC310.
 

#ifndef NRF_CRYPTO_BACKEND_CC310_ECC_SECP256K1_ENABLED
#define NRF_CRYPTO_BACKEND_CC310_ECC_SECP256K1_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_CC310_ECC_CURVE25519_ENABLED  - Enable the Curve25519 curve support using CC310.
 

#ifndef NRF_CRYPTO_BACKEND_CC310_ECC_CURVE25519_ENABLED
#define NRF_CRYPTO_BACKEND_CC310_ECC_CURVE25519_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_CC310_ECC_ED25519_ENABLED  - Enable the Ed25519 curve support using CC310.
 

#ifndef NRF_CRYPTO_BACKEND_CC310_ECC_ED25519_ENABLED
#define NRF_CRYPTO_BACKEND_CC310_ECC_ED25519_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_CC310_HASH_SHA256_ENABLED  - CC310 SHA-256 hash functionality.
 

// <i> CC310 backend implementation for hardware-accelerated SHA-256.

#ifndef NRF_CRYPTO_BACKEND_CC310_HASH_SHA256_ENABLED
#define NRF_CRYPTO_BACKEND_CC310_HASH_SHA256_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_CC310_HASH_SHA512_ENABLED  - CC310 SHA-512 hash functionality
 

// <i> CC310 backend implementation for SHA-512 (in software).

#ifndef NRF_CRYPTO_BACKEND_CC310_HASH_SHA512_ENABLED
#define NRF_CRYPTO_BACKEND_CC310_HASH_SHA512_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_CC310_HMAC_SHA256_ENABLED  - CC310 HMAC using SHA-256
 

// <i> CC310 backend implementation for HMAC using hardware-accelerated SHA-256.

#ifndef NRF_CRYPTO_BACKEND_CC310_HMAC_SHA256_ENABLED
#define NRF_CRYPTO_BACKEND_CC310_HMAC_SHA256_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_CC310_HMAC_SHA512_ENABLED  - CC310 HMAC using SHA-512
 

// <i> CC310 backend implementation for HMAC using SHA-512 (in software).

#ifndef NRF_CRYPTO_BACKEND_CC310_HMAC_SHA512_ENABLED
#define NRF_CRYPTO_BACKEND_CC310_HMAC_SHA512_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_CC310_RNG_ENABLED  - Enable RNG support using CC310.
 

#ifndef NRF_CRYPTO_BACKEND_CC310_RNG_ENABLED
#define NRF_CRYPTO_BACKEND_CC310_RNG_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_CC310_INTERRUPTS_ENABLED  - Enable Interrupts while support using CC310.
 

// <i> Select a library version compatible with the configuration. When interrupts are disable, a version named _noint must be used

#ifndef NRF_CRYPTO_BACKEND_CC310_INTERRUPTS_ENABLED
#define NRF_CRYPTO_BACKEND_CC310_INTERRUPTS_ENABLED 1
#endif


// </e>

// <e> NRF_CRYPTO_BACKEND_CIFRA_ENABLED - Enable the Cifra backend.
//==========================================================
#ifndef NRF_CRYPTO_BACKEND_CIFRA_ENABLED
#define NRF_CRYPTO_BACKEND_CIFRA_ENABLED 0
#endif
// <q> NRF_CRYPTO_BACKEND_CIFRA_AES_EAX_ENABLED  - Enable the AES EAX mode using Cifra.
 

#ifndef NRF_CRYPTO_BACKEND_CIFRA_AES_EAX_ENABLED
#define NRF_CRYPTO_BACKEND_CIFRA_AES_EAX_ENABLED 1
#endif

// </e>

// <e> NRF_CRYPTO_BACKEND_MBEDTLS_ENABLED - Enable the mbed TLS backend.
//==========================================================
#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_ENABLED 0
#endif
// <q> NRF_CRYPTO_BACKEND_MBEDTLS_AES_CBC_ENABLED  - Enable the AES CBC mode mbed TLS.
 

#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_AES_CBC_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_AES_CBC_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_MBEDTLS_AES_CTR_ENABLED  - Enable the AES CTR mode using mbed TLS.
 

#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_AES_CTR_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_AES_CTR_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_MBEDTLS_AES_CFB_ENABLED  - Enable the AES CFB mode using mbed TLS.
 

#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_AES_CFB_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_AES_CFB_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_MBEDTLS_AES_ECB_ENABLED  - Enable the AES ECB mode using mbed TLS.
 

#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_AES_ECB_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_AES_ECB_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_MBEDTLS_AES_CBC_MAC_ENABLED  - Enable the AES CBC MAC mode using mbed TLS.
 

#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_AES_CBC_MAC_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_AES_CBC_MAC_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_MBEDTLS_AES_CMAC_ENABLED  - Enable the AES CMAC mode using mbed TLS.
 

#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_AES_CMAC_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_AES_CMAC_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_MBEDTLS_AES_CCM_ENABLED  - Enable the AES CCM mode using mbed TLS.
 

#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_AES_CCM_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_AES_CCM_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_MBEDTLS_AES_GCM_ENABLED  - Enable the AES GCM mode using mbed TLS.
 

#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_AES_GCM_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_AES_GCM_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP192R1_ENABLED  - Enable secp192r1 (NIST 192-bit) curve
 

// <i> Enable this setting if you need secp192r1 (NIST 192-bit) support using MBEDTLS

#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP192R1_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP192R1_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP224R1_ENABLED  - Enable secp224r1 (NIST 224-bit) curve
 

// <i> Enable this setting if you need secp224r1 (NIST 224-bit) support using MBEDTLS

#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP224R1_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP224R1_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP256R1_ENABLED  - Enable secp256r1 (NIST 256-bit) curve
 

// <i> Enable this setting if you need secp256r1 (NIST 256-bit) support using MBEDTLS

#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP256R1_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP256R1_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP384R1_ENABLED  - Enable secp384r1 (NIST 384-bit) curve
 

// <i> Enable this setting if you need secp384r1 (NIST 384-bit) support using MBEDTLS

#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP384R1_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP384R1_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP521R1_ENABLED  - Enable secp521r1 (NIST 521-bit) curve
 

// <i> Enable this setting if you need secp521r1 (NIST 521-bit) support using MBEDTLS

#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP521R1_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP521R1_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP192K1_ENABLED  - Enable secp192k1 (Koblitz 192-bit) curve
 

// <i> Enable this setting if you need secp192k1 (Koblitz 192-bit) support using MBEDTLS

#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP192K1_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP192K1_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP224K1_ENABLED  - Enable secp224k1 (Koblitz 224-bit) curve
 

// <i> Enable this setting if you need secp224k1 (Koblitz 224-bit) support using MBEDTLS

#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP224K1_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP224K1_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP256K1_ENABLED  - Enable secp256k1 (Koblitz 256-bit) curve
 

// <i> Enable this setting if you need secp256k1 (Koblitz 256-bit) support using MBEDTLS

#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP256K1_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP256K1_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_MBEDTLS_ECC_BP256R1_ENABLED  - Enable bp256r1 (Brainpool 256-bit) curve
 

// <i> Enable this setting if you need bp256r1 (Brainpool 256-bit) support using MBEDTLS

#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_ECC_BP256R1_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_ECC_BP256R1_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_MBEDTLS_ECC_BP384R1_ENABLED  - Enable bp384r1 (Brainpool 384-bit) curve
 

// <i> Enable this setting if you need bp384r1 (Brainpool 384-bit) support using MBEDTLS

#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_ECC_BP384R1_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_ECC_BP384R1_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_MBEDTLS_ECC_BP512R1_ENABLED  - Enable bp512r1 (Brainpool 512-bit) curve
 

// <i> Enable this setting if you need bp512r1 (Brainpool 512-bit) support using MBEDTLS

#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_ECC_BP512R1_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_ECC_BP512R1_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_MBEDTLS_ECC_CURVE25519_ENABLED  - Enable Curve25519 curve
 

// <i> Enable this setting if you need Curve25519 support using MBEDTLS

#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_ECC_CURVE25519_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_ECC_CURVE25519_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_MBEDTLS_HASH_SHA256_ENABLED  - Enable mbed TLS SHA-256 hash functionality.
 

// <i> mbed TLS backend implementation for SHA-256.

#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_HASH_SHA256_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_HASH_SHA256_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_MBEDTLS_HASH_SHA512_ENABLED  - Enable mbed TLS SHA-512 hash functionality.
 

// <i> mbed TLS backend implementation for SHA-512.

#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_HASH_SHA512_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_HASH_SHA512_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_MBEDTLS_HMAC_SHA256_ENABLED  - Enable mbed TLS HMAC using SHA-256.
 

// <i> mbed TLS backend implementation for HMAC using SHA-256.

#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_HMAC_SHA256_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_HMAC_SHA256_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_MBEDTLS_HMAC_SHA512_ENABLED  - Enable mbed TLS HMAC using SHA-512.
 

// <i> mbed TLS backend implementation for HMAC using SHA-512.

#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_HMAC_SHA512_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_HMAC_SHA512_ENABLED 1
#endif

// </e>

// <e> NRF_CRYPTO_BACKEND_MICRO_ECC_ENABLED - Enable the micro-ecc backend.
//==========================================================
#ifndef NRF_CRYPTO_BACKEND_MICRO_ECC_ENABLED
#define NRF_CRYPTO_BACKEND_MICRO_ECC_ENABLED 0
#endif
// <q> NRF_CRYPTO_BACKEND_MICRO_ECC_ECC_SECP192R1_ENABLED  - Enable secp192r1 (NIST 192-bit) curve
 

// <i> Enable this setting if you need secp192r1 (NIST 192-bit) support using micro-ecc

#ifndef NRF_CRYPTO_BACKEND_MICRO_ECC_ECC_SECP192R1_ENABLED
#define NRF_CRYPTO_BACKEND_MICRO_ECC_ECC_SECP192R1_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_MICRO_ECC_ECC_SECP224R1_ENABLED  - Enable secp224r1 (NIST 224-bit) curve
 

// <i> Enable this setting if you need secp224r1 (NIST 224-bit) support using micro-ecc

#ifndef NRF_CRYPTO_BACKEND_MICRO_ECC_ECC_SECP224R1_ENABLED
#define NRF_CRYPTO_BACKEND_MICRO_ECC_ECC_SECP224R1_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_MICRO_ECC_ECC_SECP256R1_ENABLED  - Enable secp256r1 (NIST 256-bit) curve
 

// <i> Enable this setting if you need secp256r1 (NIST 256-bit) support using micro-ecc

#ifndef NRF_CRYPTO_BACKEND_MICRO_ECC_ECC_SECP256R1_ENABLED
#define NRF_CRYPTO_BACKEND_MICRO_ECC_ECC_SECP256R1_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_MICRO_ECC_ECC_SECP256K1_ENABLED  - Enable secp256k1 (Koblitz 256-bit) curve
 

// <i> Enable this setting if you need secp256k1 (Koblitz 256-bit) support using micro-ecc

#ifndef NRF_CRYPTO_BACKEND_MICRO_ECC_ECC_SECP256K1_ENABLED
#define NRF_CRYPTO_BACKEND_MICRO_ECC_ECC_SECP256K1_ENABLED 1
#endif

// </e>

// <e> NRF_CRYPTO_BACKEND_NRF_HW_RNG_ENABLED - Enable the nRF HW RNG backend.

// <i> The nRF HW backend provide access to RNG peripheral in nRF5x devices.
//==========================================================
#ifndef NRF_CRYPTO_BACKEND_NRF_HW_RNG_ENABLED
#define NRF_CRYPTO_BACKEND_NRF_HW_RNG_ENABLED 0
#endif
// <q> NRF_CRYPTO_BACKEND_NRF_HW_RNG_MBEDTLS_CTR_DRBG_ENABLED  - Enable mbed TLS CTR-DRBG algorithm.
 

// <i> Enable mbed TLS CTR-DRBG standardized by NIST (NIST SP 800-90A Rev. 1). The nRF HW RNG is used as an entropy source for seeding.

#ifndef NRF_CRYPTO_BACKEND_NRF_HW_RNG_MBEDTLS_CTR_DRBG_ENABLED
#define NRF_CRYPTO_BACKEND_NRF_HW_RNG_MBEDTLS_CTR_DRBG_ENABLED 1
#endif

// </e>

// <e> NRF_CRYPTO_BACKEND_NRF_SW_ENABLED - Enable the legacy nRFx sw for crypto.

// <i> The nRF SW cryptography backend (only used in bootloader context).
//==========================================================
#ifndef NRF_CRYPTO_BACKEND_NRF_SW_ENABLED
#define NRF_CRYPTO_BACKEND_NRF_SW_ENABLED 0
#endif
// <q> NRF_CRYPTO_BACKEND_NRF_SW_HASH_SHA256_ENABLED  - nRF SW hash backend support for SHA-256
 

// <i> The nRF SW backend provide access to nRF SDK legacy hash implementation of SHA-256.

#ifndef NRF_CRYPTO_BACKEND_NRF_SW_HASH_SHA256_ENABLED
#define NRF_CRYPTO_BACKEND_NRF_SW_HASH_SHA256_ENABLED 1
#endif

// </e>

// <e> NRF_CRYPTO_BACKEND_OBERON_ENABLED - Enable the Oberon backend

// <i> The Oberon backend
//==========================================================
#ifndef NRF_CRYPTO_BACKEND_OBERON_ENABLED
#define NRF_CRYPTO_BACKEND_OBERON_ENABLED 0
#endif
// <q> NRF_CRYPTO_BACKEND_OBERON_CHACHA_POLY_ENABLED  - Enable the CHACHA-POLY mode using Oberon.
 

#ifndef NRF_CRYPTO_BACKEND_OBERON_CHACHA_POLY_ENABLED
#define NRF_CRYPTO_BACKEND_OBERON_CHACHA_POLY_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_OBERON_ECC_SECP256R1_ENABLED  - Enable secp256r1 curve
 

// <i> Enable this setting if you need secp256r1 curve support using Oberon library

#ifndef NRF_CRYPTO_BACKEND_OBERON_ECC_SECP256R1_ENABLED
#define NRF_CRYPTO_BACKEND_OBERON_ECC_SECP256R1_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_OBERON_ECC_CURVE25519_ENABLED  - Enable Curve25519 ECDH
 

// <i> Enable this setting if you need Curve25519 ECDH support using Oberon library

#ifndef NRF_CRYPTO_BACKEND_OBERON_ECC_CURVE25519_ENABLED
#define NRF_CRYPTO_BACKEND_OBERON_ECC_CURVE25519_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_OBERON_ECC_ED25519_ENABLED  - Enable Ed25519 signature scheme
 

// <i> Enable this setting if you need Ed25519 support using Oberon library

#ifndef NRF_CRYPTO_BACKEND_OBERON_ECC_ED25519_ENABLED
#define NRF_CRYPTO_BACKEND_OBERON_ECC_ED25519_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_OBERON_HASH_SHA256_ENABLED  - Oberon SHA-256 hash functionality
 

// <i> Oberon backend implementation for SHA-256.

#ifndef NRF_CRYPTO_BACKEND_OBERON_HASH_SHA256_ENABLED
#define NRF_CRYPTO_BACKEND_OBERON_HASH_SHA256_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_OBERON_HASH_SHA512_ENABLED  - Oberon SHA-512 hash functionality
 

// <i> Oberon backend implementation for SHA-512.

#ifndef NRF_CRYPTO_BACKEND_OBERON_HASH_SHA512_ENABLED
#define NRF_CRYPTO_BACKEND_OBERON_HASH_SHA512_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_OBERON_HMAC_SHA256_ENABLED  - Oberon HMAC using SHA-256
 

// <i> Oberon backend implementation for HMAC using SHA-256.

#ifndef NRF_CRYPTO_BACKEND_OBERON_HMAC_SHA256_ENABLED
#define NRF_CRYPTO_BACKEND_OBERON_HMAC_SHA256_ENABLED 1
#endif

// <q> NRF_CRYPTO_BACKEND_OBERON_HMAC_SHA512_ENABLED  - Oberon HMAC using SHA-512
 

// <i> Oberon backend implementation for HMAC using SHA-512.

#ifndef NRF_CRYPTO_BACKEND_OBERON_HMAC_SHA512_ENABLED
#define NRF_CRYPTO_BACKEND_OBERON_HMAC_SHA512_ENABLED 1
#endif

// </e>

// <e> NRF_CRYPTO_BACKEND_OPTIGA_ENABLED - Enable the nrf_crypto Optiga Trust X backend.

// <i> Enables the nrf_crypto backend for Optiga Trust X devices.
//==========================================================
#ifndef NRF_CRYPTO_BACKEND_OPTIGA_ENABLED
#define NRF_CRYPTO_BACKEND_OPTIGA_ENABLED 0
#endif
// <q> NRF_CRYPTO_BACKEND_OPTIGA_RNG_ENABLED  - Optiga backend support for RNG
 

// <i> The Optiga backend provide external chip RNG.

#ifndef NRF_CRYPTO_BACKEND_OPTIGA_RNG_ENABLED
#define NRF_CRYPTO_BACKEND_OPTIGA_RNG_ENABLED 0
#endif

// <q> NRF_CRYPTO_BACKEND_OPTIGA_ECC_SECP256R1_ENABLED  - Optiga backend support for ECC secp256r1
 

// <i> The Optiga backend provide external chip ECC using secp256r1.

#ifndef NRF_CRYPTO_BACKEND_OPTIGA_ECC_SECP256R1_ENABLED
#define NRF_CRYPTO_BACKEND_OPTIGA_ECC_SECP256R1_ENABLED 1
#endif

// </e>

// <q> NRF_CRYPTO_CURVE25519_BIG_ENDIAN_ENABLED  - Big-endian byte order in raw Curve25519 data
 

// <i> Enable big-endian byte order in Curve25519 API, if set to 1. Use little-endian, if set to 0.

#ifndef NRF_CRYPTO_CURVE25519_BIG_ENDIAN_ENABLED
#define NRF_CRYPTO_CURVE25519_BIG_ENDIAN_ENABLED 0
#endif

// </e>
// <h> nrf_crypto_rng - RNG Configuration

//==========================================================
// <q> NRF_CRYPTO_RNG_STATIC_MEMORY_BUFFERS_ENABLED  - Use static memory buffers for context and temporary init buffer.
 

// <i> Always recommended when using the nRF HW RNG as the context and temporary buffers are small. Consider disabling if using the CC310 RNG in a RAM constrained application. In this case, memory must be provided to nrf_crypto_rng_init, or it can be allocated internally provided that NRF_CRYPTO_ALLOCATOR does not allocate memory on the stack.

#ifndef NRF_CRYPTO_RNG_STATIC_MEMORY_BUFFERS_ENABLED
#define NRF_CRYPTO_RNG_STATIC_MEMORY_BUFFERS_ENABLED 1
#endif

// <q> NRF_CRYPTO_RNG_AUTO_INIT_ENABLED  - Initialize the RNG module automatically when nrf_crypto is initialized.
 

// <i> Automatic initialization is only supported with static or internally allocated context and temporary memory.

#ifndef NRF_CRYPTO_RNG_AUTO_INIT_ENABLED
#define NRF_CRYPTO_RNG_AUTO_INIT_ENABLED 1
#endif

// </h> 
//==========================================================

// </h> 
//==========================================================

// <h> nRF_DFU 

//==========================================================
// <h> ble_dfu - Device Firmware Update

//==========================================================
// <q> BLE_DFU_ENABLED  - Enable DFU Service.
 

#ifndef BLE_DFU_ENABLED
#define BLE_DFU_ENABLED 0
#endif

// <q> NRF_DFU_BLE_BUTTONLESS_SUPPORTS_BONDS  - Buttonless DFU supports bonds.
 

#ifndef NRF_DFU_BLE_BUTTONLESS_SUPPORTS_BONDS
#define NRF_DFU_BLE_BUTTONLESS_SUPPORTS_BONDS 0
#endif

// </h> 
//==========================================================

// </h> 
//==========================================================

// <h> nRF_Drivers 

//==========================================================
// <e> COMP_ENABLED - nrf_drv_comp - COMP peripheral driver - legacy layer
//==========================================================
#ifndef COMP_ENABLED
#define COMP_ENABLED 0
#endif
// <o> COMP_CONFIG_REF  - Reference voltage
 
// <0=> Internal 1.2V 
// <1=> Internal 1.8V 
// <2=> Internal 2.4V 
// <4=> VDD 
// <7=> ARef 

#ifndef COMP_CONFIG_REF
#define COMP_CONFIG_REF 1
#endif

// <o> COMP_CONFIG_MAIN_MODE  - Main mode
 
// <0=> Single ended 
// <1=> Differential 

#ifndef COMP_CONFIG_MAIN_MODE
#define COMP_CONFIG_MAIN_MODE 0
#endif

// <o> COMP_CONFIG_SPEED_MODE  - Speed mode
 
// <0=> Low power 
// <1=> Normal 
// <2=> High speed 

#ifndef COMP_CONFIG_SPEED_MODE
#define COMP_CONFIG_SPEED_MODE 2
#endif

// <o> COMP_CONFIG_HYST  - Hystheresis
 
// <0=> No 
// <1=> 50mV 

#ifndef COMP_CONFIG_HYST
#define COMP_CONFIG_HYST 0
#endif

// <o> COMP_CONFIG_ISOURCE  - Current Source
 
// <0=> Off 
// <1=> 2.5 uA 
// <2=> 5 uA 
// <3=> 10 uA 

#ifndef COMP_CONFIG_ISOURCE
#define COMP_CONFIG_ISOURCE 0
#endif

// <o> COMP_CONFIG_INPUT  - Analog input
 
// <0=> 0 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 

#ifndef COMP_CONFIG_INPUT
#define COMP_CONFIG_INPUT 0
#endif

// <o> COMP_CONFIG_IRQ_PRIORITY  - Interrupt priority
 

// <i> Priorities 0,2 (nRF51) and 0,1,4,5 (nRF52) are reserved for SoftDevice
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 

#ifndef COMP_CONFIG_IRQ_PRIORITY
#define COMP_CONFIG_IRQ_PRIORITY 6
#endif

// </e>

// <q> EGU_ENABLED  - nrf_drv_swi - SWI(EGU) peripheral driver - legacy layer
 

#ifndef EGU_ENABLED
#define EGU_ENABLED 0
#endif

// </e>

//==========================================================

// <e> NRFX_TIMER_ENABLED - nrfx_timer - TIMER periperal driver
//==========================================================
#ifndef NRFX_TIMER_ENABLED
#define NRFX_TIMER_ENABLED 1
#endif
// <q> NRFX_TIMER0_ENABLED  - Enable TIMER0 instance
 

#ifndef NRFX_TIMER0_ENABLED
#define NRFX_TIMER0_ENABLED 0
#endif

// <q> NRFX_TIMER1_ENABLED  - Enable TIMER1 instance
 

#ifndef NRFX_TIMER1_ENABLED
#define NRFX_TIMER1_ENABLED 1
#endif

// <q> NRFX_TIMER2_ENABLED  - Enable TIMER2 instance
 

#ifndef NRFX_TIMER2_ENABLED
#define NRFX_TIMER2_ENABLED 0
#endif

// <q> NRFX_TIMER3_ENABLED  - Enable TIMER3 instance
 

#ifndef NRFX_TIMER3_ENABLED
#define NRFX_TIMER3_ENABLED 0
#endif

// <q> NRFX_TIMER4_ENABLED  - Enable TIMER4 instance
 

#ifndef NRFX_TIMER4_ENABLED
#define NRFX_TIMER4_ENABLED 0
#endif

// <o> NRFX_TIMER_DEFAULT_CONFIG_FREQUENCY  - Timer frequency if in Timer mode
 
// <0=> 16 MHz 
// <1=> 8 MHz 
// <2=> 4 MHz 
// <3=> 2 MHz 
// <4=> 1 MHz 
// <5=> 500 kHz 
// <6=> 250 kHz 
// <7=> 125 kHz 
// <8=> 62.5 kHz 
// <9=> 31.25 kHz 

#ifndef NRFX_TIMER_DEFAULT_CONFIG_FREQUENCY
#define NRFX_TIMER_DEFAULT_CONFIG_FREQUENCY 0
#endif

// <o> NRFX_TIMER_DEFAULT_CONFIG_MODE  - Timer mode or operation
 
// <0=> Timer 
// <1=> Counter 

#ifndef NRFX_TIMER_DEFAULT_CONFIG_MODE
#define NRFX_TIMER_DEFAULT_CONFIG_MODE 0
#endif

// <o> NRFX_TIMER_DEFAULT_CONFIG_BIT_WIDTH  - Timer counter bit width
 
// <0=> 16 bit 
// <1=> 8 bit 
// <2=> 24 bit 
// <3=> 32 bit 

#ifndef NRFX_TIMER_DEFAULT_CONFIG_BIT_WIDTH
#define NRFX_TIMER_DEFAULT_CONFIG_BIT_WIDTH 0
#endif

// <o> NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY  - Interrupt priority
 
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 

#ifndef NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY
#define NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY 6
#endif

// <e> NRFX_TIMER_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRFX_TIMER_CONFIG_LOG_ENABLED
#define NRFX_TIMER_CONFIG_LOG_ENABLED 0
#endif
// <o> NRFX_TIMER_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRFX_TIMER_CONFIG_LOG_LEVEL
#define NRFX_TIMER_CONFIG_LOG_LEVEL 3
#endif

// <o> NRFX_TIMER_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRFX_TIMER_CONFIG_INFO_COLOR
#define NRFX_TIMER_CONFIG_INFO_COLOR 0
#endif

// <o> NRFX_TIMER_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRFX_TIMER_CONFIG_DEBUG_COLOR
#define NRFX_TIMER_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// </e>

// <e> TIMER_ENABLED - nrf_drv_timer - TIMER periperal driver - legacy layer
//==========================================================
#ifndef TIMER_ENABLED
#define TIMER_ENABLED 1
#endif
// <o> TIMER_DEFAULT_CONFIG_FREQUENCY  - Timer frequency if in Timer mode
 
// <0=> 16 MHz 
// <1=> 8 MHz 
// <2=> 4 MHz 
// <3=> 2 MHz 
// <4=> 1 MHz 
// <5=> 500 kHz 
// <6=> 250 kHz 
// <7=> 125 kHz 
// <8=> 62.5 kHz 
// <9=> 31.25 kHz 

#ifndef TIMER_DEFAULT_CONFIG_FREQUENCY
#define TIMER_DEFAULT_CONFIG_FREQUENCY 0
#endif

// <o> TIMER_DEFAULT_CONFIG_MODE  - Timer mode or operation
 
// <0=> Timer 
// <1=> Counter 

#ifndef TIMER_DEFAULT_CONFIG_MODE
#define TIMER_DEFAULT_CONFIG_MODE 0
#endif

// <o> TIMER_DEFAULT_CONFIG_BIT_WIDTH  - Timer counter bit width
 
// <0=> 16 bit 
// <1=> 8 bit 
// <2=> 24 bit 
// <3=> 32 bit 

#ifndef TIMER_DEFAULT_CONFIG_BIT_WIDTH
#define TIMER_DEFAULT_CONFIG_BIT_WIDTH 3
#endif

// <o> TIMER_DEFAULT_CONFIG_IRQ_PRIORITY  - Interrupt priority
 

// <i> Priorities 0,2 (nRF51) and 0,1,4,5 (nRF52) are reserved for SoftDevice
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 

#ifndef TIMER_DEFAULT_CONFIG_IRQ_PRIORITY
#define TIMER_DEFAULT_CONFIG_IRQ_PRIORITY 6
#endif

// <q> TIMER0_ENABLED  - Enable TIMER0 instance
 

#ifndef TIMER0_ENABLED
#define TIMER0_ENABLED 0
#endif

// <q> TIMER1_ENABLED  - Enable TIMER1 instance
 

#ifndef TIMER1_ENABLED
#define TIMER1_ENABLED 1
#endif

// <q> TIMER2_ENABLED  - Enable TIMER2 instance
 

#ifndef TIMER2_ENABLED
#define TIMER2_ENABLED 0
#endif

// <q> TIMER3_ENABLED  - Enable TIMER3 instance
 

#ifndef TIMER3_ENABLED
#define TIMER3_ENABLED 0
#endif

// <q> TIMER4_ENABLED  - Enable TIMER4 instance
 

#ifndef TIMER4_ENABLED
#define TIMER4_ENABLED 0
#endif

// </e>

// </h> 
//==========================================================

// <h> nRF_Libraries 

//==========================================================


// <e> NRF_BALLOC_ENABLED - nrf_balloc - Block allocator module
//==========================================================
#ifndef NRF_BALLOC_ENABLED
#define NRF_BALLOC_ENABLED 1
#endif
// <e> NRF_BALLOC_CONFIG_DEBUG_ENABLED - Enables debug mode in the module.
//==========================================================
#ifndef NRF_BALLOC_CONFIG_DEBUG_ENABLED
#define NRF_BALLOC_CONFIG_DEBUG_ENABLED 0
#endif
// <o> NRF_BALLOC_CONFIG_HEAD_GUARD_WORDS - Number of words used as head guard.  <0-255> 


#ifndef NRF_BALLOC_CONFIG_HEAD_GUARD_WORDS
#define NRF_BALLOC_CONFIG_HEAD_GUARD_WORDS 1
#endif

// <o> NRF_BALLOC_CONFIG_TAIL_GUARD_WORDS - Number of words used as tail guard.  <0-255> 


#ifndef NRF_BALLOC_CONFIG_TAIL_GUARD_WORDS
#define NRF_BALLOC_CONFIG_TAIL_GUARD_WORDS 1
#endif

// <q> NRF_BALLOC_CONFIG_BASIC_CHECKS_ENABLED  - Enables basic checks in this module.
 

#ifndef NRF_BALLOC_CONFIG_BASIC_CHECKS_ENABLED
#define NRF_BALLOC_CONFIG_BASIC_CHECKS_ENABLED 0
#endif

// <q> NRF_BALLOC_CONFIG_DOUBLE_FREE_CHECK_ENABLED  - Enables double memory free check in this module.
 

#ifndef NRF_BALLOC_CONFIG_DOUBLE_FREE_CHECK_ENABLED
#define NRF_BALLOC_CONFIG_DOUBLE_FREE_CHECK_ENABLED 0
#endif

// <q> NRF_BALLOC_CONFIG_DATA_TRASHING_CHECK_ENABLED  - Enables free memory corruption check in this module.
 

#ifndef NRF_BALLOC_CONFIG_DATA_TRASHING_CHECK_ENABLED
#define NRF_BALLOC_CONFIG_DATA_TRASHING_CHECK_ENABLED 0
#endif

// <q> NRF_BALLOC_CLI_CMDS  - Enable CLI commands specific to the module
 

#ifndef NRF_BALLOC_CLI_CMDS
#define NRF_BALLOC_CLI_CMDS 0
#endif

// </e>

// </e>

// <q> NRF_BLOCK_DEV_EMPTY_ENABLED  - nrf_block_dev_empty - Empty block device
 

#ifndef NRF_BLOCK_DEV_EMPTY_ENABLED
#define NRF_BLOCK_DEV_EMPTY_ENABLED 1
#endif

// <q> NRF_BLOCK_DEV_QSPI_ENABLED  - nrf_block_dev_qspi - QSPI block device
 

#ifndef NRF_BLOCK_DEV_QSPI_ENABLED
#define NRF_BLOCK_DEV_QSPI_ENABLED 1
#endif

// <q> NRF_BLOCK_DEV_RAM_ENABLED  - nrf_block_dev_ram - RAM block device
 

#ifndef NRF_BLOCK_DEV_RAM_ENABLED
#define NRF_BLOCK_DEV_RAM_ENABLED 1
#endif

// <q> NRF_MEMOBJ_ENABLED  - nrf_memobj - Linked memory allocator module
 

#ifndef NRF_MEMOBJ_ENABLED
#define NRF_MEMOBJ_ENABLED 1
#endif

// <q> NRF_STRERROR_ENABLED  - nrf_strerror - Library for converting error code to string.
 

#ifndef NRF_STRERROR_ENABLED
#define NRF_STRERROR_ENABLED 1
#endif

// <h> nrf_fprintf - fprintf function.

//==========================================================
// <q> NRF_FPRINTF_ENABLED  - Enable/disable fprintf module.
 

#ifndef NRF_FPRINTF_ENABLED
#define NRF_FPRINTF_ENABLED 1
#endif

// <q> NRF_FPRINTF_FLAG_AUTOMATIC_CR_ON_LF_ENABLED  - For each printed LF, function will add CR.
 

#ifndef NRF_FPRINTF_FLAG_AUTOMATIC_CR_ON_LF_ENABLED
#define NRF_FPRINTF_FLAG_AUTOMATIC_CR_ON_LF_ENABLED 0
#endif

// </h> 
//==========================================================

// </h> 
//==========================================================

// <h> Log message pool - Configuration of log message pool

//==========================================================
// <o> NRF_LOG_MSGPOOL_ELEMENT_SIZE - Size of a single element in the pool of memory objects. 
// <i> If a small value is set, then performance of logs processing
// <i> is degraded because data is fragmented. Bigger value impacts
// <i> RAM memory utilization. The size is set to fit a message with
// <i> a timestamp and up to 2 arguments in a single memory object.

#ifndef NRF_LOG_MSGPOOL_ELEMENT_SIZE
#define NRF_LOG_MSGPOOL_ELEMENT_SIZE 20
#endif

// <o> NRF_LOG_MSGPOOL_ELEMENT_COUNT - Number of elements in the pool of memory objects 
// <i> If a small value is set, then it may lead to a deadlock
// <i> in certain cases if backend has high latency and holds
// <i> multiple messages for long time. Bigger value impacts
// <i> RAM memory usage.

#ifndef NRF_LOG_MSGPOOL_ELEMENT_COUNT
#define NRF_LOG_MSGPOOL_ELEMENT_COUNT 8
#endif

// </h> 
//==========================================================

// <q> NRF_LOG_ALLOW_OVERFLOW  - Configures behavior when circular buffer is full.
 

// <i> If set then oldest logs are overwritten. Otherwise a 
// <i> marker is injected informing about overflow.

#ifndef NRF_LOG_ALLOW_OVERFLOW
#define NRF_LOG_ALLOW_OVERFLOW 1
#endif

// <o> NRF_LOG_BUFSIZE  - Size of the buffer for storing logs (in bytes).
 

// <i> Must be power of 2 and multiple of 4.
// <i> If NRF_LOG_DEFERRED = 0 then buffer size can be reduced to minimum.
// <128=> 128 
// <256=> 256 
// <512=> 512 
// <1024=> 1024 
// <2048=> 2048 
// <4096=> 4096 
// <8192=> 8192 
// <16384=> 16384 

#ifndef NRF_LOG_BUFSIZE
#define NRF_LOG_BUFSIZE 1024
#endif

// <q> NRF_LOG_CLI_CMDS  - Enable CLI commands for the module.
 

#ifndef NRF_LOG_CLI_CMDS
#define NRF_LOG_CLI_CMDS 0
#endif

// <o> NRF_LOG_DEFAULT_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_LOG_DEFAULT_LEVEL
#define NRF_LOG_DEFAULT_LEVEL 3
#endif

// <q> NRF_LOG_DEFERRED  - Enable deffered logger.
 

// <i> Log data is buffered and can be processed in idle.

#ifndef NRF_LOG_DEFERRED
#define NRF_LOG_DEFERRED 1
#endif

// <q> NRF_LOG_FILTERS_ENABLED  - Enable dynamic filtering of logs.
 

#ifndef NRF_LOG_FILTERS_ENABLED
#define NRF_LOG_FILTERS_ENABLED 0
#endif

// <o> NRF_LOG_STR_PUSH_BUFFER_SIZE  - Size of the buffer dedicated for strings stored using @ref NRF_LOG_PUSH.
 
// <16=> 16 
// <32=> 32 
// <64=> 64 
// <128=> 128 
// <256=> 256 
// <512=> 512 
// <1024=> 1024 

#ifndef NRF_LOG_STR_PUSH_BUFFER_SIZE
#define NRF_LOG_STR_PUSH_BUFFER_SIZE 128
#endif

// <o> NRF_LOG_STR_PUSH_BUFFER_SIZE  - Size of the buffer dedicated for strings stored using @ref NRF_LOG_PUSH.
 
// <16=> 16 
// <32=> 32 
// <64=> 64 
// <128=> 128 
// <256=> 256 
// <512=> 512 
// <1024=> 1024 

#ifndef NRF_LOG_STR_PUSH_BUFFER_SIZE
#define NRF_LOG_STR_PUSH_BUFFER_SIZE 128
#endif

// <e> NRF_LOG_USES_COLORS - If enabled then ANSI escape code for colors is prefixed to every string
//==========================================================
#ifndef NRF_LOG_USES_COLORS
#define NRF_LOG_USES_COLORS 0
#endif
// <o> NRF_LOG_COLOR_DEFAULT  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_LOG_COLOR_DEFAULT
#define NRF_LOG_COLOR_DEFAULT 0
#endif

// <o> NRF_LOG_ERROR_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_LOG_ERROR_COLOR
#define NRF_LOG_ERROR_COLOR 2
#endif

// <o> NRF_LOG_WARNING_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_LOG_WARNING_COLOR
#define NRF_LOG_WARNING_COLOR 4
#endif

// </e>

// <e> NRF_LOG_USES_TIMESTAMP - Enable timestamping

// <i> Function for getting the timestamp is provided by the user
//==========================================================
#ifndef NRF_LOG_USES_TIMESTAMP
#define NRF_LOG_USES_TIMESTAMP 0
#endif

										   
											 
	  

// </e>

// <h> nrf_log module configuration 

//==========================================================
// <h> nrf_log in nRF_Core 

//==========================================================
// <e> NRF_MPU_LIB_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRF_MPU_LIB_CONFIG_LOG_ENABLED
#define NRF_MPU_LIB_CONFIG_LOG_ENABLED 0
#endif
// <o> NRF_MPU_LIB_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_MPU_LIB_CONFIG_LOG_LEVEL
#define NRF_MPU_LIB_CONFIG_LOG_LEVEL 3
#endif

// <o> NRF_MPU_LIB_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_MPU_LIB_CONFIG_INFO_COLOR
#define NRF_MPU_LIB_CONFIG_INFO_COLOR 0
#endif

// <o> NRF_MPU_LIB_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_MPU_LIB_CONFIG_DEBUG_COLOR
#define NRF_MPU_LIB_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> NRF_STACK_GUARD_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRF_STACK_GUARD_CONFIG_LOG_ENABLED
#define NRF_STACK_GUARD_CONFIG_LOG_ENABLED 0
#endif
// <o> NRF_STACK_GUARD_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_STACK_GUARD_CONFIG_LOG_LEVEL
#define NRF_STACK_GUARD_CONFIG_LOG_LEVEL 3
#endif

// <o> NRF_STACK_GUARD_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_STACK_GUARD_CONFIG_INFO_COLOR
#define NRF_STACK_GUARD_CONFIG_INFO_COLOR 0
#endif

// <o> NRF_STACK_GUARD_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_STACK_GUARD_CONFIG_DEBUG_COLOR
#define NRF_STACK_GUARD_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> TASK_MANAGER_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef TASK_MANAGER_CONFIG_LOG_ENABLED
#define TASK_MANAGER_CONFIG_LOG_ENABLED 0
#endif
// <o> TASK_MANAGER_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef TASK_MANAGER_CONFIG_LOG_LEVEL
#define TASK_MANAGER_CONFIG_LOG_LEVEL 3
#endif

// <o> TASK_MANAGER_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef TASK_MANAGER_CONFIG_INFO_COLOR
#define TASK_MANAGER_CONFIG_INFO_COLOR 0
#endif

// <o> TASK_MANAGER_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef TASK_MANAGER_CONFIG_DEBUG_COLOR
#define TASK_MANAGER_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// </h> 
//==========================================================

// <h> nrf_log in nRF_Drivers 

//==========================================================
// <e> CLOCK_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef CLOCK_CONFIG_LOG_ENABLED
#define CLOCK_CONFIG_LOG_ENABLED 0
#endif
// <o> CLOCK_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef CLOCK_CONFIG_LOG_LEVEL
#define CLOCK_CONFIG_LOG_LEVEL 3
#endif

// <o> CLOCK_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef CLOCK_CONFIG_INFO_COLOR
#define CLOCK_CONFIG_INFO_COLOR 0
#endif

// <o> CLOCK_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef CLOCK_CONFIG_DEBUG_COLOR
#define CLOCK_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> COMP_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef COMP_CONFIG_LOG_ENABLED
#define COMP_CONFIG_LOG_ENABLED 0
#endif
// <o> COMP_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef COMP_CONFIG_LOG_LEVEL
#define COMP_CONFIG_LOG_LEVEL 3
#endif

// <o> COMP_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef COMP_CONFIG_INFO_COLOR
#define COMP_CONFIG_INFO_COLOR 0
#endif

// <o> COMP_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef COMP_CONFIG_DEBUG_COLOR
#define COMP_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> GPIOTE_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef GPIOTE_CONFIG_LOG_ENABLED
#define GPIOTE_CONFIG_LOG_ENABLED 0
#endif
// <o> GPIOTE_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef GPIOTE_CONFIG_LOG_LEVEL
#define GPIOTE_CONFIG_LOG_LEVEL 3
#endif

// <o> GPIOTE_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef GPIOTE_CONFIG_INFO_COLOR
#define GPIOTE_CONFIG_INFO_COLOR 0
#endif

// <o> GPIOTE_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef GPIOTE_CONFIG_DEBUG_COLOR
#define GPIOTE_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> LPCOMP_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef LPCOMP_CONFIG_LOG_ENABLED
#define LPCOMP_CONFIG_LOG_ENABLED 0
#endif
// <o> LPCOMP_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef LPCOMP_CONFIG_LOG_LEVEL
#define LPCOMP_CONFIG_LOG_LEVEL 3
#endif

// <o> LPCOMP_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef LPCOMP_CONFIG_INFO_COLOR
#define LPCOMP_CONFIG_INFO_COLOR 0
#endif

// <o> LPCOMP_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef LPCOMP_CONFIG_DEBUG_COLOR
#define LPCOMP_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> MAX3421E_HOST_CONFIG_LOG_ENABLED - Enable logging in the module
//==========================================================
#ifndef MAX3421E_HOST_CONFIG_LOG_ENABLED
#define MAX3421E_HOST_CONFIG_LOG_ENABLED 0
#endif
// <o> MAX3421E_HOST_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef MAX3421E_HOST_CONFIG_LOG_LEVEL
#define MAX3421E_HOST_CONFIG_LOG_LEVEL 3
#endif

// <o> MAX3421E_HOST_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef MAX3421E_HOST_CONFIG_INFO_COLOR
#define MAX3421E_HOST_CONFIG_INFO_COLOR 0
#endif

// <o> MAX3421E_HOST_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef MAX3421E_HOST_CONFIG_DEBUG_COLOR
#define MAX3421E_HOST_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> NRFX_USBD_CONFIG_LOG_ENABLED - Enable logging in the module
//==========================================================
#ifndef NRFX_USBD_CONFIG_LOG_ENABLED
#define NRFX_USBD_CONFIG_LOG_ENABLED 0
#endif
// <o> NRFX_USBD_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRFX_USBD_CONFIG_LOG_LEVEL
#define NRFX_USBD_CONFIG_LOG_LEVEL 3
#endif

// <o> NRFX_USBD_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRFX_USBD_CONFIG_INFO_COLOR
#define NRFX_USBD_CONFIG_INFO_COLOR 0
#endif

// <o> NRFX_USBD_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRFX_USBD_CONFIG_DEBUG_COLOR
#define NRFX_USBD_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> PDM_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef PDM_CONFIG_LOG_ENABLED
#define PDM_CONFIG_LOG_ENABLED 0
#endif
// <o> PDM_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef PDM_CONFIG_LOG_LEVEL
#define PDM_CONFIG_LOG_LEVEL 3
#endif

// <o> PDM_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef PDM_CONFIG_INFO_COLOR
#define PDM_CONFIG_INFO_COLOR 0
#endif

// <o> PDM_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef PDM_CONFIG_DEBUG_COLOR
#define PDM_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> PPI_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef PPI_CONFIG_LOG_ENABLED
#define PPI_CONFIG_LOG_ENABLED 0
#endif
// <o> PPI_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef PPI_CONFIG_LOG_LEVEL
#define PPI_CONFIG_LOG_LEVEL 3
#endif

// <o> PPI_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef PPI_CONFIG_INFO_COLOR
#define PPI_CONFIG_INFO_COLOR 0
#endif

// <o> PPI_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef PPI_CONFIG_DEBUG_COLOR
#define PPI_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> PWM_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef PWM_CONFIG_LOG_ENABLED
#define PWM_CONFIG_LOG_ENABLED 0
#endif
// <o> PWM_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef PWM_CONFIG_LOG_LEVEL
#define PWM_CONFIG_LOG_LEVEL 3
#endif

// <o> PWM_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef PWM_CONFIG_INFO_COLOR
#define PWM_CONFIG_INFO_COLOR 0
#endif

// <o> PWM_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef PWM_CONFIG_DEBUG_COLOR
#define PWM_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> QDEC_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef QDEC_CONFIG_LOG_ENABLED
#define QDEC_CONFIG_LOG_ENABLED 0
#endif
// <o> QDEC_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef QDEC_CONFIG_LOG_LEVEL
#define QDEC_CONFIG_LOG_LEVEL 3
#endif

// <o> QDEC_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef QDEC_CONFIG_INFO_COLOR
#define QDEC_CONFIG_INFO_COLOR 0
#endif

// <o> QDEC_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef QDEC_CONFIG_DEBUG_COLOR
#define QDEC_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> RNG_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef RNG_CONFIG_LOG_ENABLED
#define RNG_CONFIG_LOG_ENABLED 0
#endif
// <o> RNG_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef RNG_CONFIG_LOG_LEVEL
#define RNG_CONFIG_LOG_LEVEL 3
#endif

// <o> RNG_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef RNG_CONFIG_INFO_COLOR
#define RNG_CONFIG_INFO_COLOR 0
#endif

// <o> RNG_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef RNG_CONFIG_DEBUG_COLOR
#define RNG_CONFIG_DEBUG_COLOR 0
#endif

// <q> RNG_CONFIG_RANDOM_NUMBER_LOG_ENABLED  - Enables logging of random numbers.
 

#ifndef RNG_CONFIG_RANDOM_NUMBER_LOG_ENABLED
#define RNG_CONFIG_RANDOM_NUMBER_LOG_ENABLED 0
#endif

// </e>

// <e> RTC_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef RTC_CONFIG_LOG_ENABLED
#define RTC_CONFIG_LOG_ENABLED 0
#endif
// <o> RTC_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef RTC_CONFIG_LOG_LEVEL
#define RTC_CONFIG_LOG_LEVEL 3
#endif

// <o> RTC_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef RTC_CONFIG_INFO_COLOR
#define RTC_CONFIG_INFO_COLOR 0
#endif

// <o> RTC_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef RTC_CONFIG_DEBUG_COLOR
#define RTC_CONFIG_DEBUG_COLOR 0
#endif

// <e> NRFX_SAADC_ENABLED - nrfx_saadc - SAADC peripheral driver
//==========================================================
#ifndef NRFX_SAADC_ENABLED
#define NRFX_SAADC_ENABLED 1
#endif
// <o> NRFX_SAADC_CONFIG_RESOLUTION  - Resolution
 
// <0=> 8 bit 
// <1=> 10 bit 
// <2=> 12 bit 
// <3=> 14 bit 

#ifndef NRFX_SAADC_CONFIG_RESOLUTION
#define NRFX_SAADC_CONFIG_RESOLUTION 2
#endif

// <o> NRFX_SAADC_CONFIG_OVERSAMPLE  - Sample period
 
// <0=> Disabled 
// <1=> 2x 
// <2=> 4x 
// <3=> 8x 
// <4=> 16x 
// <5=> 32x 
// <6=> 64x 
// <7=> 128x 
// <8=> 256x 

#ifndef NRFX_SAADC_CONFIG_OVERSAMPLE
#define NRFX_SAADC_CONFIG_OVERSAMPLE 0
#endif

// <q> NRFX_SAADC_CONFIG_LP_MODE  - Enabling low power mode
 

#ifndef NRFX_SAADC_CONFIG_LP_MODE
#define NRFX_SAADC_CONFIG_LP_MODE 0
#endif

// <o> NRFX_SAADC_CONFIG_IRQ_PRIORITY  - Interrupt priority
 
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 

#ifndef NRFX_SAADC_CONFIG_IRQ_PRIORITY
#define NRFX_SAADC_CONFIG_IRQ_PRIORITY 6
#endif

// </e>

// <e> SAADC_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef SAADC_CONFIG_LOG_ENABLED
#define SAADC_CONFIG_LOG_ENABLED 0
#endif
// <o> SAADC_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef SAADC_CONFIG_LOG_LEVEL
#define SAADC_CONFIG_LOG_LEVEL 3
#endif

// <o> SAADC_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef SAADC_CONFIG_INFO_COLOR
#define SAADC_CONFIG_INFO_COLOR 0
#endif

// <o> SAADC_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef SAADC_CONFIG_DEBUG_COLOR
#define SAADC_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> SPIS_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef SPIS_CONFIG_LOG_ENABLED
#define SPIS_CONFIG_LOG_ENABLED 0
#endif
// <o> SPIS_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef SPIS_CONFIG_LOG_LEVEL
#define SPIS_CONFIG_LOG_LEVEL 3
#endif

// <o> SPIS_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef SPIS_CONFIG_INFO_COLOR
#define SPIS_CONFIG_INFO_COLOR 0
#endif

// <o> SPIS_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef SPIS_CONFIG_DEBUG_COLOR
#define SPIS_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> SPI_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef SPI_CONFIG_LOG_ENABLED
#define SPI_CONFIG_LOG_ENABLED 0
#endif
// <o> SPI_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef SPI_CONFIG_LOG_LEVEL
#define SPI_CONFIG_LOG_LEVEL 3
#endif

// <o> SPI_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef SPI_CONFIG_INFO_COLOR
#define SPI_CONFIG_INFO_COLOR 0
#endif

// <o> SPI_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef SPI_CONFIG_DEBUG_COLOR
#define SPI_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> TIMER_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef TIMER_CONFIG_LOG_ENABLED
#define TIMER_CONFIG_LOG_ENABLED 0
#endif
// <o> TIMER_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef TIMER_CONFIG_LOG_LEVEL
#define TIMER_CONFIG_LOG_LEVEL 3
#endif

// <o> TIMER_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef TIMER_CONFIG_INFO_COLOR
#define TIMER_CONFIG_INFO_COLOR 0
#endif

// <o> TIMER_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef TIMER_CONFIG_DEBUG_COLOR
#define TIMER_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> TWIS_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef TWIS_CONFIG_LOG_ENABLED
#define TWIS_CONFIG_LOG_ENABLED 0
#endif
// <o> TWIS_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef TWIS_CONFIG_LOG_LEVEL
#define TWIS_CONFIG_LOG_LEVEL 3
#endif

// <o> TWIS_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef TWIS_CONFIG_INFO_COLOR
#define TWIS_CONFIG_INFO_COLOR 0
#endif

// <o> TWIS_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef TWIS_CONFIG_DEBUG_COLOR
#define TWIS_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> TWI_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef TWI_CONFIG_LOG_ENABLED
#define TWI_CONFIG_LOG_ENABLED 0
#endif
// <o> TWI_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef TWI_CONFIG_LOG_LEVEL
#define TWI_CONFIG_LOG_LEVEL 3
#endif

// <o> TWI_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef TWI_CONFIG_INFO_COLOR
#define TWI_CONFIG_INFO_COLOR 0
#endif

// <o> TWI_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef TWI_CONFIG_DEBUG_COLOR
#define TWI_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> UART_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef UART_CONFIG_LOG_ENABLED
#define UART_CONFIG_LOG_ENABLED 0
#endif
// <o> UART_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef UART_CONFIG_LOG_LEVEL
#define UART_CONFIG_LOG_LEVEL 3
#endif

// <o> UART_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef UART_CONFIG_INFO_COLOR
#define UART_CONFIG_INFO_COLOR 0
#endif

// <o> UART_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef UART_CONFIG_DEBUG_COLOR
#define UART_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> USBD_CONFIG_LOG_ENABLED - Enable logging in the module
//==========================================================
#ifndef USBD_CONFIG_LOG_ENABLED
#define USBD_CONFIG_LOG_ENABLED 0
#endif
// <o> USBD_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef USBD_CONFIG_LOG_LEVEL
#define USBD_CONFIG_LOG_LEVEL 3
#endif

// <o> USBD_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef USBD_CONFIG_INFO_COLOR
#define USBD_CONFIG_INFO_COLOR 0
#endif

// <o> USBD_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef USBD_CONFIG_DEBUG_COLOR
#define USBD_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> WDT_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef WDT_CONFIG_LOG_ENABLED
#define WDT_CONFIG_LOG_ENABLED 0
#endif
// <o> WDT_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef WDT_CONFIG_LOG_LEVEL
#define WDT_CONFIG_LOG_LEVEL 3
#endif

// <o> WDT_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef WDT_CONFIG_INFO_COLOR
#define WDT_CONFIG_INFO_COLOR 0
#endif

// <o> WDT_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef WDT_CONFIG_DEBUG_COLOR
#define WDT_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// </h> 
//==========================================================

// <h> nrf_log in nRF_Libraries 

//==========================================================
// <e> APP_TIMER_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef APP_TIMER_CONFIG_LOG_ENABLED
#define APP_TIMER_CONFIG_LOG_ENABLED 0
#endif
// <o> APP_TIMER_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef APP_TIMER_CONFIG_LOG_LEVEL
#define APP_TIMER_CONFIG_LOG_LEVEL 3
#endif

// <o> APP_TIMER_CONFIG_INITIAL_LOG_LEVEL  - Initial severity level if dynamic filtering is enabled.
 

// <i> If module generates a lot of logs, initial log level can
// <i> be decreased to prevent flooding. Severity level can be
// <i> increased on instance basis.
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef APP_TIMER_CONFIG_INITIAL_LOG_LEVEL
#define APP_TIMER_CONFIG_INITIAL_LOG_LEVEL 3
#endif

// <o> APP_TIMER_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef APP_TIMER_CONFIG_INFO_COLOR
#define APP_TIMER_CONFIG_INFO_COLOR 0
#endif

// <o> APP_TIMER_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef APP_TIMER_CONFIG_DEBUG_COLOR
#define APP_TIMER_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> APP_USBD_CDC_ACM_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef APP_USBD_CDC_ACM_CONFIG_LOG_ENABLED
#define APP_USBD_CDC_ACM_CONFIG_LOG_ENABLED 0
#endif
// <o> APP_USBD_CDC_ACM_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef APP_USBD_CDC_ACM_CONFIG_LOG_LEVEL
#define APP_USBD_CDC_ACM_CONFIG_LOG_LEVEL 3
#endif

// <o> APP_USBD_CDC_ACM_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef APP_USBD_CDC_ACM_CONFIG_INFO_COLOR
#define APP_USBD_CDC_ACM_CONFIG_INFO_COLOR 0
#endif

// <o> APP_USBD_CDC_ACM_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef APP_USBD_CDC_ACM_CONFIG_DEBUG_COLOR
#define APP_USBD_CDC_ACM_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> APP_USBD_CONFIG_LOG_ENABLED - Enable logging in the module.
//==========================================================
#ifndef APP_USBD_CONFIG_LOG_ENABLED
#define APP_USBD_CONFIG_LOG_ENABLED 0
#endif
// <o> APP_USBD_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef APP_USBD_CONFIG_LOG_LEVEL
#define APP_USBD_CONFIG_LOG_LEVEL 3
#endif

// <o> APP_USBD_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef APP_USBD_CONFIG_INFO_COLOR
#define APP_USBD_CONFIG_INFO_COLOR 0
#endif

// <o> APP_USBD_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef APP_USBD_CONFIG_DEBUG_COLOR
#define APP_USBD_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> APP_USBD_DUMMY_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef APP_USBD_DUMMY_CONFIG_LOG_ENABLED
#define APP_USBD_DUMMY_CONFIG_LOG_ENABLED 0
#endif
// <o> APP_USBD_DUMMY_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef APP_USBD_DUMMY_CONFIG_LOG_LEVEL
#define APP_USBD_DUMMY_CONFIG_LOG_LEVEL 3
#endif

// <o> APP_USBD_DUMMY_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef APP_USBD_DUMMY_CONFIG_INFO_COLOR
#define APP_USBD_DUMMY_CONFIG_INFO_COLOR 0
#endif

// <o> APP_USBD_DUMMY_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef APP_USBD_DUMMY_CONFIG_DEBUG_COLOR
#define APP_USBD_DUMMY_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> APP_USBD_MSC_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef APP_USBD_MSC_CONFIG_LOG_ENABLED
#define APP_USBD_MSC_CONFIG_LOG_ENABLED 0
#endif
// <o> APP_USBD_MSC_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef APP_USBD_MSC_CONFIG_LOG_LEVEL
#define APP_USBD_MSC_CONFIG_LOG_LEVEL 3
#endif

// <o> APP_USBD_MSC_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef APP_USBD_MSC_CONFIG_INFO_COLOR
#define APP_USBD_MSC_CONFIG_INFO_COLOR 0
#endif

// <o> APP_USBD_MSC_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef APP_USBD_MSC_CONFIG_DEBUG_COLOR
#define APP_USBD_MSC_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> APP_USBD_NRF_DFU_TRIGGER_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef APP_USBD_NRF_DFU_TRIGGER_CONFIG_LOG_ENABLED
#define APP_USBD_NRF_DFU_TRIGGER_CONFIG_LOG_ENABLED 0
#endif
// <o> APP_USBD_NRF_DFU_TRIGGER_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef APP_USBD_NRF_DFU_TRIGGER_CONFIG_LOG_LEVEL
#define APP_USBD_NRF_DFU_TRIGGER_CONFIG_LOG_LEVEL 3
#endif

// <o> APP_USBD_NRF_DFU_TRIGGER_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef APP_USBD_NRF_DFU_TRIGGER_CONFIG_INFO_COLOR
#define APP_USBD_NRF_DFU_TRIGGER_CONFIG_INFO_COLOR 0
#endif

// <o> APP_USBD_NRF_DFU_TRIGGER_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef APP_USBD_NRF_DFU_TRIGGER_CONFIG_DEBUG_COLOR
#define APP_USBD_NRF_DFU_TRIGGER_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> NRF_ATFIFO_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRF_ATFIFO_CONFIG_LOG_ENABLED
#define NRF_ATFIFO_CONFIG_LOG_ENABLED 0
#endif
// <o> NRF_ATFIFO_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_ATFIFO_CONFIG_LOG_LEVEL
#define NRF_ATFIFO_CONFIG_LOG_LEVEL 3
#endif

// <o> NRF_ATFIFO_CONFIG_LOG_INIT_FILTER_LEVEL  - Initial severity level if dynamic filtering is enabled
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_ATFIFO_CONFIG_LOG_INIT_FILTER_LEVEL
#define NRF_ATFIFO_CONFIG_LOG_INIT_FILTER_LEVEL 3
#endif

// <o> NRF_ATFIFO_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_ATFIFO_CONFIG_INFO_COLOR
#define NRF_ATFIFO_CONFIG_INFO_COLOR 0
#endif

// <o> NRF_ATFIFO_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_ATFIFO_CONFIG_DEBUG_COLOR
#define NRF_ATFIFO_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> NRF_BALLOC_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRF_BALLOC_CONFIG_LOG_ENABLED
#define NRF_BALLOC_CONFIG_LOG_ENABLED 0
#endif
// <o> NRF_BALLOC_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_BALLOC_CONFIG_LOG_LEVEL
#define NRF_BALLOC_CONFIG_LOG_LEVEL 3
#endif

// <o> NRF_BALLOC_CONFIG_INITIAL_LOG_LEVEL  - Initial severity level if dynamic filtering is enabled.
 

// <i> If module generates a lot of logs, initial log level can
// <i> be decreased to prevent flooding. Severity level can be
// <i> increased on instance basis.
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_BALLOC_CONFIG_INITIAL_LOG_LEVEL
#define NRF_BALLOC_CONFIG_INITIAL_LOG_LEVEL 3
#endif

// <o> NRF_BALLOC_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_BALLOC_CONFIG_INFO_COLOR
#define NRF_BALLOC_CONFIG_INFO_COLOR 0
#endif

// <o> NRF_BALLOC_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_BALLOC_CONFIG_DEBUG_COLOR
#define NRF_BALLOC_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> NRF_BLOCK_DEV_EMPTY_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRF_BLOCK_DEV_EMPTY_CONFIG_LOG_ENABLED
#define NRF_BLOCK_DEV_EMPTY_CONFIG_LOG_ENABLED 0
#endif
// <o> NRF_BLOCK_DEV_EMPTY_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_BLOCK_DEV_EMPTY_CONFIG_LOG_LEVEL
#define NRF_BLOCK_DEV_EMPTY_CONFIG_LOG_LEVEL 3
#endif

// <o> NRF_BLOCK_DEV_EMPTY_CONFIG_LOG_INIT_FILTER_LEVEL  - Initial severity level if dynamic filtering is enabled
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_BLOCK_DEV_EMPTY_CONFIG_LOG_INIT_FILTER_LEVEL
#define NRF_BLOCK_DEV_EMPTY_CONFIG_LOG_INIT_FILTER_LEVEL 3
#endif

// <o> NRF_BLOCK_DEV_EMPTY_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_BLOCK_DEV_EMPTY_CONFIG_INFO_COLOR
#define NRF_BLOCK_DEV_EMPTY_CONFIG_INFO_COLOR 0
#endif

// <o> NRF_BLOCK_DEV_EMPTY_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_BLOCK_DEV_EMPTY_CONFIG_DEBUG_COLOR
#define NRF_BLOCK_DEV_EMPTY_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> NRF_BLOCK_DEV_QSPI_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRF_BLOCK_DEV_QSPI_CONFIG_LOG_ENABLED
#define NRF_BLOCK_DEV_QSPI_CONFIG_LOG_ENABLED 0
#endif
// <o> NRF_BLOCK_DEV_QSPI_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_BLOCK_DEV_QSPI_CONFIG_LOG_LEVEL
#define NRF_BLOCK_DEV_QSPI_CONFIG_LOG_LEVEL 3
#endif

// <o> NRF_BLOCK_DEV_QSPI_CONFIG_LOG_INIT_FILTER_LEVEL  - Initial severity level if dynamic filtering is enabled
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_BLOCK_DEV_QSPI_CONFIG_LOG_INIT_FILTER_LEVEL
#define NRF_BLOCK_DEV_QSPI_CONFIG_LOG_INIT_FILTER_LEVEL 3
#endif

// <o> NRF_BLOCK_DEV_QSPI_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_BLOCK_DEV_QSPI_CONFIG_INFO_COLOR
#define NRF_BLOCK_DEV_QSPI_CONFIG_INFO_COLOR 0
#endif

// <o> NRF_BLOCK_DEV_QSPI_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_BLOCK_DEV_QSPI_CONFIG_DEBUG_COLOR
#define NRF_BLOCK_DEV_QSPI_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> NRF_BLOCK_DEV_RAM_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRF_BLOCK_DEV_RAM_CONFIG_LOG_ENABLED
#define NRF_BLOCK_DEV_RAM_CONFIG_LOG_ENABLED 0
#endif
// <o> NRF_BLOCK_DEV_RAM_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_BLOCK_DEV_RAM_CONFIG_LOG_LEVEL
#define NRF_BLOCK_DEV_RAM_CONFIG_LOG_LEVEL 3
#endif

// <o> NRF_BLOCK_DEV_RAM_CONFIG_LOG_INIT_FILTER_LEVEL  - Initial severity level if dynamic filtering is enabled
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_BLOCK_DEV_RAM_CONFIG_LOG_INIT_FILTER_LEVEL
#define NRF_BLOCK_DEV_RAM_CONFIG_LOG_INIT_FILTER_LEVEL 3
#endif

// <o> NRF_BLOCK_DEV_RAM_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_BLOCK_DEV_RAM_CONFIG_INFO_COLOR
#define NRF_BLOCK_DEV_RAM_CONFIG_INFO_COLOR 0
#endif

// <o> NRF_BLOCK_DEV_RAM_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_BLOCK_DEV_RAM_CONFIG_DEBUG_COLOR
#define NRF_BLOCK_DEV_RAM_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> NRF_CLI_BLE_UART_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRF_CLI_BLE_UART_CONFIG_LOG_ENABLED
#define NRF_CLI_BLE_UART_CONFIG_LOG_ENABLED 0
#endif
// <o> NRF_CLI_BLE_UART_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_CLI_BLE_UART_CONFIG_LOG_LEVEL
#define NRF_CLI_BLE_UART_CONFIG_LOG_LEVEL 3
#endif

// <o> NRF_CLI_BLE_UART_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_CLI_BLE_UART_CONFIG_INFO_COLOR
#define NRF_CLI_BLE_UART_CONFIG_INFO_COLOR 0
#endif

// <o> NRF_CLI_BLE_UART_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_CLI_BLE_UART_CONFIG_DEBUG_COLOR
#define NRF_CLI_BLE_UART_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> NRF_CLI_LIBUARTE_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRF_CLI_LIBUARTE_CONFIG_LOG_ENABLED
#define NRF_CLI_LIBUARTE_CONFIG_LOG_ENABLED 0
#endif
// <o> NRF_CLI_LIBUARTE_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_CLI_LIBUARTE_CONFIG_LOG_LEVEL
#define NRF_CLI_LIBUARTE_CONFIG_LOG_LEVEL 3
#endif

// <o> NRF_CLI_LIBUARTE_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_CLI_LIBUARTE_CONFIG_INFO_COLOR
#define NRF_CLI_LIBUARTE_CONFIG_INFO_COLOR 0
#endif

// <o> NRF_CLI_LIBUARTE_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_CLI_LIBUARTE_CONFIG_DEBUG_COLOR
#define NRF_CLI_LIBUARTE_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> NRF_CLI_UART_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRF_CLI_UART_CONFIG_LOG_ENABLED
#define NRF_CLI_UART_CONFIG_LOG_ENABLED 0
#endif
// <o> NRF_CLI_UART_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_CLI_UART_CONFIG_LOG_LEVEL
#define NRF_CLI_UART_CONFIG_LOG_LEVEL 3
#endif

// <o> NRF_CLI_UART_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_CLI_UART_CONFIG_INFO_COLOR
#define NRF_CLI_UART_CONFIG_INFO_COLOR 0
#endif

// <o> NRF_CLI_UART_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_CLI_UART_CONFIG_DEBUG_COLOR
#define NRF_CLI_UART_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> NRF_LIBUARTE_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRF_LIBUARTE_CONFIG_LOG_ENABLED
#define NRF_LIBUARTE_CONFIG_LOG_ENABLED 0
#endif
// <o> NRF_LIBUARTE_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_LIBUARTE_CONFIG_LOG_LEVEL
#define NRF_LIBUARTE_CONFIG_LOG_LEVEL 3
#endif

// <o> NRF_LIBUARTE_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_LIBUARTE_CONFIG_INFO_COLOR
#define NRF_LIBUARTE_CONFIG_INFO_COLOR 0
#endif

// <o> NRF_LIBUARTE_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_LIBUARTE_CONFIG_DEBUG_COLOR
#define NRF_LIBUARTE_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> NRF_MEMOBJ_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRF_MEMOBJ_CONFIG_LOG_ENABLED
#define NRF_MEMOBJ_CONFIG_LOG_ENABLED 0
#endif
// <o> NRF_MEMOBJ_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_MEMOBJ_CONFIG_LOG_LEVEL
#define NRF_MEMOBJ_CONFIG_LOG_LEVEL 3
#endif

// <o> NRF_MEMOBJ_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_MEMOBJ_CONFIG_INFO_COLOR
#define NRF_MEMOBJ_CONFIG_INFO_COLOR 0
#endif

// <o> NRF_MEMOBJ_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_MEMOBJ_CONFIG_DEBUG_COLOR
#define NRF_MEMOBJ_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> NRF_PWR_MGMT_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRF_PWR_MGMT_CONFIG_LOG_ENABLED
#define NRF_PWR_MGMT_CONFIG_LOG_ENABLED 0
#endif
// <o> NRF_PWR_MGMT_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_PWR_MGMT_CONFIG_LOG_LEVEL
#define NRF_PWR_MGMT_CONFIG_LOG_LEVEL 3
#endif

// <o> NRF_PWR_MGMT_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_PWR_MGMT_CONFIG_INFO_COLOR
#define NRF_PWR_MGMT_CONFIG_INFO_COLOR 0
#endif

// <o> NRF_PWR_MGMT_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_PWR_MGMT_CONFIG_DEBUG_COLOR
#define NRF_PWR_MGMT_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> NRF_QUEUE_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRF_QUEUE_CONFIG_LOG_ENABLED
#define NRF_QUEUE_CONFIG_LOG_ENABLED 0
#endif
// <o> NRF_QUEUE_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_QUEUE_CONFIG_LOG_LEVEL
#define NRF_QUEUE_CONFIG_LOG_LEVEL 3
#endif

// <o> NRF_QUEUE_CONFIG_LOG_INIT_FILTER_LEVEL  - Initial severity level if dynamic filtering is enabled
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_QUEUE_CONFIG_LOG_INIT_FILTER_LEVEL
#define NRF_QUEUE_CONFIG_LOG_INIT_FILTER_LEVEL 3
#endif

// <o> NRF_QUEUE_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_QUEUE_CONFIG_INFO_COLOR
#define NRF_QUEUE_CONFIG_INFO_COLOR 0
#endif

// <o> NRF_QUEUE_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_QUEUE_CONFIG_DEBUG_COLOR
#define NRF_QUEUE_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> NRF_SDH_ANT_LOG_ENABLED - Enable logging in SoftDevice handler (ANT) module.
//==========================================================
#ifndef NRF_SDH_ANT_LOG_ENABLED
#define NRF_SDH_ANT_LOG_ENABLED 0
#endif
// <o> NRF_SDH_ANT_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_SDH_ANT_LOG_LEVEL
#define NRF_SDH_ANT_LOG_LEVEL 3
#endif

// <o> NRF_SDH_ANT_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_SDH_ANT_INFO_COLOR
#define NRF_SDH_ANT_INFO_COLOR 0
#endif

// <o> NRF_SDH_ANT_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_SDH_ANT_DEBUG_COLOR
#define NRF_SDH_ANT_DEBUG_COLOR 0
#endif

// </e>

// <e> NRF_SDH_BLE_LOG_ENABLED - Enable logging in SoftDevice handler (BLE) module.
//==========================================================
#ifndef NRF_SDH_BLE_LOG_ENABLED
#define NRF_SDH_BLE_LOG_ENABLED 1
#endif
// <o> NRF_SDH_BLE_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_SDH_BLE_LOG_LEVEL
#define NRF_SDH_BLE_LOG_LEVEL 3
#endif

// <o> NRF_SDH_BLE_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_SDH_BLE_INFO_COLOR
#define NRF_SDH_BLE_INFO_COLOR 0
#endif

// <o> NRF_SDH_BLE_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_SDH_BLE_DEBUG_COLOR
#define NRF_SDH_BLE_DEBUG_COLOR 0
#endif

// </e>

// <e> NRF_SDH_LOG_ENABLED - Enable logging in SoftDevice handler module.
//==========================================================
#ifndef NRF_SDH_LOG_ENABLED
#define NRF_SDH_LOG_ENABLED 1
#endif
// <o> NRF_SDH_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_SDH_LOG_LEVEL
#define NRF_SDH_LOG_LEVEL 3
#endif

// <o> NRF_SDH_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_SDH_INFO_COLOR
#define NRF_SDH_INFO_COLOR 0
#endif

// <o> NRF_SDH_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_SDH_DEBUG_COLOR
#define NRF_SDH_DEBUG_COLOR 0
#endif

// </e>

// <e> NRF_SDH_SOC_LOG_ENABLED - Enable logging in SoftDevice handler (SoC) module.
//==========================================================
#ifndef NRF_SDH_SOC_LOG_ENABLED
#define NRF_SDH_SOC_LOG_ENABLED 1
#endif
// <o> NRF_SDH_SOC_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_SDH_SOC_LOG_LEVEL
#define NRF_SDH_SOC_LOG_LEVEL 3
#endif

// <o> NRF_SDH_SOC_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_SDH_SOC_INFO_COLOR
#define NRF_SDH_SOC_INFO_COLOR 0
#endif

// <o> NRF_SDH_SOC_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_SDH_SOC_DEBUG_COLOR
#define NRF_SDH_SOC_DEBUG_COLOR 0
#endif

// </e>

// <e> NRF_SORTLIST_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRF_SORTLIST_CONFIG_LOG_ENABLED
#define NRF_SORTLIST_CONFIG_LOG_ENABLED 0
#endif
// <o> NRF_SORTLIST_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_SORTLIST_CONFIG_LOG_LEVEL
#define NRF_SORTLIST_CONFIG_LOG_LEVEL 3
#endif

// <o> NRF_SORTLIST_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_SORTLIST_CONFIG_INFO_COLOR
#define NRF_SORTLIST_CONFIG_INFO_COLOR 0
#endif

// <o> NRF_SORTLIST_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_SORTLIST_CONFIG_DEBUG_COLOR
#define NRF_SORTLIST_CONFIG_DEBUG_COLOR 0
#endif

// <q> NRF_SECTION_ITER_ENABLED  - nrf_section_iter - Section iterator
 

#ifndef NRF_SECTION_ITER_ENABLED
#define NRF_SECTION_ITER_ENABLED 1
#endif

// </e>

// <e> NRF_TWI_SENSOR_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRF_TWI_SENSOR_CONFIG_LOG_ENABLED
#define NRF_TWI_SENSOR_CONFIG_LOG_ENABLED 0
#endif
// <o> NRF_TWI_SENSOR_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_TWI_SENSOR_CONFIG_LOG_LEVEL
#define NRF_TWI_SENSOR_CONFIG_LOG_LEVEL 3
#endif

// <o> NRF_TWI_SENSOR_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_TWI_SENSOR_CONFIG_INFO_COLOR
#define NRF_TWI_SENSOR_CONFIG_INFO_COLOR 0
#endif

// <o> NRF_TWI_SENSOR_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_TWI_SENSOR_CONFIG_DEBUG_COLOR
#define NRF_TWI_SENSOR_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <e> PM_LOG_ENABLED - Enable logging in Peer Manager and its submodules.
//==========================================================
#ifndef PM_LOG_ENABLED
#define PM_LOG_ENABLED 1
#endif
// <o> PM_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef PM_LOG_LEVEL
#define PM_LOG_LEVEL 3
#endif

// <o> PM_LOG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef PM_LOG_INFO_COLOR
#define PM_LOG_INFO_COLOR 0
#endif

// <o> PM_LOG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef PM_LOG_DEBUG_COLOR
#define PM_LOG_DEBUG_COLOR 0
#endif

// <h> app_button - buttons handling module

//==========================================================
// <q> BUTTON_ENABLED  - Enables Button module
 

#ifndef BUTTON_ENABLED
#define BUTTON_ENABLED 1
#endif

// <q> BUTTON_HIGH_ACCURACY_ENABLED  - Enables GPIOTE high accuracy for buttons
 

#ifndef BUTTON_HIGH_ACCURACY_ENABLED
#define BUTTON_HIGH_ACCURACY_ENABLED 0
#endif

// </e>

// </h> 
//==========================================================

// <h> nrf_log in nRF_Serialization 

//==========================================================
// <e> SER_HAL_TRANSPORT_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef SER_HAL_TRANSPORT_CONFIG_LOG_ENABLED
#define SER_HAL_TRANSPORT_CONFIG_LOG_ENABLED 0
#endif
// <o> SER_HAL_TRANSPORT_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef SER_HAL_TRANSPORT_CONFIG_LOG_LEVEL
#define SER_HAL_TRANSPORT_CONFIG_LOG_LEVEL 3
#endif

// <o> SER_HAL_TRANSPORT_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef SER_HAL_TRANSPORT_CONFIG_INFO_COLOR
#define SER_HAL_TRANSPORT_CONFIG_INFO_COLOR 0
#endif

// <o> SER_HAL_TRANSPORT_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef SER_HAL_TRANSPORT_CONFIG_DEBUG_COLOR
#define SER_HAL_TRANSPORT_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// </h> 
//==========================================================

// </h> 
//==========================================================

// </e>

// <q> NRF_LOG_STR_FORMATTER_TIMESTAMP_FORMAT_ENABLED  - nrf_log_str_formatter - Log string formatter
 

#ifndef NRF_LOG_STR_FORMATTER_TIMESTAMP_FORMAT_ENABLED
#define NRF_LOG_STR_FORMATTER_TIMESTAMP_FORMAT_ENABLED 1
#endif

// </h> 
//==========================================================

// <h> NFC_BLE_PAIR_LIB_SECURITY_PARAMETERS - Common Peer Manager security parameters.

//==========================================================
// <e> BLE_NFC_SEC_PARAM_BOND - Enables device bonding.

// <i> If bonding is enabled at least one of the BLE_NFC_SEC_PARAM_KDIST options must be enabled.
//==========================================================
#ifndef BLE_NFC_SEC_PARAM_BOND
#define BLE_NFC_SEC_PARAM_BOND 1
#endif
// <q> BLE_NFC_SEC_PARAM_KDIST_OWN_ENC  - Enables Long Term Key and Master Identification distribution by device.
 

#ifndef BLE_NFC_SEC_PARAM_KDIST_OWN_ENC
#define BLE_NFC_SEC_PARAM_KDIST_OWN_ENC 1
#endif

// <q> BLE_NFC_SEC_PARAM_KDIST_OWN_ID  - Enables Identity Resolving Key and Identity Address Information distribution by device.
 

#ifndef BLE_NFC_SEC_PARAM_KDIST_OWN_ID
#define BLE_NFC_SEC_PARAM_KDIST_OWN_ID 1
#endif

// <q> BLE_NFC_SEC_PARAM_KDIST_PEER_ENC  - Enables Long Term Key and Master Identification distribution by peer.
 

#ifndef BLE_NFC_SEC_PARAM_KDIST_PEER_ENC
#define BLE_NFC_SEC_PARAM_KDIST_PEER_ENC 1
#endif

// <q> BLE_NFC_SEC_PARAM_KDIST_PEER_ID  - Enables Identity Resolving Key and Identity Address Information distribution by peer.
 

#ifndef BLE_NFC_SEC_PARAM_KDIST_PEER_ID
#define BLE_NFC_SEC_PARAM_KDIST_PEER_ID 1
#endif

// </e>

// <o> BLE_NFC_SEC_PARAM_MIN_KEY_SIZE  - Minimal size of a security key.
 
// <7=> 7 
// <8=> 8 
// <9=> 9 
// <10=> 10 
// <11=> 11 
// <12=> 12 
// <13=> 13 
// <14=> 14 
// <15=> 15 
// <16=> 16 

#ifndef BLE_NFC_SEC_PARAM_MIN_KEY_SIZE
#define BLE_NFC_SEC_PARAM_MIN_KEY_SIZE 7
#endif

// <o> BLE_NFC_SEC_PARAM_MAX_KEY_SIZE  - Maximal size of a security key.
 
// <7=> 7 
// <8=> 8 
// <9=> 9 
// <10=> 10 
// <11=> 11 
// <12=> 12 
// <13=> 13 
// <14=> 14 
// <15=> 15 
// <16=> 16 

#ifndef BLE_NFC_SEC_PARAM_MAX_KEY_SIZE
#define BLE_NFC_SEC_PARAM_MAX_KEY_SIZE 16
#endif

// <<< end of configuration section >>>


// </h> 
//==========================================================

// </h> 
//==========================================================

// <h> nRF_SoftDevice 

//==========================================================
// <e> NRF_SDH_BLE_ENABLED - nrf_sdh_ble - SoftDevice BLE event handler
//==========================================================
#ifndef NRF_SDH_BLE_ENABLED
#define NRF_SDH_BLE_ENABLED 1
#endif
// <h> BLE Stack configuration - Stack configuration parameters

// <i> The SoftDevice handler will configure the stack with these parameters when calling @ref nrf_sdh_ble_default_cfg_set.
// <i> Other libraries might depend on these values; keep them up-to-date even if you are not explicitely calling @ref nrf_sdh_ble_default_cfg_set.
//==========================================================
// <o> NRF_SDH_BLE_GAP_DATA_LENGTH   <27-251> 


// <i> Requested BLE GAP data length to be negotiated.

#ifndef NRF_SDH_BLE_GAP_DATA_LENGTH
#define NRF_SDH_BLE_GAP_DATA_LENGTH 251
#endif

// <o> NRF_SDH_BLE_PERIPHERAL_LINK_COUNT - Maximum number of peripheral links. 
#ifndef NRF_SDH_BLE_PERIPHERAL_LINK_COUNT
#define NRF_SDH_BLE_PERIPHERAL_LINK_COUNT 1
#endif

// <o> NRF_SDH_BLE_CENTRAL_LINK_COUNT - Maximum number of central links. 
#ifndef NRF_SDH_BLE_CENTRAL_LINK_COUNT
#define NRF_SDH_BLE_CENTRAL_LINK_COUNT 0
#endif

// <o> NRF_SDH_BLE_TOTAL_LINK_COUNT - Total link count. 
// <i> Maximum number of total concurrent connections using the default configuration.

#ifndef NRF_SDH_BLE_TOTAL_LINK_COUNT
#define NRF_SDH_BLE_TOTAL_LINK_COUNT 1
#endif

// <o> NRF_SDH_BLE_GAP_EVENT_LENGTH - GAP event length. 
// <i> The time set aside for this connection on every connection interval in 1.25 ms units.

#ifndef NRF_SDH_BLE_GAP_EVENT_LENGTH
#define NRF_SDH_BLE_GAP_EVENT_LENGTH 12
#endif

// <o> NRF_SDH_BLE_GATT_MAX_MTU_SIZE - Static maximum MTU size. 
#ifndef NRF_SDH_BLE_GATT_MAX_MTU_SIZE
#define NRF_SDH_BLE_GATT_MAX_MTU_SIZE 247
#define NRF_SDH_BLE_GATT_MIN_MTU_SIZE 27
#endif

// <o> NRF_SDH_BLE_GATTS_ATTR_TAB_SIZE - Attribute Table size in bytes. The size must be a multiple of 4. 
#ifndef NRF_SDH_BLE_GATTS_ATTR_TAB_SIZE
#define NRF_SDH_BLE_GATTS_ATTR_TAB_SIZE 1408
#endif

// <o> NRF_SDH_BLE_VS_UUID_COUNT - The number of vendor-specific UUIDs. 
#ifndef NRF_SDH_BLE_VS_UUID_COUNT
#define NRF_SDH_BLE_VS_UUID_COUNT 3
#endif

// <q> NRF_SDH_BLE_SERVICE_CHANGED  - Include the Service Changed characteristic in the Attribute Table.
 

#ifndef NRF_SDH_BLE_SERVICE_CHANGED
#define NRF_SDH_BLE_SERVICE_CHANGED 0
#endif

// </h> 
//==========================================================

// <h> BLE Observers - Observers and priority levels

//==========================================================
// <o> NRF_SDH_BLE_OBSERVER_PRIO_LEVELS - Total number of priority levels for BLE observers. 
// <i> This setting configures the number of priority levels available for BLE event handlers.
// <i> The priority level of a handler determines the order in which it receives events, with respect to other handlers.

#ifndef NRF_SDH_BLE_OBSERVER_PRIO_LEVELS
#define NRF_SDH_BLE_OBSERVER_PRIO_LEVELS 4
#endif

// <h> BLE Observers priorities - Invididual priorities

//==========================================================
// <o> BLE_ADV_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the Advertising module.

#ifndef BLE_ADV_BLE_OBSERVER_PRIO
#define BLE_ADV_BLE_OBSERVER_PRIO 1
#endif

// <o> BLE_ANCS_C_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the Apple Notification Service Client.

#ifndef BLE_ANCS_C_BLE_OBSERVER_PRIO
#define BLE_ANCS_C_BLE_OBSERVER_PRIO 2
#endif

// <o> BLE_ANS_C_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the Alert Notification Service Client.

#ifndef BLE_ANS_C_BLE_OBSERVER_PRIO
#define BLE_ANS_C_BLE_OBSERVER_PRIO 2
#endif

// <o> BLE_BAS_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the Battery Service.

#ifndef BLE_BAS_BLE_OBSERVER_PRIO
#define BLE_BAS_BLE_OBSERVER_PRIO 2
#endif

#ifndef BLE_beep_BLE_OBSERVER_PRIO
#define BLE_beep_BLE_OBSERVER_PRIO 2
#endif

// <o> BLE_BAS_C_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the Battery Service Client.

#ifndef BLE_BAS_C_BLE_OBSERVER_PRIO
#define BLE_BAS_C_BLE_OBSERVER_PRIO 2
#endif

// <o> BLE_BPS_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the Blood Pressure Service.

#ifndef BLE_BPS_BLE_OBSERVER_PRIO
#define BLE_BPS_BLE_OBSERVER_PRIO 2
#endif

// <o> BLE_CONN_PARAMS_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the Connection parameters module.

#ifndef BLE_CONN_PARAMS_BLE_OBSERVER_PRIO
#define BLE_CONN_PARAMS_BLE_OBSERVER_PRIO 1
#endif

// <o> BLE_CONN_STATE_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the Connection State module.

#ifndef BLE_CONN_STATE_BLE_OBSERVER_PRIO
#define BLE_CONN_STATE_BLE_OBSERVER_PRIO 0
#endif

// <o> BLE_CSCS_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the Cycling Speed and Cadence Service.

#ifndef BLE_CSCS_BLE_OBSERVER_PRIO
#define BLE_CSCS_BLE_OBSERVER_PRIO 2
#endif

// <o> BLE_CTS_C_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the Current Time Service Client.

#ifndef BLE_CTS_C_BLE_OBSERVER_PRIO
#define BLE_CTS_C_BLE_OBSERVER_PRIO 2
#endif

// <o> BLE_DB_DISC_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the Database Discovery module.

#ifndef BLE_DB_DISC_BLE_OBSERVER_PRIO
#define BLE_DB_DISC_BLE_OBSERVER_PRIO 1
#endif

// <o> BLE_DFU_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the DFU Service.

#ifndef BLE_DFU_BLE_OBSERVER_PRIO
#define BLE_DFU_BLE_OBSERVER_PRIO 2
#endif

// <o> BLE_DIS_C_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the Device Information Client.

#ifndef BLE_DIS_C_BLE_OBSERVER_PRIO
#define BLE_DIS_C_BLE_OBSERVER_PRIO 2
#endif

// <o> BLE_GLS_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the Glucose Service.

#ifndef BLE_GLS_BLE_OBSERVER_PRIO
#define BLE_GLS_BLE_OBSERVER_PRIO 2
#endif

// <o> BLE_HIDS_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the Human Interface Device Service.

#ifndef BLE_HIDS_BLE_OBSERVER_PRIO
#define BLE_HIDS_BLE_OBSERVER_PRIO 2
#endif

// <o> BLE_HRS_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the Heart Rate Service.

#ifndef BLE_HRS_BLE_OBSERVER_PRIO
#define BLE_HRS_BLE_OBSERVER_PRIO 2
#endif

// <o> BLE_HRS_C_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the Heart Rate Service Client.

#ifndef BLE_HRS_C_BLE_OBSERVER_PRIO
#define BLE_HRS_C_BLE_OBSERVER_PRIO 2
#endif

// <o> BLE_HTS_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the Health Thermometer Service.

#ifndef BLE_HTS_BLE_OBSERVER_PRIO
#define BLE_HTS_BLE_OBSERVER_PRIO 2
#endif

// <o> BLE_IAS_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the Immediate Alert Service.

#ifndef BLE_IAS_BLE_OBSERVER_PRIO
#define BLE_IAS_BLE_OBSERVER_PRIO 2
#endif

// <o> BLE_IAS_C_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the Immediate Alert Service Client.

#ifndef BLE_IAS_C_BLE_OBSERVER_PRIO
#define BLE_IAS_C_BLE_OBSERVER_PRIO 2
#endif

// <o> BLE_LBS_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the LED Button Service.

#ifndef BLE_LBS_BLE_OBSERVER_PRIO
#define BLE_LBS_BLE_OBSERVER_PRIO 2
#endif

// <o> BLE_LBS_C_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the LED Button Service Client.

#ifndef BLE_LBS_C_BLE_OBSERVER_PRIO
#define BLE_LBS_C_BLE_OBSERVER_PRIO 2
#endif

// <o> BLE_LLS_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the Link Loss Service.

#ifndef BLE_LLS_BLE_OBSERVER_PRIO
#define BLE_LLS_BLE_OBSERVER_PRIO 2
#endif

// <o> BLE_LNS_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the Location Navigation Service.

#ifndef BLE_LNS_BLE_OBSERVER_PRIO
#define BLE_LNS_BLE_OBSERVER_PRIO 2
#endif

// <o> BLE_NUS_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the UART Service.

#ifndef BLE_NUS_BLE_OBSERVER_PRIO
#define BLE_NUS_BLE_OBSERVER_PRIO 2
#endif

#ifndef BLE_BUS_BLE_OBSERVER_PRIO
#define BLE_BUS_BLE_OBSERVER_PRIO 2
#endif

// <o> BLE_NUS_C_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the UART Central Service.

#ifndef BLE_NUS_C_BLE_OBSERVER_PRIO
#define BLE_NUS_C_BLE_OBSERVER_PRIO 2
#endif

// <o> BLE_OTS_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the Object transfer service.

#ifndef BLE_OTS_BLE_OBSERVER_PRIO
#define BLE_OTS_BLE_OBSERVER_PRIO 2
#endif

// <o> BLE_OTS_C_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the Object transfer service client.

#ifndef BLE_OTS_C_BLE_OBSERVER_PRIO
#define BLE_OTS_C_BLE_OBSERVER_PRIO 2
#endif

// <o> BLE_RSCS_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the Running Speed and Cadence Service.

#ifndef BLE_RSCS_BLE_OBSERVER_PRIO
#define BLE_RSCS_BLE_OBSERVER_PRIO 2
#endif

// <o> BLE_RSCS_C_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the Running Speed and Cadence Client.

#ifndef BLE_RSCS_C_BLE_OBSERVER_PRIO
#define BLE_RSCS_C_BLE_OBSERVER_PRIO 2
#endif

// <o> BLE_TPS_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the TX Power Service.

#ifndef BLE_TPS_BLE_OBSERVER_PRIO
#define BLE_TPS_BLE_OBSERVER_PRIO 2
#endif

// <o> BSP_BTN_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the Button Control module.

#ifndef BSP_BTN_BLE_OBSERVER_PRIO
#define BSP_BTN_BLE_OBSERVER_PRIO 1
#endif

// <o> NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the NFC pairing library.

#ifndef NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO
#define NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO 1
#endif

// <o> NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the NFC pairing library.

#ifndef NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO
#define NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO 1
#endif

// <o> NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the NFC pairing library.

#ifndef NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO
#define NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO 1
#endif

// <o> NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the NFC pairing library.

#ifndef NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO
#define NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO 1
#endif

// <o> NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the NFC pairing library.

#ifndef NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO
#define NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO 1
#endif

// <o> NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the NFC pairing library.

#ifndef NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO
#define NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO 1
#endif

// <o> NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the NFC pairing library.

#ifndef NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO
#define NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO 1
#endif

// <o> NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the NFC pairing library.

#ifndef NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO
#define NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO 1
#endif

// <o> NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the NFC pairing library.

#ifndef NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO
#define NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO 1
#endif

// <o> NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the NFC pairing library.

#ifndef NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO
#define NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO 1
#endif

// <o> NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the NFC pairing library.

#ifndef NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO
#define NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO 1
#endif

// <o> NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the NFC pairing library.

#ifndef NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO
#define NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO 1
#endif

// <o> NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the NFC pairing library.

#ifndef NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO
#define NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO 1
#endif

// <o> NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the NFC pairing library.

#ifndef NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO
#define NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO 1
#endif

// <o> NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the NFC pairing library.

#ifndef NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO
#define NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO 1
#endif

// <o> NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the NFC pairing library.

#ifndef NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO
#define NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO 1
#endif

// <o> NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the NFC pairing library.

#ifndef NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO
#define NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO 1
#endif

// <o> NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the NFC pairing library.

#ifndef NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO
#define NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO 1
#endif

// <o> NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the NFC pairing library.

#ifndef NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO
#define NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO 1
#endif

// <o> NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the NFC pairing library.

#ifndef NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO
#define NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO 1
#endif

// <o> NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the NFC pairing library.

#ifndef NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO
#define NFC_BLE_PAIR_LIB_BLE_OBSERVER_PRIO 1
#endif

// <o> NRF_BLE_BMS_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the Bond Management Service.

#ifndef NRF_BLE_BMS_BLE_OBSERVER_PRIO
#define NRF_BLE_BMS_BLE_OBSERVER_PRIO 2
#endif

// <o> NRF_BLE_CGMS_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the Contiuon Glucose Monitoring Service.

#ifndef NRF_BLE_CGMS_BLE_OBSERVER_PRIO
#define NRF_BLE_CGMS_BLE_OBSERVER_PRIO 2
#endif

// <o> NRF_BLE_ES_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the Eddystone module.

#ifndef NRF_BLE_ES_BLE_OBSERVER_PRIO
#define NRF_BLE_ES_BLE_OBSERVER_PRIO 2
#endif

// <o> NRF_BLE_GATTS_C_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the GATT Service Client.

#ifndef NRF_BLE_GATTS_C_BLE_OBSERVER_PRIO
#define NRF_BLE_GATTS_C_BLE_OBSERVER_PRIO 2
#endif

// <o> NRF_BLE_GATT_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the GATT module.

#ifndef NRF_BLE_GATT_BLE_OBSERVER_PRIO
#define NRF_BLE_GATT_BLE_OBSERVER_PRIO 1
#endif

// <o> NRF_BLE_QWR_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the Queued writes module.

#ifndef NRF_BLE_QWR_BLE_OBSERVER_PRIO
#define NRF_BLE_QWR_BLE_OBSERVER_PRIO 2
#endif

// <o> NRF_BLE_SCAN_OBSERVER_PRIO  
// <i> Priority for dispatching the BLE events to the Scanning Module.

#ifndef NRF_BLE_SCAN_OBSERVER_PRIO
#define NRF_BLE_SCAN_OBSERVER_PRIO 1
#endif

// <o> PM_BLE_OBSERVER_PRIO - Priority with which BLE events are dispatched to the Peer Manager module. 
#ifndef PM_BLE_OBSERVER_PRIO
#define PM_BLE_OBSERVER_PRIO 1
#endif

// </h> 
//==========================================================

// </h> 
//==========================================================


// </e>

// <e> NRF_SDH_ENABLED - nrf_sdh - SoftDevice handler
//==========================================================
#ifndef NRF_SDH_ENABLED
#define NRF_SDH_ENABLED 1
#endif
// <h> Dispatch model 

// <i> This setting configures how Stack events are dispatched to the application.
//==========================================================
// <o> NRF_SDH_DISPATCH_MODEL
 

// <i> NRF_SDH_DISPATCH_MODEL_INTERRUPT: SoftDevice events are passed to the application from the interrupt context.
// <i> NRF_SDH_DISPATCH_MODEL_APPSH: SoftDevice events are scheduled using @ref app_scheduler.
// <i> NRF_SDH_DISPATCH_MODEL_POLLING: SoftDevice events are to be fetched manually.
// <0=> NRF_SDH_DISPATCH_MODEL_INTERRUPT 
// <1=> NRF_SDH_DISPATCH_MODEL_APPSH 
// <2=> NRF_SDH_DISPATCH_MODEL_POLLING 

#ifndef NRF_SDH_DISPATCH_MODEL
#define NRF_SDH_DISPATCH_MODEL 0
#endif

// </h> 
//==========================================================

// <h> Clock - SoftDevice clock configuration

//==========================================================
// <o> NRF_SDH_CLOCK_LF_SRC  - SoftDevice clock source.
 
// <0=> NRF_CLOCK_LF_SRC_RC 
// <1=> NRF_CLOCK_LF_SRC_XTAL 
// <2=> NRF_CLOCK_LF_SRC_SYNTH 

#ifndef NRF_SDH_CLOCK_LF_SRC
#define NRF_SDH_CLOCK_LF_SRC 1
#endif

// <o> NRF_SDH_CLOCK_LF_RC_CTIV - SoftDevice calibration timer interval. 
#ifndef NRF_SDH_CLOCK_LF_RC_CTIV
#define NRF_SDH_CLOCK_LF_RC_CTIV 0
#endif

// <o> NRF_SDH_CLOCK_LF_RC_TEMP_CTIV - SoftDevice calibration timer interval under constant temperature. 
// <i> How often (in number of calibration intervals) the RC oscillator shall be calibrated
// <i>  if the temperature has not changed.

#ifndef NRF_SDH_CLOCK_LF_RC_TEMP_CTIV
#define NRF_SDH_CLOCK_LF_RC_TEMP_CTIV 0
#endif

// <o> NRF_SDH_CLOCK_LF_ACCURACY  - External clock accuracy used in the LL to compute timing.
 
// <0=> NRF_CLOCK_LF_ACCURACY_250_PPM 
// <1=> NRF_CLOCK_LF_ACCURACY_500_PPM 
// <2=> NRF_CLOCK_LF_ACCURACY_150_PPM 
// <3=> NRF_CLOCK_LF_ACCURACY_100_PPM 
// <4=> NRF_CLOCK_LF_ACCURACY_75_PPM 
// <5=> NRF_CLOCK_LF_ACCURACY_50_PPM 
// <6=> NRF_CLOCK_LF_ACCURACY_30_PPM 
// <7=> NRF_CLOCK_LF_ACCURACY_20_PPM 
// <8=> NRF_CLOCK_LF_ACCURACY_10_PPM 
// <9=> NRF_CLOCK_LF_ACCURACY_5_PPM 
// <10=> NRF_CLOCK_LF_ACCURACY_2_PPM 
// <11=> NRF_CLOCK_LF_ACCURACY_1_PPM 

#ifndef NRF_SDH_CLOCK_LF_ACCURACY
#define NRF_SDH_CLOCK_LF_ACCURACY 7
#endif

// </h> 
//==========================================================

// <h> SDH Observers - Observers and priority levels

//==========================================================
// <o> NRF_SDH_REQ_OBSERVER_PRIO_LEVELS - Total number of priority levels for request observers. 
// <i> This setting configures the number of priority levels available for the SoftDevice request event handlers.
// <i> The priority level of a handler determines the order in which it receives events, with respect to other handlers.

#ifndef NRF_SDH_REQ_OBSERVER_PRIO_LEVELS
#define NRF_SDH_REQ_OBSERVER_PRIO_LEVELS 2
#endif

// <o> NRF_SDH_STATE_OBSERVER_PRIO_LEVELS - Total number of priority levels for state observers. 
// <i> This setting configures the number of priority levels available for the SoftDevice state event handlers.
// <i> The priority level of a handler determines the order in which it receives events, with respect to other handlers.

#ifndef NRF_SDH_STATE_OBSERVER_PRIO_LEVELS
#define NRF_SDH_STATE_OBSERVER_PRIO_LEVELS 2
#endif

// <o> NRF_SDH_STACK_OBSERVER_PRIO_LEVELS - Total number of priority levels for stack event observers. 
// <i> This setting configures the number of priority levels available for the SoftDevice stack event handlers (ANT, BLE, SoC).
// <i> The priority level of a handler determines the order in which it receives events, with respect to other handlers.

#ifndef NRF_SDH_STACK_OBSERVER_PRIO_LEVELS
#define NRF_SDH_STACK_OBSERVER_PRIO_LEVELS 2
#endif


// <h> State Observers priorities - Invididual priorities

//==========================================================
// <o> CLOCK_CONFIG_STATE_OBSERVER_PRIO  
// <i> Priority with which state events are dispatched to the Clock driver.

#ifndef CLOCK_CONFIG_STATE_OBSERVER_PRIO
#define CLOCK_CONFIG_STATE_OBSERVER_PRIO 0
#endif

// <o> POWER_CONFIG_STATE_OBSERVER_PRIO  
// <i> Priority with which state events are dispatched to the Power driver.

#ifndef POWER_CONFIG_STATE_OBSERVER_PRIO
#define POWER_CONFIG_STATE_OBSERVER_PRIO 0
#endif

// <o> RNG_CONFIG_STATE_OBSERVER_PRIO  
// <i> Priority with which state events are dispatched to this module.

#ifndef RNG_CONFIG_STATE_OBSERVER_PRIO
#define RNG_CONFIG_STATE_OBSERVER_PRIO 0
#endif


// </h> 
//==========================================================

// <h> Stack Event Observers priorities - Invididual priorities

//==========================================================
// <o> NRF_SDH_ANT_STACK_OBSERVER_PRIO  
// <i> This setting configures the priority with which ANT events are processed with respect to other events coming from the stack.
// <i> Modify this setting if you need to have ANT events dispatched before or after other stack events, such as BLE or SoC.
// <i> Zero is the highest priority.

#ifndef NRF_SDH_ANT_STACK_OBSERVER_PRIO
#define NRF_SDH_ANT_STACK_OBSERVER_PRIO 0
#endif

// <o> NRF_SDH_BLE_STACK_OBSERVER_PRIO  
// <i> This setting configures the priority with which BLE events are processed with respect to other events coming from the stack.
// <i> Modify this setting if you need to have BLE events dispatched before or after other stack events, such as ANT or SoC.
// <i> Zero is the highest priority.

#ifndef NRF_SDH_BLE_STACK_OBSERVER_PRIO
#define NRF_SDH_BLE_STACK_OBSERVER_PRIO 0
#endif

// <o> NRF_SDH_SOC_STACK_OBSERVER_PRIO  
// <i> This setting configures the priority with which SoC events are processed with respect to other events coming from the stack.
// <i> Modify this setting if you need to have SoC events dispatched before or after other stack events, such as ANT or BLE.
// <i> Zero is the highest priority.

#ifndef NRF_SDH_SOC_STACK_OBSERVER_PRIO
#define NRF_SDH_SOC_STACK_OBSERVER_PRIO 0
#endif

// </h> 
//==========================================================

// </h> 
//==========================================================


// </e>

// <e> NRF_SDH_SOC_ENABLED - nrf_sdh_soc - SoftDevice SoC event handler
//==========================================================
#ifndef NRF_SDH_SOC_ENABLED
#define NRF_SDH_SOC_ENABLED 1
#endif
// <h> SoC Observers - Observers and priority levels

//==========================================================
// <o> NRF_SDH_SOC_OBSERVER_PRIO_LEVELS - Total number of priority levels for SoC observers. 
// <i> This setting configures the number of priority levels available for the SoC event handlers.
// <i> The priority level of a handler determines the order in which it receives events, with respect to other handlers.

#ifndef NRF_SDH_SOC_OBSERVER_PRIO_LEVELS
#define NRF_SDH_SOC_OBSERVER_PRIO_LEVELS 2
#endif

// <h> SoC Observers priorities - Invididual priorities

//==========================================================
// <o> BLE_ADV_SOC_OBSERVER_PRIO  
// <i> Priority with which SoC events are dispatched to the Advertising module.

#ifndef BLE_ADV_SOC_OBSERVER_PRIO
#define BLE_ADV_SOC_OBSERVER_PRIO 1
#endif

// <o> BLE_DFU_SOC_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the DFU Service.

#ifndef BLE_DFU_SOC_OBSERVER_PRIO
#define BLE_DFU_SOC_OBSERVER_PRIO 1
#endif

// <o> CLOCK_CONFIG_SOC_OBSERVER_PRIO  
// <i> Priority with which SoC events are dispatched to the Clock driver.

#ifndef CLOCK_CONFIG_SOC_OBSERVER_PRIO
#define CLOCK_CONFIG_SOC_OBSERVER_PRIO 0
#endif

// <o> POWER_CONFIG_SOC_OBSERVER_PRIO  
// <i> Priority with which SoC events are dispatched to the Power driver.

#ifndef POWER_CONFIG_SOC_OBSERVER_PRIO
#define POWER_CONFIG_SOC_OBSERVER_PRIO 0
#endif

// </h> 
//==========================================================

// </h> 
//==========================================================


	   

		
															

									   
#endif //SDK_CONFIG_H

