/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
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
/** @file
 * @defgroup nrf_dev_timer_example_main main.c
 * @{
 * @ingroup nrf_dev_timer_example
 * @brief Timer Example Application main file.
 *
 * This file contains the source code for a sample application using Timer0.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "main.h"
#include "nrf.h"
#include "bsp.h"
#include "app_error.h"
#include "LoRaWAN_imp.h"
#include "SQ.h"
#define NRF_LOG_MODULE_NAME Main
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "app_timer.h"
#include "nvm_fs.h"
#include "utilities.h"
#include "board.h"
#include "gpio-board.h"
#include "HX711.h"
#include "DS18B20_app.h"
#include "nRF_ADC.h"
#include "Buzzer.h"
#include "pwm.h"
#include "BLE_setup.h"
#include "Commissioning.h"
#include "power_app.h"
#include "I2C.h"
#include "TLV320ADC3100.h"
#include "beep_protocol.h"
#include "HX711_app.h"
#include "nrf_delay.h"
#include "nrf_pwr_mgmt.h"
#include "timestamp_timer.h"
#include "nrf_sdh.h"
#include "nrf_bootloader_info.h"
#include "nrf_power.h"
#include "atecc608A.h"
#include "mx_flash_app.h"
#include "mx_flash_fs.h"
#include "nrf_ble_lesc.h"
#include "audioFFT.h"
#include "audio_app.h"
#include "alarm_app.h"
#include "bme_app.h"

APP_TIMER_DEF(sampleTimer);
APP_TIMER_DEF(reedswitchtimer);

MAIN_APP_s main_app = 
{
	.sampleInterval_min	= 0,
	.new_Timer_config	= false,

    #if joinIndicationSend
        .joinIndicationSend = false,
    #else
        .joinIndicationSend = true,
    #endif
};

uint32_t err_code;
static volatile bool enterDFU = false;
static uint8_t reedSwitchTicks = 0;

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Handler for shutdown preparation.
 *
 * @details During shutdown procedures, this function will be called at a 1 second interval
 *          untill the function returns true. When the function returns true, it means that the
 *          app is ready to reset to DFU mode.
 *
 * @param[in]   event   Power manager event.
 *
 * @retval  True if shutdown is allowed by this power manager handler, otherwise false.
 */
bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_DFU:
            NRF_LOG_INFO("Power management wants to reset to DFU mode.");

			switch(main_app.current_state)
			{
				case VERTICAL_POWER_DOWN:	// In powerdown the application may enter DFU
				case HORIZONTAL_DFU_READY:
					NRF_LOG_INFO("Power management allowed to reset to DFU mode.");
					return true;
					break;
				default:
					// DFU should wait until the main application state is ready.
                    NRF_LOG_INFO("Power management not ready to reset to DFU mode.");
					enterDFU = true;	
					return false;
					break;
			}
            break;

        default:
            return true;
    }

    
    return true;
}
NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);

static void buttonless_dfu_sdh_state_observer(nrf_sdh_state_evt_t state, void * p_context)
{
    if (state == NRF_SDH_EVT_STATE_DISABLED)
    {
        // Softdevice was disabled before going into reset. Inform bootloader to skip CRC on next boot.
        nrf_power_gpregret2_set(BOOTLOADER_DFU_SKIP_CRC);

        //Go to system off.
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
    }
}

/* nrf_sdh state observer. */
NRF_SDH_STATE_OBSERVER(m_buttonless_dfu_state_obs, 0) =
{
    .handler = buttonless_dfu_sdh_state_observer,
};


static void reedSwitchTimerEnable(bool enable)
{
    uint32_t ret;
    reedSwitchTicks = 0;
    if(enable)
    {
        ret = app_timer_start(reedswitchtimer, APP_TIMER_TICKS(1000), NULL);
    }
    else
    {
        ret = app_timer_stop(reedswitchtimer);
    }

}

static void reedSwitchCallback(void * p_context)
{
    uint32_t ret;

    // Check the pin state, when high increment. If low clear the timer.
    if(!nrf_gpio_pin_read(BUTTON_1))
    {
        reedSwitchTicks++;
        NRF_LOG_INFO("Reedswitch tick count: %u", reedSwitchTicks);
        if(reedSwitchTicks >= 30)
        {
            reedSwitchTimerEnable(false);
            Buzzer_default_tune(1);

            // Reset the pincode to default
            uint8_t pincode[] = {PIN_CODE_DEFAULT};
            BEEP_protocol_s prot;
            memset(&prot, 0, sizeof(BEEP_protocol_s));
            prot.command = WRITE_PINCODE;
            prot.param.lorawan_key.lenght = PIN_CODE_BLE_LENGHT;
            memcpy(&prot.param.lorawan_key.data, pincode, PIN_CODE_BLE_LENGHT);

            ret = nvm_fds_eeprom_set(&prot);
            if(ret == NRF_SUCCESS)
            {
                NRF_LOG_INFO("Pin code Reset");
                load_passKey();
                delete_bonds();
            }
        }
    }
    else
    {
        NRF_LOG_INFO("Reedswitch STOP tick count: %u", reedSwitchTicks);
        reedSwitchTimerEnable(false);
    }
}

static void reedswitchTimerInit(void)
{
    uint32_t ret;
    ret = app_timer_create(&reedswitchtimer, APP_TIMER_MODE_REPEATED, reedSwitchCallback);
    APP_ERROR_CHECK(ret);
}


/**@brief Function for handling bsp events.
 */
void bsp_evt_handler(bsp_event_t evt)
{
	static uint32_t bsp_count = 0;
    uint32_t ret;
    switch (evt)
    {
        case BSP_EVENT_KEY_0:
			NRF_LOG_INFO("%u - Reed switch BSP event", bsp_count++);

			if(main_app.current_state == HORIZONTAL_ACTIVE)
			{
				ret = advertising_start(false);
                reedSwitchTimerEnable(true);
                Buzzer_default_tune(2);
			}
			else if(main_app.current_state == HORIZONTAL_IDLE)
			{
                reedSwitchTimerEnable(true);
				ret = advertising_start(false);
			}

            #if BLE_ADV_START_STOP
                // Disable advertising if already on
                if(ret != NRF_SUCCESS)
                {
                    advertising_stop();
                }
            #endif
            break;

        default:
            return; // no implementation needed
    }
}


/**@Brief Measurement result handler from all the different sensors which connect the results to the Beep service where the data is send to the user.
 */
void measurement_handler(MEASUREMENT_RESULT_s * meas)
{
	uint32_t retval;
	retval = ble_measurement_send(meas);

	#if 0
		const char * source[] = {"INTERNAL", "BLE", "LORAWAN", "UNKNOWN"};
		NRF_LOG_INFO("Measurement send: 0x%04X, source: %s", retval, source[((meas->source >= UNKNOWN_SOURCE) ? UNKNOWN_SOURCE : meas->source)]);
	#endif

    // Check the new sample whether the new measurement result is cause for an alarm status
    checkSample(meas);

	// Check the source of the measurement, whether it was started from a command from BLE or LoRaWAN.
	if(meas->source == INTERNAL_SOURCE || meas->source >= UNKNOWN_SOURCE)
	{
		return;
	}

    BEEP_protocol_s prot;
	memset(&prot, 0, sizeof(prot));

	switch(meas->type)
	{

		///------------------------------------------------------------------------------------------------------------------------------------------------------------------------
		case DS18B20:
		{
			memcpy(&prot.param.meas_result.result.ds18B20, &meas->result.ds18B20, sizeof(DS18B20_RESULTS_s));
			prot.param.meas_result.type = DS18B20;

			// Send the conversion result 
			prot.command = (meas->result.ds18B20.devices == 0xFF ? WRITE_DS18B20_CONVERSION : READ_DS18B20_CONVERSION);
			break;
		}

		///------------------------------------------------------------------------------------------------------------------------------------------------------------------------
		case HX711:
		{
			HX711_CONV_s * hx = &meas->result.hx711;
			memcpy(&prot.param.meas_result.result.hx711, &meas->result.hx711, sizeof(HX711_CONV_s));
			prot.param.meas_result.type = HX711;
			prot.command				= WRITE_HX711_CONVERSION;
			NRF_LOG_INFO("HX711 main cb channel: %u, samples: %u, average: %i", hx->channel, hx->samples, hx->value);
			break;
		}

        ///------------------------------------------------------------------------------------------------------------------------------------------------------------------------
		case AUDIO_ADC:
		{
			FFT_RESULTS * fft = &meas->result.fft;
			memcpy(&prot.param.meas_result.result.fft, fft, sizeof(FFT_RESULTS));
			prot.param.meas_result.type = AUDIO_ADC;
			prot.command				= READ_AUDIO_ADC_CONVERSION;
			break;
		}

        ///------------------------------------------------------------------------------------------------------------------------------------------------------------------------
		case nRF_ADC:
		{
			ADC_s * adc = &meas->result.saadc;
			memcpy(&prot.param.meas_result.result.saadc, adc, sizeof(ADC_s));
			prot.param.meas_result.type = nRF_ADC;
			prot.command				= WRITE_nRF_ADC_CONVERSION;
			break;
		}

        ///------------------------------------------------------------------------------------------------------------------------------------------------------------------------
		case BME280:
		{
			BME280_RESULT_s * bme = &meas->result.bme280;
			memcpy(&prot.param.meas_result.result.bme280, bme, sizeof(BME280_RESULT_s));
			prot.param.meas_result.type = BME280;
			prot.command				= BME280_CONVERSION_READ;
			break;
		}

		///------------------------------------------------------------------------------------------------------------------------------------------------------------------------
		default:
			return;
			break;
	}

    sendProtocolField(&prot, meas->source);
}


/*
 */
void sendProtocolField(BEEP_protocol_s * prot, CONTROL_SOURCE source)
{
	if(source == BLE_SOURCE)
	{
		ble_controlPoint_send(prot);
	}
	// LORAWAN_SOURCE
	else
	{
		lorawanDownlinkAppend(prot);
	}
}

/*
 */
void sendResponse(CONTROL_SOURCE source, BEEP_CID cmd, uint32_t error_code)
{
	if(source == BLE_SOURCE)
	{
		ble_controlPoint_response(cmd, error_code);
	}
	else
	{
		lorawanDownlinkResponseAppend(cmd, error_code);
	}
}


/**@brief Function for handling Beep Control point events
 */
void beep_ctrlpt_event_handler(CONTROL_SOURCE source, BEEP_protocol_s * prot)
{
	uint32_t ret;
	BEEP_protocol_s reply;
	memset(&reply, 0, sizeof(BEEP_protocol_s));

    reply.command = prot->command;

	switch(prot->command)
	{
		//-----------------------------------------------------------------------------
		case RESPONSE:
			break;

		//-----------------------------------------------------------------------------
		case READ_FIRMWARE_VERSION:
        case READ_ATECC_READ_ID:
        case READ_HARDWARE_VERSION:
		case READ_HX711_STATE:
			nvm_fds_eeprom_get(&reply);
			break;

        //-----------------------------------------------------------------------------
        case WRITE_DS18B20_STATE:
			ret = DS18B20_App_setStatus(prot->param.status.statusflag, prot->param.status.interval);
			sendResponse(source, prot->command, ret);
			return;
			break;

		//-----------------------------------------------------------------------------
		case READ_DS18B20_STATE:
			DS18B20_App_getStatus(&reply.param.status.statusflag, &reply.param.status.interval);
			break;

		//-----------------------------------------------------------------------------
        case READ_DS18B20_CONVERSION:
			DS18B20_App_getTemp(&reply.param.meas_result);
			break;

        //-----------------------------------------------------------------------------
		case WRITE_DS18B20_CONVERSION:
			ret = DS18B20_App_start_conversion(source, prot->param.meas_result.result.ds18B20.devices);

            // Do not send a response for LoRaWAN when ret == NRF_SUCCESS
			if(source != LORAWAN_SOURCE || (source == LORAWAN_SOURCE && ret != NRF_SUCCESS))
			{
				sendResponse(source, prot->command, ret);
			}
			return;
			break;

        //-----------------------------------------------------------------------------
		case READ_HX711_CONVERSION:
			HX711_app_getLastResult(&reply.param.meas_result);
			break;

        //-----------------------------------------------------------------------------
		case WRITE_HX711_CONVERSION:
		{
			HX711_CONV_s * hx = &prot->param.meas_result.result.hx711;
			ret = HX711_app_start_sampling(source, hx->samples, hx->channel);

			// Do not send a response for LoRaWAN when ret == NRF_SUCCESS
			if(source != LORAWAN_SOURCE || (source == LORAWAN_SOURCE && ret != NRF_SUCCESS))
			{
				sendResponse(source, prot->command, ret);
			}
			return;
			break;
		}

        //-----------------------------------------------------------------------------
		case READ_LORAWAN_STATE:
			reply.param.status.statusflag = Lorawan_get_status();
			break;

		//-----------------------------------------------------------------------------
		case WRITE_LORAWAN_STATE:
			lorawan_set_status(prot->param.status.statusflag);
            sendResponse(source, prot->command, NRF_SUCCESS);
			return;
			break;

		//-----------------------------------------------------------------------------
		case WRITE_LORAWAN_TRANSMIT:
		{
			uint32_t ret;
			ret = BEEP_Send(BEEP_BLE_CUSTOM, prot->param.lorawan_key.data, prot->param.lorawan_key.lenght);
			if(source == BLE_SOURCE)
			{
				sendResponse(source, prot->command, ret);
			}
			NRF_LOG_INFO("LoRaWAN custom transmit ret: %u/0x%04x", ret, ret);
			return;
			break;	
		}

        //-----------------------------------------------------------------------------
        case WRITE_LORAWAN_DEVEUI:
        case WRITE_LORAWAN_APPEUI:
        case WRITE_LORAWAN_APPKEY:
			nvm_fds_eeprom_set(prot);
			sendResponse(source, prot->command, NRF_SUCCESS);

			// Update the advertising name when a new DEVEUI is written.
			if(prot->command == WRITE_LORAWAN_DEVEUI)
			{
				ble_updateAdvertisingData();
			}
			return;
			break;

		//-----------------------------------------------------------------------------
        case READ_LORAWAN_DEVEUI:
        case READ_LORAWAN_APPEUI:
        case READ_LORAWAN_APPKEY:
			nvm_fds_eeprom_get(&reply);
			break;

        //-----------------------------------------------------------------------------
		case WRITE_nRF_ADC_CONVERSION:
			saadc_start_conversion(source);
			return;
			break;

		//-----------------------------------------------------------------------------
        case READ_nRF_ADC_CONVERSION:
		{
			get_saadc_result(&reply.param.meas_result);
			break;
		}

        //-----------------------------------------------------------------------------
        case READ_APPLICATION_CONFIG:
			nvm_fds_eeprom_get(&reply);
			break;

        //-----------------------------------------------------------------------------
        case WRITE_APPLICATION_CONFIG:
			ret = nvm_fds_eeprom_set(prot);
            sendResponse(source, WRITE_APPLICATION_CONFIG, ret);
			if(ret == NRF_SUCCESS)
			{
				main_app.new_Timer_config = true;
			}
			return;
			break;

        //-----------------------------------------------------------------------------
		case READ_PINCODE:
			nvm_fds_eeprom_get(&reply);
			break;

		//-----------------------------------------------------------------------------
		case WRITE_PINCODE:
        	ret = nvm_fds_eeprom_set(prot);
            if(ret == NRF_SUCCESS)
            {
                load_passKey();
            }
            sendResponse(source, prot->command, ret);
			return;
			break;

		//-----------------------------------------------------------------------------
		case WRITE_HX711_STATE:
			ret = nvm_fds_eeprom_set(prot);
            sendResponse(source, prot->command, ret);
			return;
			break;

		//-----------------------------------------------------------------------------
		case WRITE_BUZZER_CUSTOM_TUNE:
		{
			BUZZER_s * buzz = &prot->param.buzz;
			ret = Buzzer_sound(buzz->index, 
								(uint32_t) buzz->freq_100Hz * 100, 
								buzz->on_time_ms, 
								buzz->off_time_ms, 
								buzz->repeatCount);
            sendResponse(source, prot->command, ret);
			return;
			break;
		}

        //-----------------------------------------------------------------------------
        case WRITE_BUZZER_DEFAULT_TUNE:
			ret = Buzzer_default_tune(prot->param.buzz.index);
            sendResponse(source, prot->command, ret);
			return;
			break;

        //-----------------------------------------------------------------------------
        case READ_MX_FLASH:
        {
            flashApp_cmd_handler(APP_FLASH_READ, prot);
            return;
            break;
        }

        //-----------------------------------------------------------------------------
        case ERASE_MX_FLASH:
        {
            flashApp_cmd_handler(APP_FLASH_ERASE, prot);
            return;
            break;
        }

        //-----------------------------------------------------------------------------
        case SIZE_MX_FLASH:
        {
            flashApp_cmd_handler(APP_FLASH_SIZE_FLASH, prot);
            return;
            break;
        }

        //-----------------------------------------------------------------------------
        case READ_AUDIO_ADC_CONVERSION:
            audio_app_get_result(&reply.param.meas_result);
            reply.command = READ_AUDIO_ADC_CONVERSION;
			break;

        //-----------------------------------------------------------------------------
        case READ_AUDIO_ADC_CONFIG:
            nvm_fds_eeprom_get(&reply);
			break;

        //-----------------------------------------------------------------------------
        case WRITE_AUDIO_ADC_CONFIG:
            ret = nvm_fds_eeprom_set(prot);
            sendResponse(source, prot->command, ret);
            return;
			break;

        //-----------------------------------------------------------------------------
        case START_AUDIO_ADC_CONVERSION:
            ret = audio_app_start(true, source);

			// Do not send a response for LoRaWAN when ret == NRF_SUCCESS
			if(source != LORAWAN_SOURCE || (source == LORAWAN_SOURCE && ret != NRF_SUCCESS))
			{
				sendResponse(source, prot->command, ret);
			}
			return;
			break;

        //-----------------------------------------------------------------------------
        case READ_BOOT_COUNT:
            nvm_fds_eeprom_get(&reply);
            break;

        //-----------------------------------------------------------------------------
        case ALARM_CONFIG_READ:
            reply.param.alarm.type = prot->param.alarm.type;
            ret = nvm_fds_eeprom_get(&reply);
            if(ret != NRF_SUCCESS)
            {
                sendResponse(source, prot->command, ret);
                return;
            }
            break;

        //-----------------------------------------------------------------------------
        case ALARM_CONFIG_WRITE:
            ret = nvm_fds_eeprom_set(prot);
            sendResponse(source, prot->command, ret);
            return;
            break;

        //-----------------------------------------------------------------------------
        case ALARM_STATUS_READ:
            readAlarmParam(&reply);
			break;

        //-----------------------------------------------------------------------------
		case BME280_CONVERSION_READ:
			bme_app_get_result(&reply.param.meas_result);
            reply.command = BME280_CONVERSION_READ;
			break;

        //-----------------------------------------------------------------------------
		case BME280_CONVERSION_START:
			ret = bme_app_start(true, source);

            // Do not send a response for LoRaWAN when ret == NRF_SUCCESS
			if(source != LORAWAN_SOURCE || (source == LORAWAN_SOURCE && ret != NRF_SUCCESS))
			{
				sendResponse(source, prot->command, ret);
			}
			return;
			break;

        //-----------------------------------------------------------------------------
        case READ_CID_nRF_FLASH:
			break;

        //-----------------------------------------------------------------------------
        case READ_nRF_ADC_CONFIG:
			break;

        //-----------------------------------------------------------------------------
        case READ_APPLICATION_STATE:
			break;

        //-----------------------------------------------------------------------------
		case READ_DS18B20_CONFIG:
			break;

        //-----------------------------------------------------------------------------
        case READ_BME280_I2C:
			break;

        //-----------------------------------------------------------------------------
        case READ_ATECC_I2C:
			break;

        //-----------------------------------------------------------------------------
        case READ_BUZZER_STATE:
			break;

        //-----------------------------------------------------------------------------
        case READ_SQ_MIN_STATE:
			break;
			
		//-----------------------------------------------------------------------------
		default:
			return;
			break;
	}	

	if(source == BLE_SOURCE)
	{
		ble_controlPoint_send(&reply);
	}
	else
	{
		lorawanDownlinkAppend(&reply);
	}
}


static void sampleTimerCallback( void * p_context )
{
	uint32_t ret;

	if(main_app.current_state == HORIZONTAL_ACTIVE && main_app.sample_state == SAMPLE_IDLE)
	{
		main_app.sample_state	= SAMPLE_START;
	}

	#if 0
		static uint16_t samplecount = 0;
		NRF_LOG_INFO("%u - Sample Timer", samplecount++);
	#endif
}

static void sampleTimerStart(bool en)
{
	uint32_t ret;
	uint32_t msInterval;
    BEEP_protocol_s	get;

    main_app.sample_state = SAMPLE_IDLE;
	if(en)
	{
		// Retrieve the interval and message ratio from the flash
		get.command = READ_APPLICATION_CONFIG;
		nvm_fds_eeprom_get(&get);

		main_app.sampleInterval_min			= get.param.status.interval;
		main_app.lorawan_ratio_threshold	= get.param.status.statusflag;
		main_app.lorawan_ratio_counter		= 0;

		// Check for rational sample interval
		if(main_app.sampleInterval_min >= SAMPLE_INTERVAL_MIN_MAX)
		{
			main_app.sampleInterval_min = SAMPLE_INTERVAL_MIN_MAX;
		}

		// Use a faster interval while debugging.
		#if DEBUG
			msInterval = main_app.sampleInterval_min * 1000 * 20;
		#else
			msInterval = main_app.sampleInterval_min * 1000 * 60;
		#endif

		ret = app_timer_stop(sampleTimer);
		ret = app_timer_start(sampleTimer, APP_TIMER_TICKS(msInterval), NULL);
	}
	else
	{
		ret = app_timer_stop(sampleTimer);
	}
}


void main_application_change_state(MAIN_STATE_e newState)
{
	main_app.current_state				= newState;
	main_app.timestamp_current_state	= app_timer_cnt_get();

	#if 1
		const char * stateStrings[] =
		{
			"POWER_ON state",
            "HORIZONTAL_START state",
            "HORIZONTAL_ACTIVE state",
			"HORIZONTAL_DFU_START",
			"HORIZONTAL_DFU_READY",
            "HORIZONTAL_STOP state",
			"HORIZONTAL_IDLE state",
            "VERTICAL_START state",
			"VERTICAL_POWER_DOWN State",
			"ERROR main state!",
		};
		NRF_LOG_INFO("New Main App state: 0x%04X = %s", (uint16_t) newState, ((newState >= ERROR_STATE) ? stateStrings[ERROR_STATE] : stateStrings[newState]));
		NRF_LOG_FLUSH();
	#endif
}


uint32_t main_application_time_since_state_change(void)
{
	uint32_t time_ms;
	uint32_t curr	= app_timer_cnt_get();
	uint32_t dTick	= ((curr > main_app.timestamp_current_state) ? curr - main_app.timestamp_current_state : main_app.timestamp_current_state - curr);
	time_ms			= (dTick * 1000) / APP_TIMER_CLOCK_FREQ;
	return time_ms;
}

static void sample_statemachine(void)
{
	uint32_t ret = 0;

	switch(main_app.sample_state)
	{
		default:
		case SAMPLE_IDLE:
			// Check if the sample interval has been reconfigured and the timer needs to be stopped and reloads 
			if(main_app.new_Timer_config)
			{
				main_app.new_Timer_config = false;
				sampleTimerStart(true);
			}
			break;

		///-------------------------------------------------------------------------------------------------------------------------
		case SAMPLE_START:
			// Check if the LoRaWAN stack isn't transmitting
			if(!lorawan_busy() && !beep_flash_busy())
			{
			#if DISABLE_SAMPLING_SENSORS
				main_app.sample_state			= SAMPLE_WAIT;
			#else
				BEEP_protocol_s hxConf;
                hxConf.command                  = READ_HX711_STATE;
				nvm_fds_eeprom_get(&hxConf);
				main_app.sample_state			= SAMPLE_WAIT;

                saadc_start_conversion(INTERNAL_SOURCE);
                HX711_app_start_sampling(INTERNAL_SOURCE, hxConf.param.status.interval, hxConf.param.status.statusflag);
				DS18B20_App_start_conversion(INTERNAL_SOURCE, 0xFF); //start a conversion with all sensors
                bme_app_start(true, INTERNAL_SOURCE);
				main_app.sample_start_timestamp = app_timer_cnt_get();
			#endif

			// fallthrough to next state
			}
			else
			{
				return;
			}

		///-------------------------------------------------------------------------------------------------------------------------
		case SAMPLE_WAIT:
		{
			const uint32_t timeSinceStart = app_timer_time_since_start_ms(main_app.sample_start_timestamp);

			// Check if a timeout of 5s hass passed or the sensors have completed their conversions.
			if(timeSinceStart >= 100 && !DS18B20_App_busy() && !HX711_app_busy() && !bme_app_busy())
			{
				main_app.sample_state = SAMPLE_AUDIO;
                main_app.sample_start_timestamp = app_timer_cnt_get();
                audio_app_start(true, INTERNAL_SOURCE);
                #if 0
                    NRF_LOG_INFO("Sampling completed!");
                #endif
				// fallthrough to next state	
			}
			else if(timeSinceStart >= (5 * 1000))
			{
				// Timeout
				main_app.sample_state = SAMPLE_AUDIO;
                main_app.sample_start_timestamp = app_timer_cnt_get();
                audio_app_start(true, INTERNAL_SOURCE);
                NRF_LOG_INFO("Sampling timed-out! Busy: DS18B20: %u,  Hx711 %u, BME280: %u",
						DS18B20_App_busy(),
						HX711_app_busy(),
						bme_app_busy());

                // Shutdown the DS18B20 statemachine when still busy
                if(DS18B20_App_busy())
                {
                    DS18B20_App_shutdown();
                }

                // Shutdown the HX711 statemachine when still busy
                if(HX711_app_busy())
                {
                    HX711_app_shutdown();
                }

                // Shutdown the BME statemachine when still busy
                if(bme_app_busy())
                {
                    bme_app_start(false, INTERNAL_SOURCE);
                }

				// fallthrough to next state
			}
			else
			{
				// wait for the timeout or the sensors to complete
				break;
			}
		}

        ///-------------------------------------------------------------------------------------------------------------------------
		case SAMPLE_AUDIO:
		{
			const uint32_t timeSinceStart = app_timer_time_since_start_ms(main_app.sample_start_timestamp);

			// Check if a timeout of 5s hass passed or the sensors have completed their conversions.
			if(timeSinceStart >= 100 && !audio_app_busy())
			{
				main_app.sample_state = SAMPLE_STORE;
                #if 0
                    NRF_LOG_INFO("Sampling Audio completed! %u", timeSinceStart);
                #endif
				// fallthrough to next state	
			}
			else if(timeSinceStart >= (5 * 1000))
			{
				// Timeout  
				main_app.sample_state = SAMPLE_STORE;

                // Force the statemachine to shutdown when still busy.
                if(audio_app_busy())
                {
                    audio_app_start(false, INTERNAL_SOURCE);
                }
                NRF_LOG_INFO("Sampling Audio timed-out!");
				// fallthrough to next state
			}
			else
			{
				// wait for the timeout or the sensors to complete
				break;
			}
		}

		///-------------------------------------------------------------------------------------------------------------------------
        case SAMPLE_STORE:
        {
            bool retBool;
            BEEP_protocol_s prot[5];

            // Add battery voltage
            prot[0].command = READ_nRF_ADC_CONVERSION;
            get_saadc_result(&prot[0].param.meas_result);

            // Add HX711 value
            prot[1].command = READ_HX711_CONVERSION;
            HX711_app_getLastResult(&prot[1].param.meas_result);

            // Add DS18B20 temperature
            prot[2].command = READ_DS18B20_CONVERSION;
            DS18B20_App_getTemp(&prot[2].param.meas_result);

            // Add TLV result
            prot[3].command = READ_AUDIO_ADC_CONVERSION;
            audio_app_get_result(&prot[3].param.meas_result);

            // Add the temperature, Relative Humidity and Barometric pressure to the log
            prot[4].command = BME280_CONVERSION_READ;
            bme_app_get_result(&prot[4].param.meas_result);

            flash_Write_BeepProtocol(BEEP_KEEP_ALIVE, prot, 5);

            // Intentional fall through state
            main_app.sample_state = SAMPLE_SEND;
        }

		///-------------------------------------------------------------------------------------------------------------------------
        case SAMPLE_SEND:
		{
            BEEP_STATUS msgType;
            bool downlinkResponseAvailable = !lorawanDownlinkResponseEmpty(); 

			if(!lorawan_joined_network())
			{
				main_app.sample_state = SAMPLE_IDLE;
				return;
			}

			// Increment the ratio counter and compare it with the treshold value
            main_app.lorawan_ratio_counter++;
			if( main_app.lorawan_ratio_counter >= main_app.lorawan_ratio_threshold || 
                main_app.lorawan_ratio_threshold == 0 || 
                !main_app.joinIndicationSend ||
                downlinkResponseAvailable)
			{
				main_app.lorawan_ratio_counter = 0;
			}
			else
			{
				// Do not transmit 
                NRF_LOG_INFO("LoRaWAN ratio counter: %u/%u", main_app.lorawan_ratio_counter, main_app.lorawan_ratio_threshold);
                main_app.sample_state = SAMPLE_IDLE;
				return;
			}

            if(getAlarmsTransmit(false))
            {
                msgType = BEEP_ALARM;
            }
            else if(!main_app.joinIndicationSend)
            {
                msgType = BEEP_SENSOR_ON;
            }
            else if(downlinkResponseAvailable)
            {
                msgType = BEEP_DOWNLINK_RESPONSE;
            }
            else
            {
                msgType = BEEP_KEEP_ALIVE;
            }

            ret = Beep_SendFormat(msgType, NULL);
            if(msgType == BEEP_SENSOR_ON && ret == NRF_SUCCESS && !main_app.joinIndicationSend)
            {
                main_app.joinIndicationSend = true;
            }
            else if(msgType == BEEP_KEEP_ALIVE && ret == NRF_SUCCESS && !main_app.dataIndicationSend)
            {
                main_app.dataIndicationSend = true;
            }
			main_app.sample_state = SAMPLE_IDLE;
			break;
		}
	}
}


void main_application_while(void)
{
	bool vertical	= SQ_getOrientation();
    bool vbatOk		= saadc_VbatOk();

	switch(main_app.current_state)
	{
		/* Initial state in which the microcontroller gathers information about the battery and orientation. From this state the microcontroller will go to
		 * either Horizontal or Vertical. All sensors are in low power mode and are disabled.
		 * If battery=ok and orientation=horizontal: HORIZONTAL_START
		 * If battery=Nok or orientation=vertical: VERTICAL_START
		 */
		case POWER_ON:
		{
			uint32_t start_up_delay = main_application_time_since_state_change();

			// Wait for a set amount of time or until the SQ and SAADC have completed their start-up
			if((start_up_delay >= (3 * 1000)) || (!SQ_startup() && !saadc_startUp()))
			{
                INTERNAL_PARAM  get;
                MAIN_STATE_e    prevState;
				vbatOk          = saadc_VbatOk();
				bool horizontal = SQ_getOrientation();

                // Get the previous main application state.
                get.param = PREVIOUS_MAIN_STATE;
                nvm_fds_internal_param_get(&get);
                prevState = get.value.previous_state;

                // The system woke-up from a vibration
                if(prevState == VERTICAL_START && !horizontal)
                {
                    // Go immediately to shutdown state and don't sound the buzzer.
                    main_application_change_state(VERTICAL_POWER_DOWN);
                }
				else if(horizontal && vbatOk)
				{
					main_application_change_state(HORIZONTAL_START);
                    get.value.previous_state = HORIZONTAL_START;
                    nvm_fds_internal_param_set(&get);
                    flash_WriteStartUpLine();
				}
				else //if(!vertical || !vbatOk)
				{
					// Go to the low power shutdown mode when either the sensor is in a vertical position or the battery supply voltage is not ok.
					main_application_change_state(VERTICAL_START);
                    get.value.previous_state = VERTICAL_START;
                    nvm_fds_internal_param_set(&get);
				}
			}
			break;	
		}				

		/*
		 * When the sensor is Horizontal this state starts the horizontal mode and turns sensors on when available.
		 */
		case HORIZONTAL_START:
        {
            static bool startUpMelodyStarted = false;
            uint32_t timeout = main_application_time_since_state_change();

            if(!startUpMelodyStarted)
            {
                startUpMelodyStarted = true;
                #if BUZZER_START_UP_MELODY_EN
                    Buzzer_default_tune(0); // Start-up indication melody
                #endif
            }
            else
            {
                // Wait for the buzzer to finish
                if(!BuzzerIsOn() || timeout >= (10 * 1000))
                {
                    startUpMelodyStarted = false;
                    #if BLE_ADV_START_UP_ENABLE
                        // Start BLE advertising when enabled. Might be disabled for power consumption validation
                        advertising_start(false);
                    #endif
                
                    lorawan_AppEnable(LORAWAN_JOIN_ENABLE);
                    sampleTimerStart(true);
                    main_application_change_state(HORIZONTAL_ACTIVE);
                }
            }
			break;
        }

        /*
		 * While the sensor is Horizontal the sensors will be periodically sampled and the data transmitted with LoRaWAN.
		 * Will go to HORIZONTAL_STOP if the orientation changes to vertical.
		 * Will go to HORIZONTAL_STOP if the battery voltage = Nok
		 */
		case HORIZONTAL_ACTIVE:
		{
			sample_statemachine();

            #if 0
            // Check if LoRaWAN joining was completed and no join indication has been transmitted yet.
            if(lorawan_joined_network())
            {
                if(!main_app.joinIndicationSend)
                {
                    // Attempt to send the firmware version
                    if(Beep_SendFormat(BEEP_SENSOR_ON, NULL) == NRF_SUCCESS)
                    {
                        main_app.joinIndicationSend = true;
                        main_app.dataIndicationSend = false;
                        sampleTimerCallback(NULL); // Start a conversion 
                    }
                }
                else if(!main_app.dataIndicationSend && main_app.sample_state == SAMPLE_IDLE)
                {
                    // Attempt to send the firmware version
                    if(Beep_SendFormat(BEEP_KEEP_ALIVE, NULL) == NRF_SUCCESS)
                    {
                        main_app.dataIndicationSend = true;
                    }
                }
            }
            else
            {
                 main_app.joinIndicationSend = false;
                 main_app.dataIndicationSend = false;
            }
            #endif
            

			if(enterDFU)
			{
				// Enter the HORIZONTAL_DFU_START to finish the current tasks and then enter DFU ready mode.
                lorawan_AppEnable(false);
                sampleTimerStart(false);
				main_application_change_state(HORIZONTAL_DFU_START);
			}
			else if(!vertical || !vbatOk)
			{
				main_application_change_state(HORIZONTAL_STOP);
			}
			break;
		}

        /*
		 * Powers the sensors down if needed, then continues to VERTICAL_START.
		 */
		case HORIZONTAL_STOP:
			lorawan_AppEnable(false);
			advertising_stop();
            sampleTimerStart(false);
			disconnect_clients();

			if(vbatOk)
			{
				main_application_change_state(VERTICAL_START);
			}
			else
			{
				main_application_change_state(HORIZONTAL_IDLE);
			}
			break;

		/*
		 * Wait for the Application to finish certain tasks and then continue to the DFU ready state
		 */
		case HORIZONTAL_DFU_START:
		{
			uint32_t timeout = main_application_time_since_state_change();
			bool loraBusy	= lorawan_busy();
			bool buzzerOn	= BuzzerIsOn();
			bool dsBusy		= DS18B20_App_busy();
			bool hxBusy		= HX711_app_busy();
			bool nvmBusy	= nvm_fds_check_busy();
			bool notBusy	= ((!loraBusy && !buzzerOn && !dsBusy && !hxBusy && !nvmBusy));

			// Wait for all sensors to be shutdown or a timeout
			if(timeout >= (3 * 1000) || notBusy)
			{
				if(notBusy)
				{
					NRF_LOG_INFO("HORIZONTAL_DFU_START: Sensors shutdown");
				}
				else
				{
					NRF_LOG_INFO("HORIZONTAL_DFU_START: Timeout")
				}

				if(powerApp_getEnabled(PWR_ALL))
				{
					powerApp_Enable(false, PWR_ALL);
				}

				// Complete the last processes before going to sleep.
				main_application_change_state(HORIZONTAL_DFU_READY);
				#ifdef DEBUG
					NRF_LOG_PROCESS();
				#else
					NRF_LOG_FINAL_FLUSH();
				#endif
			}
			break;
		}

		/*
		 * Ready for DFU start from the app_shutdown_handler.
		 */
		case HORIZONTAL_DFU_READY:
			break;

		/*
		 * Horizontal, but Battery power isn't ok. We could go to Horizontal IDLE, but this would be quite definitive, as it would only wake-up from user interaction.
		 * As long as the power supply remains nok and the sensor remains horizontal, the application will remain here.
		 */
		case HORIZONTAL_IDLE:

			// Restart the sensors when the supply voltage is ok again.
			if(vbatOk)
			{
				main_application_change_state(HORIZONTAL_START);
			}

			if(!vertical)
			{
				main_application_change_state(VERTICAL_START);
			}
			break;

		/*
		 * Powers the sensors down if needed and preperes for power down, then continues to VERTICAL_POWER_DOWN.
		 */
		case VERTICAL_START:
			#ifndef DEBUG
				SQ_deinit();
			#endif

            // Check if the battery voltage is ok, otherwise don't play the shutdown tune
            #if BUZZER_START_UP_MELODY_EN
                if(saadc_VbatOk())
                {
                    Buzzer_default_tune(1); // Shutdown tune
                }
            #endif
            main_application_change_state(VERTICAL_POWER_DOWN);
			break;

        /*
		 * The MCU will now powerdown and can only wake-up with a reset, orientation change or reed switch
		 */
		case VERTICAL_POWER_DOWN:
		{
			bool loraBusy = lorawan_busy();
			bool buzzerOn = BuzzerIsOn();
            uint32_t start_up_delay = main_application_time_since_state_change();

			if((start_up_delay >= (3 * 1000)) || (!loraBusy && !buzzerOn))
			{
				// Check if the locks bits are still set and disable all when a lock is present.
				if(powerApp_getEnabled(PWR_ALL))
				{
					powerApp_Enable(false, PWR_ALL);
				}

                // Process the fds
                nvm_fds_check_pending();

				// Complete the last processes before going to sleep.
				NRF_LOG_PROCESS();
				NRF_LOG_FINAL_FLUSH();

				// Power the MCU down
				#ifndef DEBUG
					BoardDeInitMcu();
				#else
					// Add an return state for debugging, so the microcontroller doesn't crashed the debugger.
					if(vertical)
					{
						main_application_change_state(HORIZONTAL_START);
					}
				#endif
			}
			break;
		}
	} // switch(main_app.current_state)
}



static void power_management_init(void)
{
    uint32_t err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}




/**
 * @brief Function for main application entry.
 */
int main(void)
{
	uint32_t ret;

    GpioMcuInit();
    RFM_LowPowerMode();// Disable the clock output on DIO5 and set the RFM to sleep mode.
    
    #if NRF_LOG_USES_TIMESTAMP
		ret = NRF_LOG_INIT(get_timestamp_counter);
    #else
		ret = NRF_LOG_INIT(NULL);
    #endif   

    APP_ERROR_CHECK(ret);
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    bootloader_setupSVCI();

    BoardInitPeriph();

    power_management_init();

    NRF_LOG_INFO( "###### ===== Beep application start-up ==== ######" );

    SQ_init(measurement_handler);

    // Force the device to write the new bootcount to flash.
    nvm_fds_changed();
    
    lorawan_implementation_init(beep_ctrlpt_event_handler);

    saadc_start(0, measurement_handler);

	HX711_app_Init(measurement_handler);

	// Init the OWI bus and search for DS18B20 devices
    DS18B20_App_init(300, TEMP_12BIT_RESOLUTION, measurement_handler);

	// Buzzer initialization
    Buzzer_init(0);

    ret = app_timer_create(&sampleTimer, APP_TIMER_MODE_REPEATED, sampleTimerCallback);
    APP_ERROR_CHECK(ret);

    reedswitchTimerInit();

	#if ATECC_ENABLED
		atecc_getID();
	#endif

    //Init the MX25R6435F Flash IC
    MXflash_FSinit();

    audio_app_init(measurement_handler);
    BLE_setup(beep_ctrlpt_event_handler);
    main_application_change_state(POWER_ON);
    audioFFT_init();

    bme_app_init(measurement_handler);

    while (1)
    {
        DS18B20_App_while();

        HX711_app_While();

        lorawan_implementation_while();

        main_application_while();

        beep_fileTransfer_while();

        audio_app_while();

        bme_app_while();

        NRF_LOG_PROCESS();

        if(!DS18B20_App_busy())
		{
			nvm_fds_check_pending();
		}

        // Handle encryption requests when needed
        err_code = nrf_ble_lesc_request_handler();
        APP_ERROR_CHECK(err_code);

		if(!lorawan_busy() && !DS18B20_App_busy() && !audio_app_sleep())
		{
			// The MCU wakes up through events
			BoardLowPowerHandler( );
		}
    }
}

/** @} */
