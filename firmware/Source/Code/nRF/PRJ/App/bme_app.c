#define NRF_LOG_MODULE_NAME BME_APP
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
#include "bme_app.h"
#include "nvm_fs.h"
#include "app_timer.h"
#include "I2C2.h"
#include "BME280.h"
#include "power_app.h"
#include "gpio-board.h"
#include "nrf_delay.h"
#include "nvm_fs.h"
#include "app_timer.h"
#include <stdio.h>

struct bme280_dev	dev;
struct bme280_data	comp_data;
BEEP_protocol_s     bme_result;
BEEP_protocol_s     bme_settings;
APP_TIMER_DEF(bmeTimer);

BME_APPLICATIONs bme = 
{
    .state = BME_IDLE,
};


bool bme_app_busy(void)
{
    #if !BME280_ENABLE
        return false;
    #else
        // When a data block is available when in BME_IDLE state, don't go to sleep.
        return (bme.state != BME_IDLE);
    #endif
}

bool bme_app_sleep(void)
{
    #if !BME280_ENABLE
        return false;
    #else
        return ((bme.state == BME_IDLE) || (bme.state == BME_READ_RESULT)) ? false : true;
    #endif
}


static void bme_timerStart(uint32_t delay_ms)
{
	uint32_t ret;
	ret = app_timer_start(bmeTimer, APP_TIMER_TICKS(delay_ms), NULL);
	bme.waitDone = false;
}


static void bme_app_nextState(BME_STATES next)
{
    bme.state                 = next;
    bme.timestampStateChanged = app_timer_cnt_get();
    #if BME_APP_LOG_ENABLED
        NRF_LOG_INFO("New state: %u", (uint32_t) bme.state);
    #endif
}


int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	uint32_t ret = 0;
    ret = I2C2_readArray(id, reg_addr, data, len);
    if(ret != NRF_SUCCESS){
        return BME280_E_COMM_FAIL;
    }

    return BME280_OK;
}


int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	uint32_t ret = 0;
	ret = I2C2_writeArray(id, reg_addr, data, len);
    if(ret != NRF_SUCCESS)
	{
        return BME280_E_COMM_FAIL;
    }
    return BME280_OK;
}


uint32_t bme_app_config(void)
{
    int8_t rslt;
    uint8_t settings_sel;
    struct bme280_data comp_data;

    /* Recommended mode of operation: Indoor navigation */
    dev.settings.osr_h	= BME280_OVERSAMPLING_1X;
    dev.settings.osr_p	= BME280_OVERSAMPLING_16X;
    dev.settings.osr_t	= BME280_OVERSAMPLING_2X;
    dev.settings.filter = BME280_FILTER_COEFF_16;

    settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

    rslt = bme280_set_sensor_settings(settings_sel, &dev);
    if (rslt != BME280_OK)
    {
        NRF_LOG_INFO("Failed to set sensor settings: %i/0x%02X.", rslt, rslt);
        return NRF_ERROR_INTERNAL;
    }
	return NRF_SUCCESS;
}

void bme_print_sensor_data(struct bme280_data *comp_data)
{
	char log [36] = {0};
    float temp, press, hum;
#ifdef BME280_FLOAT_ENABLE
    temp = comp_data->temperature;
    press = 0.01 * comp_data->pressure;
    hum = comp_data->humidity;
#else
#ifdef BME280_64BIT_ENABLE
    temp    = 0.01f * comp_data->temperature;
    press   = 0.0001f * comp_data->pressure;
    hum     = 1.0f / 1024.0f * comp_data->humidity;
#else
    temp = 0.01f * comp_data->temperature;
    press = 0.01f * comp_data->pressure;
    hum = 1.0f / 1024.0f * comp_data->humidity;
#endif
#endif
    snprintf(log, 100, "%0.2f deg C, %0.2f hPa, RH: %0.2f%%", temp, press, hum);
	NRF_LOG_INFO("%s", NRF_LOG_PUSH(log));
}



void bme_app_while(void)
{
    uint32_t retval;
    int8_t rslt;

    #if !BME280_ENABLE
		return;
    #else

    switch(bme.state)
    {
        //-----------------------------------------------------------------------------------------------------------------------------------------------------------------
        case BME_IDLE:
            break;

        //-----------------------------------------------------------------------------------------------------------------------------------------------------------------
        case BME_START:
        {
            BME_CONFIG_s * config = &bme_settings.param.bme_config;

            // Initialize the I2C and I2S peripherals for communication with the Audio ADC
            if(!powerApp_getEnabled(PWR_BME280))
			{
				powerApp_Enable(true, PWR_BME280);

                // Retrieve the flash settings from the FLASH storage
                bme_settings.command = BME280_CONFIG_READ;
                nvm_fds_eeprom_get(&bme_settings);
			}
            else if(I2C2_state() != NRFX_DRV_STATE_POWERED_ON)
            {
                I2C2_init();
				#if 0
					I2C2_searchSlaves();
				#endif
            }
            else
            {
				dev.dev_id		= BME280_I2C_ADDR_PRIM;
				dev.intf		= BME280_I2C_INTF;
				dev.read		= user_i2c_read;
				dev.write		= user_i2c_write;
				dev.delay_ms	= nrf_delay_ms;

				rslt = bme280_init(&dev);

                // If the BME isn't detected, disable the statemachine
                if(rslt == BME280_E_DEV_NOT_FOUND)
                {
                    bme_app_nextState(BME_STOP);
                    return;
                }
				else if(rslt != BME280_OK)
				{
					return;
				}
                
				rslt = bme_app_config();

                if (rslt != BME280_OK)
				{
                    bme_app_nextState(BME_STOP);
					return;
				}

                // Set the sensor mode.
				rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
				if (rslt != BME280_OK)
				{
					NRF_LOG_INFO("Failed to set sensor mode %i/0x%02X.", rslt, rslt);
                    bme_app_nextState(BME_STOP);
                    return;
				}
				/* Wait for the measurement to complete and print data @25Hz */
				bme_timerStart(40);

                bme_app_nextState(BME_READ_RESULT);
			}
            break;
        }

        //-----------------------------------------------------------------------------------------------------------------------------------------------------------------
        case BME_READ_RESULT:
		{
            const uint32_t timeSinceStart = app_timer_time_since_start_ms(bme.timestampStateChanged);

			// Return while the sample delays hasn't finished yet.
			if(!bme.waitDone && timeSinceStart < 1000)
			{
				return;
			}

			rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
			if (rslt != BME280_OK)
			{
				NRF_LOG_INFO("Failed to get sensor data code %i/0x%02X.", rslt, rslt);
                bme_app_nextState(BME_STOP);
				break;
			}
            
            /*
             * Convert the temperature to an int16_t with 2 decimals accuracy,
             * Convert the Relative Humidity to a uint16_t with 2 decimal accuracy
             * Convert the Barrometric Pressure to an uin16_t with integer values (0 decimal accuracy)
             */
			bme_result.param.meas_result.result.bme280.temperature  = (int16_t)  comp_data.temperature;
			bme_result.param.meas_result.result.bme280.humidity		= (uint16_t) ((comp_data.humidity * 100) / 1024);
			bme_result.param.meas_result.result.bme280.airPressure	= (uint16_t) (comp_data.pressure / 10000);

            bme_print_sensor_data(&comp_data);
                
			if(bme.callback != NULL)
			{
				bme_result.param.meas_result.type	= BME280;
				bme_result.param.meas_result.source = bme.source;
				bme.callback(&bme_result.param.meas_result);
			}

            // fall-through statement
			bme_app_nextState(BME_STOP);
		}

        //-----------------------------------------------------------------------------------------------------------------------------------------------------------------
        case BME_STOP:

            // Disable the power to the TLV Audio ADC
            if(powerApp_getEnabled(PWR_BME280))
			{
                I2C2_uninit();
				powerApp_Enable(false, BME280);
			}
            else
            {
                bme_app_nextState(BME_IDLE);
            }
            break;

        //-----------------------------------------------------------------------------------------------------------------------------------------------------------------
        default:
            break;
    }
    #endif
}


uint32_t bme_app_get_result(MEASUREMENT_RESULT_s * ret)
{
	if(ret == NULL)
	{
		return NRF_ERROR_INVALID_PARAM;
	}

    memset(ret, 0, sizeof(MEASUREMENT_RESULT_s));
    bme_result.param.meas_result.type = BME280;
	memcpy(ret, &bme_result.param.meas_result, sizeof(MEASUREMENT_RESULT_s));
    return NRF_SUCCESS;
}


uint32_t bme_app_start(const bool en, CONTROL_SOURCE source)
{
    #if !BME280_ENABLE
        return NRF_ERROR_INVALID_STATE;
    #else
        if(en)
        {
            if(bme.state == BME_IDLE)
            {
                bme.source = source;
                bme_app_nextState(BME_START);
                return NRF_SUCCESS;
            }
            else
            {
                return NRF_ERROR_INVALID_STATE;
            }
        }
        else
        {
            if(bme.state != BME_IDLE)
            {
                bme_app_nextState(BME_STOP);
                return NRF_SUCCESS;
            }
            else
            {
                return NRF_ERROR_INVALID_STATE;
            }
        }
    #endif
}


void bme_TimerCallback(void * p_context)
{
	if(bme.state == BME_READ_RESULT)
	{
		bme.waitDone = true;
	}
}

void bme_app_init(measurement_callback measurement_handler)
{
	uint32_t ret;
    bme_app_nextState(BME_IDLE);
    bme.callback      = measurement_handler;

    ret = app_timer_create(&bmeTimer, APP_TIMER_MODE_SINGLE_SHOT, bme_TimerCallback);
    APP_ERROR_CHECK(ret);
}

