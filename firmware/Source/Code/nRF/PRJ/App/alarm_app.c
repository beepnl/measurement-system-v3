#define NRF_LOG_MODULE_NAME Error
#include "alarm_app.h"
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
#include "nvm_fs.h"


APP_SAMPLE_ERROR_s err;

uint16_t getAlarmsTransmit(bool clear)
{
    uint16_t retVal     = err.transmitErrors;

    if(clear)
    {
        err.transmitErrors  = 0;
    }
    return retVal;
}


uint16_t getAlarmsActive(void)
{
    return err.activeErrors;
}


void readAlarmParam(BEEP_protocol_s * reply)
{
    if(reply == NULL){
        return;
    }

    reply->param.status.statusflag = (uint8_t) err.activeErrors;
}


static void setAlarmBit(bool en, SENSOR_TYPE type)
{
    uint16_t prev = err.activeErrors;
    if(en)
    {
        err.activeErrors |= (1<<(uint8_t) type);
    }
    else
    {
        err.activeErrors &= ~(1<<(uint8_t) type);
    }

    // Set the errors for transmission
    err.transmitErrors |= err.activeErrors;

    // Notify on Change
    if(prev != err.activeErrors)
    {
        NRF_LOG_INFO("Alarmbit %u/%s %s, status: 0x%04X", (uint16_t) type, beep_protocol_sensor_strget(type), ((en) ? "set": "cleared"), err.activeErrors);
    }
}


void checkSample(MEASUREMENT_RESULT_s * meas)
{
    BEEP_protocol_s flash;
    memset(&flash, 0, sizeof(BEEP_protocol_s));

    switch(meas->type)
    {
        //###############################################################################################################################################################
        case DS18B20:
        {
            bool DSalarmActive = false;
            uint8_t i;
            DS18B20_RESULTS_s * ds      = &meas->result.ds18B20;
            DS_ALARM_s        * DSthr   = &flash.param.alarm.thr.ds;

            // Get the Min, Max and Diff values from the flash storage
            flash.command           = ALARM_CONFIG_READ;
            flash.param.alarm.type  = DS18B20;

            nvm_fds_eeprom_get(&flash);

            for(i=0; i<ds->devices; i++)
            {
                // Calaculate the absolute difference from the current and last measurement.
                int16_t diff_temp   = 0;
                int16_t temp_now    = ds->temperatures[i];
                int16_t temp_prev   = err.ds18B20.temperatures[i];

                // Check if the previous sample has a temperature measurement.
                if(i < err.ds18B20.devices)
                {
                    diff_temp = (temp_now > temp_prev) ?  temp_now - temp_prev : temp_prev - ds->temperatures[i];
                }

                // Compare the current temperature measurement with the min,max and Diff thresholds.
                if( (temp_now >= DSthr->Max && DSthr->Max != INT16_MAX) || 
                    (temp_now <= DSthr->Min && DSthr->Min != INT16_MIN) || 
                    (diff_temp > DSthr->Diff && DSthr->Diff != UINT16_MAX))
                {
                    DSalarmActive = true;
                }
            }

            // Store the result in a bit-wise variable
            setAlarmBit(DSalarmActive, DS18B20);

            // Save the latest measurement result for the next sample
            memcpy(&err.ds18B20, ds, sizeof(DS18B20_RESULTS_s));
            break;
        }

        //###############################################################################################################################################################
        case HX711:
        {
            bool HXalarmActive = false;
            uint8_t i;
            HX711_CONV_s * hx   = &meas->result.hx711;
            HX711_ALARM_s * HXthr = &flash.param.alarm.thr.hx;

            // Get the Min, Max and Diff values from the flash storage
            flash.command           = ALARM_CONFIG_READ;
            flash.param.alarm.type  = HX711;

            nvm_fds_eeprom_get(&flash);

            for(i=0; i<HX711_N_CHANNELS; i++)
            {
                // Calaculate the absolute difference from the current and last measurement.
                uint32_t diff_adc = 0;
                int32_t hx_now  = hx->value[i];
                int32_t hx_prev = err.hx711.value[i];

                if(!(hx->channel & (1<<i)))
                {
                    continue;
                }

                // Check if the current and previous measurements have a sample result of that channel.
                if(err.hx711.channel & (1<<i))
                {
                    diff_adc = (uint32_t) (hx_now > hx_prev) ? (hx_now - hx_prev) : (hx_prev - hx_now);
                }

                // Compare the current temperature measurement with the Min, Max and Diff thresholds.
                if( (hx_now >= HXthr->Max && HXthr->Max != INT32_MAX) || 
                    (hx_now <= HXthr->Min && HXthr->Min != INT32_MIN) || 
                    (diff_adc > HXthr->Diff && HXthr->Diff < UINT32_MAX))
                {
                    HXalarmActive = true;
                }
            }

            // Store the result in a bit-wise variable
            setAlarmBit(HXalarmActive, HX711);

            // Save the latest measurement result for the next sample
            memcpy(&err.hx711, hx, sizeof(HX711_CONV_s));
            break;
        }

        //###############################################################################################################################################################
        case nRF_ADC:
        {
            bool ADCalarmActive = false;
            uint8_t i;
            ADC_s * adc             = &meas->result.saadc;
            SUPPLY_ALARM_s * ADCthr = &flash.param.alarm.thr.supply;
            uint16_t diff_bat_mV, diff_vcc_mV;
            uint16_t bat_prev       = err.saadc.battVoltage_mV;
            uint16_t bat_now        = adc->battVoltage_mV;
            uint16_t vcc_prev       = err.saadc.vccVoltage_mV;
            uint16_t vcc_now        = adc->vccVoltage_mV;

            // Get the Min, Max and Diff values from the flash storage
            flash.command           = ALARM_CONFIG_READ;
            flash.param.alarm.type  = nRF_ADC;

            nvm_fds_eeprom_get(&flash);

            diff_bat_mV = 0;
            if(bat_prev != 0)
            {
                diff_bat_mV = (bat_prev > bat_now) ? (bat_prev - bat_now) : (bat_now - bat_prev);
            }

            if( (bat_now < ADCthr->Min && ADCthr->Min != 0) || 
                (bat_now > ADCthr->Max && ADCthr->Max != UINT16_MAX) || 
                (diff_bat_mV > ADCthr->Diff && ADCthr->Diff != UINT16_MAX))
            {
                ADCalarmActive = true;
            }

            diff_vcc_mV = 0;
            if(vcc_prev != 0)
            {
                diff_vcc_mV = (vcc_prev > vcc_now) ? (vcc_prev - vcc_now) : (vcc_now - vcc_prev);
            }

            if( (vcc_now < ADCthr->Min && ADCthr->Min != 0) || 
                (vcc_now > ADCthr->Max && ADCthr->Max != UINT16_MAX) || 
                (diff_vcc_mV > ADCthr->Diff && ADCthr->Diff != UINT16_MAX))
            {
                ADCalarmActive = true;
            }


            // Store the result in a bit-wise variable
            setAlarmBit(ADCalarmActive, nRF_ADC);

            // Save the latest measurement result for the next sample
            memcpy(&err.saadc, adc, sizeof(ADC_s));
            break;
        }

        //###############################################################################################################################################################   	
        case BME280:
        {
			bool BMEalarmActive = false;
			int16_t temp_now, temp_prev;
			uint16_t humidity_now, humidity_prev;
			uint16_t pressure_now, pressure_prev;
			uint16_t temp_diff, humidity_diff, pressure_diff;

			BME280_RESULT_s * bme = &meas->result.bme280;
            BME_ALARM_s * BMEthr  = &flash.param.alarm.thr.bme;

            // Get the Min, Max and Diff values from the flash storage
            flash.command           = ALARM_CONFIG_READ;
            flash.param.alarm.type  = BME280;
            nvm_fds_eeprom_get(&flash);

			// Temperature
            temp_diff	= 0;
            temp_now	= bme->temperature;
            temp_prev	= err.bme280.temperature;

            if(temp_prev != 0)
            {
                temp_diff = (temp_prev > temp_now) ? (temp_prev - temp_now) : (temp_now - temp_prev);
            }

            if( (temp_now < BMEthr->Temp_Min	&& BMEthr->Temp_Min  != INT16_MIN) || 
                (temp_now > BMEthr->Temp_Max	&& BMEthr->Temp_Max  != INT16_MAX) || 
                (temp_diff > BMEthr->Temp_Diff	&& BMEthr->Temp_Diff != UINT16_MAX && BMEthr->Temp_Diff != 0))
            {
                BMEalarmActive = true;
            }

			// Humidity
            humidity_diff	= 0;
            humidity_now	= bme->humidity;
            humidity_prev	= err.bme280.humidity;

            if(humidity_prev != 0)
            {
                humidity_diff = (humidity_prev > humidity_now) ? (humidity_prev - humidity_now) : (humidity_now - humidity_prev);
            }

            if( (humidity_now <  BMEthr->humidity_Min	&& BMEthr->humidity_Min  != 0) || 
                (humidity_now >  BMEthr->humidity_Max	&& BMEthr->humidity_Max  != UINT16_MAX) || 
                (humidity_diff > BMEthr->humidity_Diff	&& BMEthr->humidity_Diff != UINT16_MAX && BMEthr->humidity_Diff != 0))
            {
                BMEalarmActive = true;
            }

			// Pressure
            pressure_diff	= 0;
            pressure_now	= bme->airPressure;
            pressure_prev	= err.bme280.airPressure;

            if(pressure_prev != 0)
            {
                pressure_diff = (pressure_prev > pressure_now) ? (pressure_prev - pressure_now) : (pressure_now - pressure_prev);
            }

            if( (pressure_now <  BMEthr->press_Min	&& BMEthr->press_Min  != 0) || 
                (pressure_now >  BMEthr->press_Max	&& BMEthr->press_Max  != UINT16_MAX) || 
                (pressure_diff > BMEthr->press_Diff	&& BMEthr->press_Diff != UINT16_MAX && BMEthr->press_Diff != 0))
            {
                BMEalarmActive = true;
            }

			// Store the result in a bit-wise variable
            setAlarmBit(BMEalarmActive, BME280);

            // Save the latest measurement result for the next sample
            memcpy(&err.bme280, bme, sizeof(BME280_RESULT_s));
            break;
        }
          
        //###############################################################################################################################################################        
        case ATECC:
        case BUZZER:
        case SQ_MIN:
        case LORAWAN:
        case MX_FLASH:
        case nRF_FLASH:
        case APPLICATIE:
        default:
            return;
            break;
    }
}




void testAlarm(void)
{
    int16_t temp;
    uint8_t i;
    MEASUREMENT_RESULT_s measDS, measADC, measHX;
    memset(&measDS,  0, sizeof(MEASUREMENT_RESULT_s));
    memset(&measADC, 0, sizeof(MEASUREMENT_RESULT_s));
    memset(&measHX,  0, sizeof(MEASUREMENT_RESULT_s));

    DS18B20_RESULTS_s * ds  = &measDS.result.ds18B20;
    measDS.source = DS18B20;
    ds->devices = 2;

    // -100C to 100C in steps of 0.01 C
    for(temp=(-26*100); temp<(100*100); temp+=100)
    {
        for(i=0; i<ds->devices; i++)
        {
            ds->temperatures[i] = temp;
        }
        checkSample(&measDS);
    }

    // Test the ADC thresholds
    ADC_s * adc             = &measADC.result.saadc;
    measADC.type            = nRF_ADC;
    uint16_t supply;

    for(supply=1700; supply<=3400; supply+=100)
    {
        adc->battVoltage_mV = supply;
        adc->vccVoltage_mV  = supply;
        checkSample(&measADC);
    }

    // Test the HX711
    HX711_CONV_s * hx   = &measHX.result.hx711;
    int32_t val;
    measHX.type = HX711;

    for(val=-20000; val<20000; val+=500)
    {
        hx->channel = 0x07;
        hx->value[0] = (hx->channel & (1<<0)) ? val : 0;
        hx->value[1] = (hx->channel & (1<<1)) ? val : 0;
        hx->value[2] = (hx->channel & (1<<2)) ? val : 0;
        checkSample(&measHX);
    }
}

