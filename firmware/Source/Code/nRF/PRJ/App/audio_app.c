#define NRF_LOG_MODULE_NAME AUDIO_APP
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
#include "audio_app.h"
#include "nvm_fs.h"
#include "app_timer.h"
#include "I2S.h"
#include "I2C.h"
#include "TLV320ADC3100.h"
#include "power_app.h"
#include "arm_const_structs.h"
#include "audioFFT.h"
#include "gpio-board.h"
#include "nrf_delay.h"
#include "nvm_fs.h"

static uint32_t m_buffer_rx[2][I2S_DATA_BLOCK_WORDS];
static volatile bool fftIsBusy = false;
static float m_fft_output_f32[I2S_DATA_BLOCK_WORDS];             //!< FFT output data. Frequency domain.
static float m_fft_input_f32[FFT_COMPLEX_INPUT] = {0};     //!< FFT input array for complex numbers. Time domain.

static uint32_t     fftSamples[I2S_DATA_BLOCK_WORDS] = {0};
BEEP_protocol_s     fft_result;
BEEP_protocol_s     settings;

AUDIO_APPLICATIONs audio = 
{
    .state = AUDIO_IDLE,
};


bool audio_app_busy(void)
{
    #if !TLV_ENABLE
        return false;
    #else
        // When a data block is available when in AUDIO_IDLE state, don't go to sleep.
        return (audio.state != AUDIO_IDLE);
    #endif
}

bool audio_app_sleep(void)
{
    #if !TLV_ENABLE
        return false;
    #else
        // When a data block is available when in AUDIO_IDLE state, don't go to sleep.
        return ((audio.state == AUDIO_IDLE && !fftIsBusy) || (audio.state == AUDIO_SAMPLING)) ? false : true;
    #endif
}


static void print_output_data(float const * p_block, uint16_t size)
{
    uint16_t i;

    NRF_LOG_RAW_INFO("ARM_OUT = [");

    for (i = 0; i < size; ++i)
    {
		if(i < (size - 1))
		{
			NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER", ", NRF_LOG_FLOAT(p_block[i]));
			NRF_LOG_FLUSH();
		}
		else
		{
			NRF_LOG_RAW_INFO(" "NRF_LOG_FLOAT_MARKER" ];\n", NRF_LOG_FLOAT(p_block[i]));
			NRF_LOG_FLUSH();
		}
    }	
}

static void print_i2S_data(uint32_t const * p_block, uint16_t size)
{
    uint16_t i;
    int16_t val;

    NRF_LOG_RAW_INFO("R = [");
    NRF_LOG_FLUSH();

    for (i = 0; i<size; i++)
    {
        val = (uint16_t)(p_block[i] >> 8);
		if(i < (size - 1))
		{
			NRF_LOG_RAW_INFO("%i, ", val);
		}
		else
		{
			NRF_LOG_RAW_INFO(" %i ];\n", val);
			
		}
        NRF_LOG_FLUSH();
    }	
}



static void audio_app_nextState(AUDIO_STATES next)
{
    audio.state                 = next;
    audio.timestampStateChanged = app_timer_cnt_get();
    #if AUDIO_APP_LOG_ENABLED
        NRF_LOG_INFO("New state: %u", (uint32_t) audio.state);
    #endif
}


static void data_handler(nrf_drv_i2s_buffers_t const * p_released, uint32_t status)
{
    // 'nrf_drv_i2s_next_buffers_set' is called directly from the handler
    // each time next buffers are requested, so data corruption is not
    // expected.
    ASSERT(p_released);

    // When the handler is called after the transfer has been stopped
    // (no next buffers are needed, only the used buffers are to be
    // released), there is nothing to do.
    if (!(status & NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED))
    {
        return;
    }

    // First call of this handler occurs right after the transfer is started.
    // No data has been transferred yet at this point, so there is nothing to
    // check. Only the buffers for the next part of the transfer should be
    // provided.
    if (!p_released->p_rx_buffer)
    {
        nrf_drv_i2s_buffers_t const next_buffers = 
        {
            .p_rx_buffer = m_buffer_rx[1],
            .p_tx_buffer = NULL,
        };
        APP_ERROR_CHECK(nrf_drv_i2s_next_buffers_set(&next_buffers));
    }
    else
    {
        #if 0
            NRF_LOG_INFO("Buffer released: %u, FFTdone: %u", p_released->p_rx_buffer, (uint32_t) fftIsBusy);
        #endif

        // Check if the FFT is ready to accept new data
        if(!fftIsBusy)
        {
            memcpy(fftSamples, p_released->p_rx_buffer, sizeof(fftSamples));
            fftIsBusy = true;
        }
        
        // The driver has just finished accessing the buffers pointed by
        // 'p_released'. They can be used for the next part of the transfer
        // that will be scheduled now.
        APP_ERROR_CHECK(nrf_drv_i2s_next_buffers_set(p_released));
    }
}



static void fft_create_result(BEEP_protocol_s * ret, float * fft, uint16_t count, uint16_t start, uint16_t stop, uint16_t fftSize)
{
    uint16_t binOffset;
    uint16_t i, j, sumNbins, diff;
    float binSum;
    FFT_RESULTS * result;

    memset(ret, 0, sizeof(BEEP_protocol_s));
    result = &ret->param.meas_result.result.fft;
    result->bins    = 0;
    result->start   = start / 2;
    result->stop    = stop  / 2;
    ret->param.meas_result.type = AUDIO_ADC;

    if(fftSize < count || fftSize == 0 || count > FFT_MAX_BINS || count == 0 || start >= stop)
    {
        return;
    }

    diff = stop - start;
    sumNbins = diff / count;

    /*
     * Round up the number of bins to sum when the number of bins times the size is smaller than the fft size
     * start=0, stop=255
     * 256 / 20 = 13.8
     * 13 * 20 = 240. 240 < 256 -> sumNbins = 14
     */
    if((count * sumNbins) < diff)
    {
        sumNbins++;
    }
    binOffset = start;
     
    for(i=0; i<count; i++)
    {
        binSum = 0.0;

        for(j=0; j<sumNbins; j++)
        {
            if(binOffset >= stop)
            {
                break;
            }
            binSum += fft[binOffset];
            binOffset++;
        }

        #if 0
            // Average bin value
            float binAverage = binSum / (float) j;
            result->values[i] = (binAverage > UINT16_MAX) ? UINT16_MAX : (uint16_t) binAverage;
        #else
            // Total sum value per bin
            result->values[i] = (binSum > UINT16_MAX) ? UINT16_MAX : (uint16_t) binSum;
        #endif
        result->bins++;
    }

    #if AUDIO_APP_LOG_ENABLED

        NRF_LOG_INFO("FFT result start %u/%u Hz, stop: %u/%u Hz, bin count: %u, samples/bin %u", start, start * FFT_TEST_SAMPLE_RES_HZ,  stop, stop * FFT_TEST_SAMPLE_RES_HZ, count, sumNbins);

        for(i=0; i<count; i++)
        {
            // Total sum value per bin
            NRF_LOG_INFO("[%u] = %u", i, result->values[i]);
        }
    #endif
}



void audio_app_while(void)
{
    uint32_t retval;

    #if !TLV_ENABLE
    #else

    switch(audio.state)
    {
        //-----------------------------------------------------------------------------------------------------------------------------------------------------------------
        case AUDIO_IDLE:
            break;

        //-----------------------------------------------------------------------------------------------------------------------------------------------------------------
        case AUDIO_START:
        {
            AUDIO_CONFIG_s * config = &settings.param.audio_config;

            // Initialize the I2C and I2S peripherals for communication with the Audio ADC
            if(!powerApp_getEnabled(PWR_AUDIO))
			{
				powerApp_Enable(true, PWR_AUDIO);
                TLV_reset();

                // Retrieve the flash settings from the FLASH storage
                settings.command = READ_AUDIO_ADC_CONFIG;
                nvm_fds_eeprom_get(&settings);
			}
            else if(I2C_state() != NRFX_DRV_STATE_POWERED_ON)
            {
                I2C_init();
                I2S_init(config->channel, data_handler);
                retval = I2S_start(&m_buffer_rx[0][0], I2S_DATA_BLOCK_WORDS);
            }
            else if(TLV_init(config->channel, config->volume, config->gain, config->min6dB))
            {
                audioFFT_init();
                audio_app_nextState(AUDIO_SAMPLING);
                audio.blocksTransferred = 0;
                fftIsBusy = false;
            }
            break;
        }

        //-----------------------------------------------------------------------------------------------------------------------------------------------------------------
        case AUDIO_SAMPLING:

            // Check whether a block has been released by the I2S peripheral.
            if(fftIsBusy)
            {
                audio.blocksTransferred++;

                // Only process the I2S data of the I2S measurement result we'll actually use. When loop==true each sample is processed.
                if((audio.blocksTransferred < audio.blocksOffset) && (audio.blocksOffset != 0) && !audio.loop)
                {
                    fftIsBusy = false;
                    return;
                }

                #if 0
                    print_i2S_data(fftSamples, I2S_DATA_BLOCK_WORDS); // mp_block_to_check
                #endif

                #if 1
                    /* Convert the uint32_t array containing the samples of the Audio ADC to an float array with complex numbers. The real part will be placed on the even 
                     * indexes and the imaginary part will be set to 0 on all uneven indexes. This means that the complex input array is twice the size of the number of
                     * samples.
                     */
                    fft_generate_complexNumbersArray(m_fft_input_f32, fftSamples, I2S_DATA_BLOCK_WORDS);
                #else
                    audioFFT_generate_sine_complex(m_fft_input_f32, I2S_DATA_BLOCK_WORDS);
                #endif

                /* Use CFFT module to process the data.
                 * Ensure that the used module is equal to the number of complex number of samples.
                 */
                #if (I2S_DATA_BLOCK_WORDS != 1024)
                    #error("Number of complex samples does not match the arm_cfft_sR_f32_len1024")
                #endif
                arm_cfft_f32(&arm_cfft_sR_f32_len1024, m_fft_input_f32, 0, 1);

                // Calculate the magnitude at each bin using Complex Magnitude Module function.
                arm_cmplx_mag_f32(m_fft_input_f32, m_fft_output_f32, I2S_DATA_BLOCK_WORDS);

                fft_normalize(m_fft_output_f32, I2S_DATA_BLOCK_WORDS);
                
                // Print the results and draw FFT bin power chart.
                #if AUDIO_APP_LOG_ENABLED
                    print_output_data(m_fft_output_f32, FFT_OUTPUT_SIZE);
                    draw_fft_data(m_fft_output_f32, FFT_OUTPUT_SIZE, GRAPH_WINDOW_HEIGHT);
                #endif

                // Compress the data to the beep format for storage and transmission.
                m_fft_output_f32[0] = 0.0; // Remove the DC offset
                fft_create_result(  &fft_result,
                                    m_fft_output_f32, 
                                    settings.param.audio_config.fft_count,
                                    settings.param.audio_config.fft_start * 2,
                                    settings.param.audio_config.fft_stop  * 2,
                                    FFT_OUTPUT_SIZE);

                if(audio.callback != NULL)
                {
                    fft_result.param.meas_result.source = audio.source;
                    audio.callback(&fft_result.param.meas_result);
                }

                #if AUDIO_APP_LOG_ENABLED
                    NRF_LOG_INFO("Received block: %u", audio.blocksTransferred);
                #endif
                fftIsBusy = false;
            }

            // Sample the TLV with the I2S interface until enough samples have been taken or sample for as long as the loop boolean is set.
            if(audio.blocksTransferred >= audio.blocksOffset && !audio.loop)
            {
                I2S_stop();
                I2C_uninit();
                audioFFT_deinit();
                TLV_reset();
                audio_app_nextState(AUDIO_PROCESS);
            }
            break;

        //-----------------------------------------------------------------------------------------------------------------------------------------------------------------
        case AUDIO_PROCESS:
            if(1)
            {
                audio_app_nextState(AUDIO_STOP);
            }
            break;

        //-----------------------------------------------------------------------------------------------------------------------------------------------------------------
        case AUDIO_STOP:
            // Disable the power to the TLV Audio ADC
            if(powerApp_getEnabled(PWR_AUDIO))
			{
				powerApp_Enable(false, PWR_AUDIO);
			}
            else
            {
                audio_app_nextState(AUDIO_IDLE);
            }
            break;

        //-----------------------------------------------------------------------------------------------------------------------------------------------------------------
        default:
            break;
    }
    #endif
}


uint32_t audio_app_get_result(MEASUREMENT_RESULT_s * result)
{
	if(result == NULL)
	{
		return NRF_ERROR_INVALID_PARAM;
	}

    memset(result, 0, sizeof(MEASUREMENT_RESULT_s));
	memcpy(result, &fft_result.param.meas_result, sizeof(MEASUREMENT_RESULT_s));
    fft_result.param.meas_result.type = AUDIO_ADC;
    return NRF_SUCCESS;
}


uint32_t audio_app_start(const bool en, CONTROL_SOURCE source)
{
    #if !TLV_ENABLE
        return NRF_ERROR_INVALID_STATE;
    #else
        if(en)
        {
            if(audio.state == AUDIO_IDLE)
            {
                audio.source = source;
                audio_app_nextState(AUDIO_START);
                return NRF_SUCCESS;
            }
            else
            {
                return NRF_ERROR_INVALID_STATE;
            }
        }
        else
        {
            if(audio.state != AUDIO_IDLE)
            {
                audio_app_nextState(AUDIO_STOP);
                return NRF_SUCCESS;
            }
            else
            {
                return NRF_ERROR_INVALID_STATE;
            }
        }
    #endif
}

void audio_app_init(measurement_callback measurement_handler)
{
    audio_app_nextState(AUDIO_IDLE);
    audio.loop          = false; // For testing purposes set to true
    audio.callback      = measurement_handler;
    audio.blocksOffset  = BLOCKS_TO_TRANSFER;
    /*
		AIN_IN3LM       = 0,
		AIN_IN2LP       = 1,
		AIN_IN2RP       = 2, // default
		AIN_OFF         = 3,
     */
}