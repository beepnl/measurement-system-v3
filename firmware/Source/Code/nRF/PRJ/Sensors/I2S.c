#define NRF_LOG_MODULE_NAME I2S_TLV
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
#include "I2S.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "boards.h"



uint32_t I2S_start(uint32_t * buf, uint16_t buffer_size)
{
    uint32_t err_code;
    nrf_drv_i2s_buffers_t const initial_buffers = {
        .p_tx_buffer = NULL,
        .p_rx_buffer = buf,
    };
    err_code = nrf_drv_i2s_start(&initial_buffers, buffer_size, 0);
    APP_ERROR_CHECK(err_code);
}

void I2S_stop(void)
{
    nrf_drv_i2s_stop();
    nrf_drv_i2s_uninit();
}


void I2S_init(AUDIO_INPUTe channel, nrf_drv_i2s_data_handler_t handler)
{
	ret_code_t err_code = NRF_SUCCESS;
	nrf_drv_i2s_config_t config = NRF_DRV_I2S_DEFAULT_CONFIG;

	config.sdout_pin	= NRFX_I2S_PIN_NOT_USED;
    config.sdin_pin		= I2S_DOUT;
	config.lrck_pin		= I2S_LRCLK;
    config.sck_pin		= I2S_BCLK;
    config.mck_pin		= I2S_MCLK;
    config.mck_setup	= NRF_I2S_MCK_32MDIV31,     ///< 32 MHz / 31 = 1.0322581 MHz.
    config.ratio		= NRF_I2S_RATIO_128X;
    config.sample_width	= I2S_CONFIG_SWIDTH_SWIDTH_16Bit;
	config.format		= NRF_I2S_FORMAT_I2S;
	config.alignment	= NRF_I2S_ALIGN_RIGHT;

	switch(channel)
    {
        case AIN_IN3LM:      // 0, Left-PGA
        case AIN_IN2LP:      // 1, Left-PGA
            config.channels = NRF_I2S_CHANNELS_LEFT;
            break;

        // Default right channel
        default:
        case AIN_IN2RP:      // 2, default  Right-PGA
            config.channels = NRF_I2S_CHANNELS_RIGHT;
            break;
    }
    err_code = nrf_drv_i2s_init(&config, handler);
    APP_ERROR_CHECK(err_code);
}


void print_rx_data(uint32_t const * p_block, uint16_t size)
{
    // [each data word contains two 16-bit samples]
	uint8_t m_zero_samples_to_ignore = 0;	
	int32_t lsum = 0, lavg = 0, lmin = UINT32_MAX, lmax = 0;
	int32_t rsum = 0, ravg = 0, rmin = UINT32_MAX, rmax = 0;
    uint16_t i;

	#if PRINT_SAMPLES
		NRF_LOG_RAW_INFO("R = [");
	#endif

    for (i = 0; i < size; ++i)
    {
        uint32_t const * p_word = &p_block[i];
        int16_t actual_sample_l = (uint16_t)(p_block[i] >> 4);
        int16_t actual_sample_r = (uint16_t)(p_block[i] >> 4);

		// Left channel
        lsum += actual_sample_l;
		if(actual_sample_l < lmin)
		{
			lmin = actual_sample_l;
		}
		if(actual_sample_l > lmax)
		{
			lmax = actual_sample_l;
		}

        // Right channel
        rsum += actual_sample_r;
		if(actual_sample_r < rmin)
		{
			rmin = actual_sample_r;
		}
		if(actual_sample_r > rmax)
		{
			rmax = actual_sample_r;
		}
		#if PRINT_SAMPLES
		if(i < (size - 1))
		{
			NRF_LOG_RAW_INFO("%i, ", actual_sample_r);
			NRF_LOG_FLUSH();
		}
		else
		{
			NRF_LOG_RAW_INFO(" %i ];\n", actual_sample_r);
			NRF_LOG_FLUSH();
		}
		#endif

    }

    lavg = lsum / size;
    ravg = rsum / size;

#if !PRINT_SAMPLES
    NRF_LOG_INFO("%02uL - average: %i, min: %i, max: %i,", m_blocks_transferred, lavg, lmin, lmax);
    NRF_LOG_INFO("%02uR - average: %i, min: %i, max: %i,", m_blocks_transferred, ravg, rmin, rmax);
    NRF_LOG_FLUSH();
#endif
	
}










