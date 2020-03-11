#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#define NRF_LOG_MODULE_NAME FFT
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
#include "audioFFT.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "audio_app.h"
/*lint -save -e689 */ /* Apparent end of comment ignored */
#include "arm_const_structs.h"
/*lint -restore */



void audioFFT_generate_sine(uint32_t * p_input, uint16_t size, float sampling_freq, float sine_freq, float offset)
{
    uint32_t i;

    for (i = 0; i < (size - 1UL); i += 2)
    {
        p_input[i] = (uint32_t) (offset + 100.0 * sin(sine_freq * (2.0f * PI) * i / sampling_freq));
    }
}

void audioFFT_generate_sine_complex(float * p_input, uint16_t size)
{
    static float freq = 0.0;
    freq += 1000.0;
    if(freq >= 20000.0)
    {
        freq = 1000.0;
    }

    NRF_LOG_INFO("Freq:"NRF_LOG_FLOAT_MARKER "Hz", NRF_LOG_FLOAT(freq));
    NRF_LOG_FLUSH();

    uint32_t i;

    /* Remember that sample is represented as two next values in array. */
    uint32_t complex_idx = 0;

    #if FFT_LOG_ENABLED
        NRF_LOG_RAW_INFO("R = [");
        NRF_LOG_FLUSH();
    #endif

    for (i = 0; i < size; i++)
    {
        complex_idx = i * 2;

        // Real part.
        p_input[complex_idx + 0] = (1000.0 * sin(freq * (2.f * PI) * i / FFT_TEST_SAMPLE_FREQ_HZ)) + 8000.0;

        // Img part.
        p_input[complex_idx + 1] = 0;

        #if FFT_LOG_ENABLED
            if(i < (size - 1))
            {
                NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER", ", NRF_LOG_FLOAT(p_input[complex_idx + 0]));
            }
            else
            {
                NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER" ];\n",  NRF_LOG_FLOAT(p_input[complex_idx + 0]));
			
            }
            NRF_LOG_FLUSH();
        #endif
    }
}

/**
 * @brief Function for generating sine wave samples for FFT calculations.
 *
 * This function fill up output array with generated sine wave data with proper sampling frequency.
 * Must be executed before fft_process function.
 *
 * @param[in] p_input     Input array to fill with sine wave generated data.
 * @param[in] size        Input array size.
 * @param[in] sample_freq Sine wave sampling frequency.
 * @param[in] sine_freq   Sine wave frequency.
 * @param[in] add_noise   Flag for enable or disble adding noise for generated data.
 */
void fft_generate_complexNumbersArray(float  * p_input, uint32_t * samples, uint16_t size)
{
    uint32_t i;
    int16_t sample;

    /* Remember that sample is represented as two next values in array. */
    uint32_t complex_idx = 0;

    #if FFT_LOG_ENABLED
        NRF_LOG_RAW_INFO("R = [");
        NRF_LOG_FLUSH();
    #endif

    for (i = 0; i < size; i+=2)
    {
        complex_idx = i * 2;
        sample = (uint16_t) (samples[i] >> 0);

        // Real part.
        p_input[complex_idx + 0] = (float32_t) sample;

        // Img part.
        p_input[complex_idx + 1] = 0;


        complex_idx = (i+1) * 2;
        sample = (uint16_t) (samples[i] >> 16);

        // Real part.
        p_input[complex_idx + 0] = (float32_t) sample;

        // Img part.
        p_input[complex_idx + 1] = 0;

        #if FFT_LOG_ENABLED
            if(i < (size - 1))
            {
                NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER", ", NRF_LOG_FLOAT(p_input[complex_idx + 0]));
            }
            else
            {
                NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER" ];\n",  NRF_LOG_FLOAT(p_input[complex_idx + 0]));
            }
            NRF_LOG_FLUSH();
        #endif
    }
}

void fft_normalize(float * p_output, uint16_t output_size)
{
    uint16_t i;
    float32_t size = output_size;

    for(i=0; i<output_size; i++)
    {
        p_output[i] = (2.0 * p_output[i]) / size;
    }
}


/**
 * @brief Function for processing generated sine wave samples.
 * @param[in] p_input        Pointer to input data array with complex number samples in time domain.
 * @param[in] p_input_struct Pointer to cfft instance structure describing input data.
 * @param[out] p_output      Pointer to processed data (bins) array in frequency domain.
 * @param[in] output_size    Processed data array size.
 */
static void fft_process(float32_t * p_input, const arm_cfft_instance_f32 * p_input_struct, float32_t * p_output, uint16_t output_size)
{
    // Use CFFT module to process the data.
    arm_cfft_f32(p_input_struct, p_input, 0, 1);
    // Calculate the magnitude at each bin using Complex Magnitude Module function.
    arm_cmplx_mag_f32(p_input, p_output, output_size);

    fft_normalize(p_output, output_size);
}




/**
 * @brief FPU Interrupt handler. Clearing exception flag at the stack.
 *
 * Function clears exception flag in FPSCR register and at the stack. During interrupt handler
 * execution FPU registers might be copied to the stack (see lazy stacking option) and
 * it is necessary to clear data at the stack which will be recovered in the return from
 * interrupt handling.
 */
void FPU_IRQHandler(void)
{
    // Prepare pointer to stack address with pushed FPSCR register.
    uint32_t * fpscr = (uint32_t * )(FPU->FPCAR + FPU_FPSCR_REG_STACK_OFF);
    // Execute FPU instruction to activate lazy stacking.
    (void)__get_FPSCR();
    // Clear flags in stacked FPSCR register.
    *fpscr = *fpscr & ~(FPU_EXCEPTION_MASK);
}


/**
 * @brief Function for drawing line with given width.
 * @param[in] line_width Line width.
 */
static void draw_line(uint16_t line_width)
{
    uint32_t i;
    char     line[line_width + 1];

    for (i = 0; i < line_width; i++)
    {
        line[i] = '-';
    }
    line[line_width] = 0;
    NRF_LOG_RAW_INFO("%s\r\n", nrf_log_push(line));
}

/**
 * @brief Function for drawing line and processed data informations.
 * @param[in] input_sine_freq Input sine wave frequency.
 * @param[in] is_noisy        Flag if data is noisy.
 * @param[in] chart_width     Drawing chart height.
 */
static void draw_fft_header(float32_t input_sine_freq)
{
    NRF_LOG_RAW_INFO("fSample: %u Hz, res:%u Hz\r\n", (uint32_t)input_sine_freq, (uint32_t) FFT_TEST_SAMPLE_RES_HZ);
}

/**
 * @brief Function for drawing ASCII data processed by FFT function.
 * @param[in] p_input_data Pointer to FFT data array.
 * @param[in] data_size    FFT array size.
 * @param[in] chart_height Drawing chart height.
 */
void draw_fft_data(float * m_fft_output_f32, uint16_t data_size, uint16_t chart_height)
{
    uint32_t  graph_y, graph_x;
    float32_t curr_drawing_val;
    float32_t curr_percent;
    float32_t max_value = 0;
    uint32_t  max_val_index = 0;
    char      tmp_str[data_size + 1];

    // Search FFT max value in input array with an offset of 10 to ignore the DC part.
    uint32_t offset = 10;
    arm_max_f32(&m_fft_output_f32[offset], data_size-offset, &max_value, &max_val_index);
    max_val_index += offset;

    // Draw graph. Put space if number is less than currently processed, put '|' character otherwise.
    for (graph_y = chart_height; graph_y > 0; graph_y--)
    {
        curr_percent = ((graph_y - 1) / (chart_height * 1.f));
        curr_drawing_val = max_value * curr_percent;
        for (graph_x = 0; graph_x < data_size; graph_x++)
        {
            if (m_fft_output_f32[graph_x] > curr_drawing_val)
            {
                tmp_str[graph_x] = '|';
            } else
            {
                tmp_str[graph_x] = ' ';
            }
        }
        tmp_str[data_size] = 0;
        NRF_LOG_RAW_INFO("%s\r\n", NRF_LOG_PUSH(tmp_str));
        NRF_LOG_FLUSH();
    }

    draw_line(data_size);

    NRF_LOG_RAW_INFO("Max value: "NRF_LOG_FLOAT_MARKER" at [%u] = %u Hz\r\n\r\n", NRF_LOG_FLOAT(max_value), max_val_index, (max_val_index * FFT_TEST_SAMPLE_RES_HZ));
}

/**
 * @brief Function for application main entry.
 */
void audioFFT_init(void)
{
    // Enable FPU interrupt
    NVIC_SetPriority(FPU_IRQn, APP_IRQ_PRIORITY_LOWEST);
    NVIC_ClearPendingIRQ(FPU_IRQn);
    NVIC_EnableIRQ(FPU_IRQn);
}

void audioFFT_deinit(void)
{
    // Disable FPU interrupt
    NVIC_DisableIRQ(FPU_IRQn);
    NVIC_ClearPendingIRQ(FPU_IRQn);
}


