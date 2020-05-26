/**
 * Copyright (c) 2016 - 2019, Nordic Semiconductor ASA
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
 * @defgroup fpu_fft_example_main main.c
 * @{
 * @ingroup fpu_fft_example
 * @brief Floating Point Number FFT Example Application main file.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "app_util_platform.h"
#include "bsp.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
/*lint -save -e689 */ /* Apparent end of comment ignored */
#include "arm_const_structs.h"
/*lint -restore */

#define GRAPH_WINDOW_HEIGHT              20                              //!< Graph window height used in draw function.

#define FPU_EXCEPTION_MASK               0x0000009F                      //!< FPU exception mask used to clear exceptions in FPSCR register.
#define FPU_FPSCR_REG_STACK_OFF          0x40                            //!< Offset of FPSCR register stacked during interrupt handling in FPU part stack.

// We want to use 44100 Hz sampling rate to reach 22050Hz band. 128 (64 pairs) samples are used
// in FFT calculation with result contains 64 bins (22050Hz/64bins -> ~344,5Hz per bin).
#define FFT_TEST_SAMPLE_FREQ_HZ          (4E6/96.0)                        //!< Frequency of complex input samples.
#define FFT_TEST_COMP_SAMPLES_LEN        1024                             //!< Complex numbers input data array size. Correspond to FFT calculation this number must be power of two starting from 2^5 (2^4 pairs) with maximum value 2^13 (2^12 pairs).
#define FFT_TEST_OUT_SAMPLES_LEN         (FFT_TEST_COMP_SAMPLES_LEN / 2) //!< Output array size.
#define FFT_TEST_SAMPLE_RES_HZ           (FFT_TEST_SAMPLE_FREQ_HZ/FFT_TEST_OUT_SAMPLES_LEN) 

#define SIGNALS_RESOLUTION               100.0f                          //!< Sine wave frequency and noise amplitude resolution. To count resolution as decimal places in number use this formula: resolution = 1/SIGNALS_RESOLUTION .
#define SINE_WAVE_FREQ_MAX               20000                           //!< Maximum frequency of generated sine wave.
#define NOISE_AMPLITUDE                  1                               //!< Amplitude of generated noise added to signal.

static uint32_t  m_ifft_flag             = 0;                            //!< Flag that selects forward (0) or inverse (1) transform.
static uint32_t  m_do_bit_reverse        = 1;                            //!< Flag that enables (1) or disables (0) bit reversal of output.
static uint16_t m_samples[FFT_TEST_OUT_SAMPLES_LEN] = {8784, 8544, 7904, 7280, 6960, 7168, 7792, 8432, 8432, 8736, 8544, 7936, 7296, 6976, 7136, 7728, 8384, 8736, 8528, 7952, 7264, 6912, 7088, 7680, 8320, 8704, 8528, 7968, 7312, 6944, 7056, 7616, 8352, 8704, 8592, 8048, 7360, 6944, 7072, 7552, 8224, 8656, 8560, 8048, 7360, 6944, 6992, 7536, 8192, 8608, 8560, 8048, 7392, 6896, 6944, 7472, 8096, 8576, 8576, 8080, 7424, 6944, 6944, 7424, 8128, 8608, 8624, 8192, 7472, 6960, 6944, 7392, 7392, 8048, 8560, 8592, 8192, 7536, 6960, 6912, 7344, 7984, 8512, 8624, 8224, 7536, 6992, 6880, 7264, 7952, 8512, 8608, 8272, 7552, 6960, 6816, 7200, 7872, 8464, 8640, 8272, 7600, 7008, 6848, 7168, 7840, 8464, 8624, 8304, 7664, 7024, 6848, 7136, 7760, 8400, 8624, 8288, 7680, 7040, 6800, 7104, 7760, 8368, 8608, 8336, 7696, 7056, 6800, 7040, 7680, 8336, 8592, 8368, 7728, 7104, 6784, 6784, 6992, 7600, 8256, 8576, 8368, 7792, 6960, 6624, 6832, 7424, 8064, 8368, 8256, 7632, 6944, 6608, 6800, 7360, 8032, 8384, 8208, 7664, 7008, 6608, 6752, 7296, 7984, 8400, 8288, 7776, 7088, 6688, 6752, 7248, 7968, 8416, 8352, 7840, 7168, 6720, 6736, 7280, 7936, 8400, 8384, 7840, 7168, 6720, 6752, 7264, 7936, 8432, 8384, 7888, 7216, 6736, 6720, 7200, 7888, 8400, 8416, 7984, 7312, 7312, 6832, 6768, 7168, 7840, 8416, 8464, 8016, 7344, 6816, 6736, 7168, 7808, 8352, 8528, 8096, 7424, 6880, 6752, 7136, 7792, 8352, 8464, 8144, 7440, 6864, 6672, 7056, 7728, 8304, 8480, 8160, 7520, 6928, 6704, 7056, 7696, 8320, 8496, 8160, 7536, 6928, 6752, 7008, 7664, 8256, 8480, 8224, 7616, 6976, 6720, 6992, 7648, 8288, 8544, 8288, 7680, 7024, 6768, 6992, 7584, 8240, 8528, 8304, 8304, 7712, 7056, 6752, 6960, 7520, 8192, 8512, 8320, 7760, 7120, 6752, 6912, 7520, 8176, 8528, 8368, 7824, 7136, 6768, 6896, 7472, 8144, 8512, 8400, 7872, 7168, 6784, 6880, 7424, 8112, 8496, 8432, 7904, 7216, 6816, 6928, 7440, 8064, 8496, 8464, 7920, 7280, 6800, 6864, 7360, 8048, 8528, 8496, 8032, 7344, 6896, 6912, 7360, 8048, 8512, 8544, 8064, 7392, 6912, 6864, 7328, 7968, 8464, 8464, 8560, 8096, 7392, 6912, 6848, 7248, 7936, 8480, 8544, 8112, 7456, 6928, 6800, 7200, 7872, 8464, 8576, 8176, 7536, 6960, 6832, 7184, 7888, 8416, 8608, 8240, 7568, 7040, 6832, 7168, 7840, 8416, 8592, 8256, 7632, 7056, 6864, 7184, 7808, 8432, 8640, 8336, 7712, 7088, 6864, 7184, 7776, 8384, 8656, 8352, 7744, 7104, 6816, 7072, 7680, 8320, 8608, 8368, 7760, 6736, 6592, 6944, 7552, 7552, 8112, 8320, 8000, 7376, 6768, 6560, 6896, 7504, 8096, 8336, 8032, 7376, 6784, 6592, 6816, 7440, 8064, 8320, 8000, 7392, 6816, 6560, 6784, 7408, 8032, 8288, 8080, 7424, 6800, 6496, 6704, 7312, 7936, 8224, 8064, 7456, 6816, 6528, 6720, 7280, 7904, 8272, 8080, 7536, 6864, 6480, 6688, 7216, 7872, 8208, 8048, 7520, 6880, 6496, 6608, 7152, 7824, 8176, 8096, 7536, 6896, 6512, 6592, 6592, 7104, 7760, 8176, 8096, 7600, 6944, 6528, 6560, 7072, 7760, 8160, 8128, 7600, 6976, 6512, 6544, 7008, 7696, 8160, 8128, 7632, 6992, 6528, 6496, 6976, 7632, 8080, 8144, 7696, 7008, 6560, 6480, 6944, 7584, 8064, 8128, 7696, 7056, 6528, 6464, 6928, 7568, 8096, 8160, 7744, 7072, 6576, 6480, 6864, 7504, 8016, 8128, 7776, 7104,  6560 };    
static float32_t m_fft_input_f32[FFT_TEST_COMP_SAMPLES_LEN] = { 0 };           //!< FFT input array. Time domain.
static float32_t m_fft_output_f32[FFT_TEST_OUT_SAMPLES_LEN];             //!< FFT output data. Frequency domain.

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
static void fft_generate_samples(float32_t  * p_input,
                                 uint16_t   * samples,
                                 uint16_t    size)
{
    uint32_t i;

    /* Remember that sample is represented as two next values in array. */
    uint32_t sample_idx = 0;

    if (2 > size)
    {
        return;
    }

    for (i = 0; i < (size - 1UL); i += 2) {
        sample_idx = i / 2;
        // Real part.
        p_input[i] = ((float32_t) samples[sample_idx]); // / 100.0;

        // Img part.
        p_input[i + 1] = 0;
    }
}

/**
 * @brief Function for processing generated sine wave samples.
 * @param[in] p_input        Pointer to input data array with complex number samples in time domain.
 * @param[in] p_input_struct Pointer to cfft instance structure describing input data.
 * @param[out] p_output      Pointer to processed data (bins) array in frequency domain.
 * @param[in] output_size    Processed data array size.
 */
static void fft_process(float32_t *                   p_input,
                        const arm_cfft_instance_f32 * p_input_struct,
                        float32_t *                   p_output,
                        uint16_t                      output_size)
{
    // Use CFFT module to process the data.
    arm_cfft_f32(p_input_struct, p_input, m_ifft_flag, m_do_bit_reverse);
    // Calculate the magnitude at each bin using Complex Magnitude Module function.
    arm_cmplx_mag_f32(p_input, p_output, output_size);
}

static void fft_normalize(float32_t * p_output, uint16_t output_size)
{
    uint16_t i;
    float32_t size = output_size;

    for(i=0; i<output_size; i++)
    {
        p_output[i] = p_output[i] / size;
    }
}

#ifdef FPU_INTERRUPT_MODE
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
#endif

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
static void draw_fft_data(float32_t * p_input_data, uint16_t data_size, uint16_t chart_height)
{
    uint32_t  graph_y, graph_x;
    float32_t curr_drawing_val;
    float32_t curr_percent;
    float32_t max_value;
    uint32_t  max_val_index;
    char      tmp_str[data_size + 1];

    // Search FFT max value in input array.
    arm_max_f32(p_input_data, data_size, &max_value, &max_val_index);

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

    arm_max_f32(&p_input_data[1], data_size-1, &max_value, &max_val_index);
    max_val_index += 1;
    NRF_LOG_RAW_INFO("Max value: %u at [%u] = %u Hz\r\n", max_value, max_val_index, (max_val_index * FFT_TEST_SAMPLE_RES_HZ));
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    uint32_t  err_code;
    bool      noise = false;
    float32_t sine_freq = FFT_TEST_SAMPLE_FREQ_HZ;

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    // Welcome message.
    NRF_LOG_INFO("FPU FFT example started.");
    NRF_LOG_RAW_INFO("This is FPU usage example with FFT calculation and drawing.\r\n");

#ifdef FPU_INTERRUPT_MODE
    // Enable FPU interrupt
    NVIC_SetPriority(FPU_IRQn, APP_IRQ_PRIORITY_LOWEST);
    NVIC_ClearPendingIRQ(FPU_IRQn);
    NVIC_EnableIRQ(FPU_IRQn);
#endif

    // Enter main loop.
    for (;;)
    {
        // Process generated data. 64 pairs of complex data (real, img). It is important to use
        // proper arm_cfft_sR_f32 structure associated with input/output data length.
        // For example:
        //  - 128 numbers in input array (64 complex pairs of samples) -> 64 output bins power data -> &arm_cfft_sR_f32_len64.
        //  - 256 numbers in input array (128 complex pairs of samples) -> 128 output bins power data -> &arm_cfft_sR_f32_len128.
        fft_generate_samples(m_fft_input_f32, m_samples, FFT_TEST_COMP_SAMPLES_LEN);
        fft_process(m_fft_input_f32, &arm_cfft_sR_f32_len512, m_fft_output_f32, FFT_TEST_OUT_SAMPLES_LEN);
        fft_normalize(m_fft_output_f32, FFT_TEST_OUT_SAMPLES_LEN);

        // Draw FFT bin power chart.
        draw_fft_header(sine_freq);
        draw_fft_data(m_fft_output_f32, FFT_TEST_OUT_SAMPLES_LEN / 2, GRAPH_WINDOW_HEIGHT);

        NRF_LOG_FLUSH();

#ifndef FPU_INTERRUPT_MODE
        /* Clear FPSCR register and clear pending FPU interrupts. This code is base on
         * nRF5x_release_notes.txt in documentation folder. It is necessary part of code when
         * application using power saving mode and after handling FPU errors in polling mode.
         */
        __set_FPSCR(__get_FPSCR() & ~(FPU_EXCEPTION_MASK));
        (void) __get_FPSCR();
        NVIC_ClearPendingIRQ(FPU_IRQn);
#endif
        nrf_delay_ms(1000);
    }
}

/**
 * @}
 */
