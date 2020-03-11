
#ifndef AUDIO_FFT
#define	AUDIO_FFT
	#include <stdint.h>
	#include <stdbool.h>

    #define FFT_LOG_ENABLED                     0
    #define GRAPH_WINDOW_HEIGHT                 20                              //!< Graph window height used in draw function.

    #define FPU_EXCEPTION_MASK                  0x0000009F                      //!< FPU exception mask used to clear exceptions in FPSCR register.
    #define FPU_FPSCR_REG_STACK_OFF             0x40                            //!< Offset of FPSCR register stacked during interrupt handling in FPU part stack.

    #define SIGNALS_RESOLUTION                  100.0f                          //!< Sine wave frequency and noise amplitude resolution. To count resolution as decimal places in number use this formula: resolution = 1/SIGNALS_RESOLUTION .
    #define SINE_WAVE_FREQ_MAX                  20000                           //!< Maximum frequency of generated sine wave.
    #define NOISE_AMPLITUDE                     1                               //!< Amplitude of generated noise added to signal.

    void    audioFFT_init                       (void);
    void    audioFFT_deinit                     (void);
    void    audioFFT_generate_sine              (uint32_t * p_input, uint16_t size, float sampling_freq, float sine_freq, float offset);
    void    audioFFT_generate_sine_complex      (float  * p_input, uint16_t size);
    void    fft_generate_complexNumbersArray    (float  * p_input, uint32_t * samples, uint16_t size);
    void    audioFFT_process                    (float * m_fft_output_f32, uint32_t * samples, uint16_t sampleSize);
    void    draw_fft_data                       (float * p_input_data, uint16_t data_size, uint16_t chart_height);
    void    fft_normalize                       (float * p_output, uint16_t output_size);


#endif	/* AUDIO_FFT */

