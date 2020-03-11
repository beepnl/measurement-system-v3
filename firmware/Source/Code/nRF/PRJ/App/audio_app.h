#ifndef AUDIO_APP_H
#define	AUDIO_APP_H

	#include <stdint.h>
	#include <stdbool.h>
    #include "beep_types.h"
    #include "beep_protocol.h"

    #define AUDIO_APP_LOG_ENABLED               0
    #define BLOCKS_TO_TRANSFER                  2
    #define I2S_DATA_BLOCK_WORDS                1024UL
    #define FFT_COMPLEX_INPUT                   (I2S_DATA_BLOCK_WORDS * 2UL)
    #define FFT_OUTPUT_SIZE                     (I2S_DATA_BLOCK_WORDS / 2UL)
    #define MCLK_FREQ_HZ                        (32E6/31.0)                         //!< Frequency the Master Clock
    #define FFT_TEST_SAMPLE_FREQ_HZ             (MCLK_FREQ_HZ/128.0f)               //!< Frequency of complex input samples.
    #define FFT_TEST_SAMPLE_RES_HZ              (FFT_TEST_SAMPLE_FREQ_HZ/FFT_COMPLEX_INPUT) 

    typedef enum
    {
        AUDIO_IDLE,
        AUDIO_START,
        AUDIO_SAMPLING,
        AUDIO_PROCESS,
        AUDIO_STOP,
    }AUDIO_STATES;

    typedef struct
    {
        AUDIO_STATES            state;
        uint32_t                timestampStateChanged;
        uint16_t volatile       blocksTransferred;
        uint16_t                blocksOffset;       // The number of I2S results that are discarded before processing with the FFT
        bool                    loop;
        measurement_callback    callback;
        CONTROL_SOURCE          source;
    }AUDIO_APPLICATIONs;

    uint32_t    audio_app_get_result(MEASUREMENT_RESULT_s * result);
    bool        audio_app_busy      (void);
    bool        audio_app_sleep     (void);
    void        audio_app_while     (void);
    uint32_t    audio_app_start     (const bool en, CONTROL_SOURCE source);
    void        audio_app_init      (measurement_callback measurement_handler);

#endif	/* AUDIO_APP_H */

