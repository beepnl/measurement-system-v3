
#ifndef I2S_H
#define	I2S_H
	#include <stdint.h>
	#include <stdbool.h>
    #include "nrf_drv_i2s.h"
    #include "beep_types.h"

    #define PRINT_SAMPLES 1

	bool        I2S_checkRX     (void);
    void        I2S_init        (AUDIO_INPUTe channel, nrf_drv_i2s_data_handler_t handler);
    uint32_t    I2S_start       (uint32_t * buf, uint16_t buffer_size);
    void        I2S_stop        (void);
    void        print_rx_data   (uint32_t const * p_block, uint16_t size);

#endif	/* I2S_H */

