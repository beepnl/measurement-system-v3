#ifndef __RNG_NRF_H__ 
#define __RNG_NRF_H__

    #include "nrf_drv_rng.h"


    void init_rng(void);

    uint8_t random_array_generate(uint8_t * p_buff, uint8_t size);

    uint16_t get_random_devNonce    (void);


#endif /* __RNG_NRF_H__ */