#include "rng_nrf.h"
#include "nrf_drv_rng.h"


void init_rng(void)
{
	uint32_t err_code;

	err_code = nrf_drv_rng_init(NULL);
	APP_ERROR_CHECK(err_code);
}

/*@Brief: Get an array of random numbers
*/
uint8_t random_array_generate(uint8_t * p_buff, uint8_t size)
{
    uint32_t err_code;
    uint8_t  available;

    nrf_drv_rng_bytes_available(&available);
    uint8_t length = MIN(size, available);

    err_code = nrf_drv_rng_rand(p_buff, length);
    APP_ERROR_CHECK(err_code);

    return length;
}

/*@brief: Get a random Devnonce value
*/
uint16_t get_random_devNonce(void)
{
    uint8_t rng[2];
    nrf_drv_rng_block_rand(rng, 2);
    return (((uint16_t)rng[0]) << 8) | (((uint16_t)rng[1]) << 0);
}