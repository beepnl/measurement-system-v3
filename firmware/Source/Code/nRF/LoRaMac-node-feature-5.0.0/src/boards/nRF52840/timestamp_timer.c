#include "timestamp_timer.h"
#include "app_timer.h"
#include "nrf_log.h"
#include "sdk_config.h"

const nrf_drv_timer_t TIMESTAMP_TIMER   = NRF_DRV_TIMER_INSTANCE(1);

#define SELECT_TIMER_MODULE(MODULE) ((MODULE == 0) ? &A_RX_TIMER : &B_RX_TIMER)

/**
 * @brief Handler for timer events.
 */
void timestamp_timer_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            #if 0
              volatile uint32_t timestamp_seconds = 0;

              // increment the number of seconds
              timestamp_seconds++;  
              NRF_LOG_INFO("Sec: %u", timestamp_seconds);
            #endif
            break;

        default:
            //Do nothing.
            break;
    }
}



/*
 */
void timestamp_init(void)
{
    uint32_t err_code;
    uint32_t time_ticks;

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    timer_cfg.frequency = NRF_TIMER_FREQ_1MHz;

    err_code = nrf_drv_timer_init(&TIMESTAMP_TIMER, &timer_cfg, timestamp_timer_event_handler);
    APP_ERROR_CHECK(err_code);

    time_ticks = nrf_drv_timer_ms_to_ticks(&TIMESTAMP_TIMER, 1000);

    nrf_drv_timer_extended_compare(&TIMESTAMP_TIMER, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

    nrf_drv_timer_enable(&TIMESTAMP_TIMER);
}



/*
 */
uint32_t get_timestamp_counter(void)
{
	#if NRF_LOG_HIGH_ACCURACY_TIMESTAMP
		return nrf_drv_timer_capture(&TIMESTAMP_TIMER, NRF_TIMER_CC_CHANNEL0);
	#else
		return app_timer_cnt_get();
	#endif
}

/*
 */
uint32_t timestamp_calculate_difference (uint32_t timestamp_cur, uint32_t timestamp_prev)
{
    uint32_t timstamp_delta;
    if(timestamp_prev > timestamp_cur)
    {
		// roll-over detected, substract the previous timestamp from uint32_max and add the current timestamp to calculate the delta time.
		timstamp_delta = (UINT32_MAX - timestamp_prev) + timestamp_cur;
    }
    else
    {
		timstamp_delta = timestamp_cur - timestamp_prev;
    }

    return timstamp_delta;
}

/*
 */
void rx_timer_init(const nrf_drv_timer_t * timer_instance, nrfx_timer_event_handler_t timer_instance_handler, void * p_context)
{
    uint32_t err_code;
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG; 
    timer_cfg.p_context = p_context;

    err_code = nrf_drv_timer_init(timer_instance, &timer_cfg, timer_instance_handler);
    APP_ERROR_CHECK(err_code);
}

/*
 */
void rx_timer_RX_window(const nrf_drv_timer_t * timer_instance, uint32_t rx1_time, uint32_t rx2_time)
{
    uint32_t err_code;
    uint32_t time_ticks;


    if(rx2_time < rx1_time)
    {
		time_ticks = nrf_drv_timer_ms_to_ticks(timer_instance, rx1_time);
		nrf_drv_timer_extended_compare(timer_instance, NRF_TIMER_CC_CHANNEL1, time_ticks, NRF_TIMER_SHORT_COMPARE1_STOP_MASK, true); 
    }
    else
    {
		time_ticks = nrf_drv_timer_ms_to_ticks(timer_instance, rx1_time);
		nrf_drv_timer_extended_compare(timer_instance, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE1_STOP_MASK, true);

		time_ticks = nrf_drv_timer_ms_to_ticks(timer_instance, rx2_time);
		nrf_drv_timer_extended_compare(timer_instance, NRF_TIMER_CC_CHANNEL1, time_ticks, NRF_TIMER_SHORT_COMPARE1_STOP_MASK, true);  
    }

    nrf_drv_timer_enable(timer_instance);
}

/*
 */
void rx_timer_disable(const nrf_drv_timer_t * timer_instance)
{
    nrfx_timer_disable(timer_instance);
}