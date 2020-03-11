#ifndef TIMESTAMP_TIMER_H
#define TIMESTAMP_TIMER_H

  #include <stdbool.h>
  #include <stdint.h>
  #include "nrf.h"
  #include "nrf_drv_timer.h"

  typedef uint32_t timestamp_t;

  void      timestamp_init                  (void);  
  uint32_t  get_timestamp_counter           (void);
  uint32_t  timestamp_calculate_difference  (uint32_t timestamp_cur, uint32_t timestamp_prev);
//  void      timestamp_time_window           (uint32_t rx1_time, uint32_t rx2_time);

  void      rx_timer_init                   (const nrf_drv_timer_t * timer_instance, nrfx_timer_event_handler_t timer_instance_handler, void * p_context);
  void      rx_timer_RX_window              (const nrf_drv_timer_t * timer_instance, uint32_t rx1_time, uint32_t rx2_time);
  void      rx_timer_disable                (const nrf_drv_timer_t * timer_instance);

#endif
