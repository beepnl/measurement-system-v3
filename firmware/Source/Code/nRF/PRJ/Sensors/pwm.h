/*!
 * \file      pwm.h
 *
 * \brief     PWM for the buzzer.
 *
 */
#ifndef __PWM_H__
#define __PWM_H__

	#include <stdbool.h>
	#include <stdint.h>

    #define PWM_LOG_EN  0

	void PWM_event(uint8_t dutycycle, uint16_t frequency, uint16_t nRepeat);
    void PWM_shutdown(void);

#endif // __PWM_H__
