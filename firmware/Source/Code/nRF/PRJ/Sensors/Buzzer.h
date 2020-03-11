/*!
 * \file      Buzzer.h
 *
 * \brief     Buzzer implementation
 *
 */
#ifndef __BUZZER_H__
#define __BUZZER_H__

	#include <stdbool.h>
	#include <stdint.h>

	bool		BuzzerIsOn			(void);
	void		Buzzer_init			(const uint32_t interval);
    uint32_t	Buzzer_sound		(const uint8_t dutyCycle, const uint32_t freq, const uint16_t onTime, const uint16_t offtime, const uint16_t repeat);
    uint32_t	Buzzer_default_tune	(uint8_t TuneIndex);

#endif // __BUZZER_H__
