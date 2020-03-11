/*!
 * \file      SQ-min-200.h
 *
 * \brief     SQ-MIN-200 vibration sensor and SQ-SEN-645B implementation
 *
 */
#ifndef __SQMIN200_H__
#define __SQMIN200_H__

	#include <stdbool.h>
	#include <stdint.h>
	#include "beep_protocol.h"

	#define SQ_DEBUG 1


	typedef struct
	{
		bool		startUp;
		uint32_t	stateCurrently;
		uint32_t	stateNew;
		uint32_t	timeCount;
		uint32_t	timeThreshold;
		uint8_t		countVertical;
		uint8_t		countHorizontal;
	}SQ_SEN_645B;


    void SQ_init			(measurement_callback handler);
    void SQ_deinit			(void);
    bool SQ_getOrientation	(void);
	bool SQ_startup			(void);


#endif // __SQMIN200_H__
