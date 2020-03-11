
#ifndef I2C2_H
#define	I2C2_H
	#include <stdint.h>
	#include <stdbool.h>
	#include "sdk_errors.h"	
    #include "nrfx_common.h"


	ret_code_t	I2C2_write			(const uint8_t address, const uint8_t reg, uint8_t data);
	ret_code_t	I2C2_writeArray		(const uint8_t address, const uint8_t reg, uint8_t * data, uint16_t lenght);
	ret_code_t	I2C2_read			(const uint8_t address, const uint8_t reg, uint8_t *retval);
	ret_code_t	I2C2_readArray		(const uint8_t address, const uint8_t reg, uint8_t * data, uint16_t lenght);

    nrfx_drv_state_t I2C2_state              (void);
	uint32_t    I2C2_init					(void);
    void        I2C2_uninit					(void);
    void        I2C2_searchSlaves			(void);

#endif	/* I2C2_H */

