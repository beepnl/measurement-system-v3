
#ifndef I2C_H
#define	I2C_H
	#include <stdint.h>
	#include <stdbool.h>
	#include "sdk_errors.h"	
    #include "nrfx_common.h"


	ret_code_t	I2C_write			(const uint8_t address, const uint8_t reg, uint8_t data);
	ret_code_t	I2C_writeArray		(const uint8_t address, uint8_t * data, uint8_t lenght);
	ret_code_t	I2C_read			(const uint8_t address, const uint8_t reg, uint8_t *retval);
	ret_code_t	I2C_readArray		(const uint8_t address, const uint8_t reg, uint8_t * data, uint8_t lenght);

    nrfx_drv_state_t I2C_state              (void);
	uint32_t    I2C_init					(void);
    void        I2C_uninit					(void);
    void        I2C_searchSlaves			(void);

#endif	/* I2C_H */

