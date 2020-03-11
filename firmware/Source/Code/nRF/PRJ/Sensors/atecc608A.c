#define NRF_LOG_MODULE_NAME ATECC_608A
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
#include "atecc608A.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_twi.h"
#include "boards.h"
#include "atecc608A.h"
#include "atca_device.h"
#include "power_app.h"
#include "atca_basic.h"
#include "atca_device.h"
#include "beep_protocol.h"
#include "nvm_fs.h"
#include "I2C.h"


void atecc_getID(void)
{
	BEEP_protocol_s prot;
	ATCA_STATUS status;

	memset(&prot, 0, sizeof(BEEP_protocol_s));

	// Enable power for the ATECC608A.
	powerApp_Enable(true, PWR_ATECC);

	nrf_delay_ms(10);
    
	cfg_ateccx08a_i2c_default.devtype = 0x60;
    status = atcab_init(&cfg_ateccx08a_i2c_default);
	if(status != ATCA_SUCCESS)
	{
        #if ATECC_LOG_ENABLED
            NRF_LOG_INFO("atcab_init error status: %ud/%04X", status, status);
        #endif
	}

	uint8_t i = 0;

	for(i=0 ; i < 3 ; i++)
	{
		status = atcab_read_serial_number(prot.param.atecc_id.ROM);
		if(status != ATCA_SUCCESS)
		{
            #if 1 //ATECC_LOG_ENABLED
                NRF_LOG_INFO("atcab_read_serial_number error status: %ud/%04X", status, status);
            #endif
		}
		else
		{
            #if ATECC_LOG_ENABLED
                NRF_LOG_INFO("atcab_read_serial_number:");
                NRF_LOG_HEXDUMP_INFO(&prot.param.atecc_id.ROM, 9);
            #endif

			// Save the ATECC608A ID
			prot.command = READ_ATECC_READ_ID;
			nvm_fds_eeprom_set(&prot);
			break;
		}
		NRF_LOG_FLUSH();
	}

    I2C_uninit();

	powerApp_Enable(false, PWR_ATECC);
}







