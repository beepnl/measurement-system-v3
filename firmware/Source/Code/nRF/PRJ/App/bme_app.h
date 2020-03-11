#ifndef BME_APP_H
#define	BME_APP_H

	#include <stdint.h>
	#include <stdbool.h>
    #include "beep_types.h"
    #include "beep_protocol.h"

	#define BME_APP_LOG_ENABLED 0

    typedef enum
    {
        BME_IDLE,
        BME_START,
        BME_READ_RESULT,
        BME_STOP,
    }BME_STATES;

    typedef struct
    {
        BME_STATES              state;
        measurement_callback    callback;
        CONTROL_SOURCE          source;
        uint32_t                timestampStateChanged;
		bool					waitDone;
    }BME_APPLICATIONs;

    uint32_t    bme_app_get_result(MEASUREMENT_RESULT_s * result);
    bool        bme_app_busy      (void);
    bool        bme_app_sleep     (void);
    void        bme_app_while     (void);
    uint32_t    bme_app_start     (const bool en, CONTROL_SOURCE source);
    void        bme_app_init      (measurement_callback measurement_handler);

#endif	/* BME_APP_H */

