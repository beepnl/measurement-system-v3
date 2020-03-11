#ifndef MX_FLASH_APP_H
#define	MX_FLASH_APP_H

	#include <stdint.h>
	#include <stdbool.h>
    #include "beep_types.h"
    #include "beep_protocol.h"
    #include "ble_beep.h"

    #define MX_FLASH_APP_LOG_ENABLED    0
    #define MX_FLASH_LINE_LENGHT_MAX    (BLE_NUS_MAX_DATA_LEN * 12)

    #if (MX_FLASH_LINE_LENGHT_MAX % 4)
        #error("QSPI buffer size isn't a multiple of four")
    #endif

    typedef struct{
        uint8_t     data[MX_FLASH_LINE_LENGHT_MAX];
        uint16_t    lenght;
        uint8_t     counter;
    }FLASH_BUFFs;

    typedef enum
    {
        READ_OPEN_FILE,
        READ_ACCESS_FILE,
        READ_TRANSMIT_FILE,
        READ_CLOSE_FILE,
        READ_IDLE,
    }READ_DATA_STATEs;


    void        flashApp_cmd_handler        (app_flash_evt_type_t type, BEEP_protocol_s * prot);
    uint32_t    flash_size                  (void);
    bool        beep_flash_busy             (void);
    void        flash_newLineClear          (void);
    bool        flash_newLineStart          (BEEP_STATUS status);
    bool        flash_newLineAppendBeep     (BEEP_protocol_s * prot);
    bool        flash_newLineEnd            (void);
    bool        flash_Write_BeepProtocol    (BEEP_STATUS status, BEEP_protocol_s * param, uint8_t lenght);
    bool        flash_WriteStartUpLine      (void);
    void        beep_fileTransfer_init      (void);
    void        beep_fileTransfer_while     (void);


#endif	/* MX_FLASH_APP_H */

