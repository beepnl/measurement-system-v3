#define NRF_LOG_MODULE_NAME MX_APP
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "mx_flash_app.h"
#include "mx_flash_fs.h"
#include "nrf_log_ctrl.h"
#include "nrf.h"
#include "nvm_fs.h"
#include "ble_setup.h"
#include "app_timer.h"
#include "app_util.h"
#include "main.h"
#include "gpio-board.h"
#include "nrf_delay.h"



struct LINE_s
{
    bool            claimed;
    uint8_t         index;
    uint8_t         dataBytes;
    uint8_t         buff[BLE_NUS_MAX_DATA_LEN];
}line;

app_flash_evt_type_t  whileCMD       = UINT8_MAX;
CONTROL_SOURCE        whileCMDsource = INTERNAL_SOURCE;
FLASH_BUFFs         buff;
volatile bool       readNewBufferData = false;
volatile bool       clientDisconnected = false;
volatile bool       TXnotificationEnabled = false;    
volatile bool       busy = false;
volatile bool       erasing = false;
char                fileName[] = "BEEPlog.txt";
uint16_t            bufferCounter;
uint32_t			timestamp;
FIL                 fp;
READ_DATA_STATEs    readState = READ_IDLE;
uint32_t            bytesTransferred;
uint32_t            startTimestamp;
uint8_t				eraseType;
FSIZE_t             read_offset;
UINT                bytesRead = 0;




void flash_newLineClear(void)
{
    memset(line.buff, 0, BLE_NUS_MAX_DATA_LEN);
    line.index   = 0;
    line.claimed = false;
}


bool flash_newLineStart(BEEP_STATUS status)
{
    if(line.claimed)
    {
        return false;
    }

    flash_newLineClear();
    line.claimed    = true;
    line.buff[0]    = (uint8_t) status;
    line.buff[1]    = 0; // data bytes
    line.index      = 2;
    line.dataBytes  = 0;
    return true;
}


bool flash_newLineAppendBeep(BEEP_protocol_s * prot)
{
    uint32_t ret;
    

    if(!line.claimed && line.index < BLE_NUS_MAX_DATA_LEN)
    {
        return false;
    }

    #if 1 // 1= binair, 0 = ASCII

        ret = beep_protocol_encode(true, prot, &line.buff[line.index], &line.dataBytes, BLE_NUS_MAX_DATA_LEN);
        if(ret != NRF_SUCCESS)
        {
            return false;
        }

    #else 
        // Encode the data according to the protocol
        uint8_t i, index = 0;
        uint8_t tempHex[BLE_NUS_MAX_DATA_LEN] = {0};

        ret = beep_protocol_encode(true, prot, tempHex, &index, BLE_NUS_MAX_DATA_LEN);
        if(ret != NRF_SUCCESS)
        {
            return false;
        }

        // Add the data in ASCII hex to the storage array
        for(i=0; i<index; i++)
        {
            line.index += uint8_ASCIIencode(tempHex[i], &line.buff[line.index], &line.dataBytes);
        }
    #endif

    return true;
}


bool flash_newLineEnd(void)
{
    UINT bytesWritten;
    if(!line.claimed)
    {
        return false;
    }

	if(whileCMD != UINT8_MAX)
	{
		line.claimed = false;
		return false;
	}

    // Add the number of data bytes in the message to the array on index[1]
    line.buff[1] = line.dataBytes;
    line.index  += line.dataBytes;

    // Add the line end character.
    line.buff[line.index++] = '\n';

    // Write the data to the Flash
    bytesWritten = fatfs_write(line.buff, (UINT) line.index);

    // check if all bytes have been written
    if(bytesWritten != line.index)
    {
        NRF_LOG_INFO("The number of bytes written to the flash is NOK: Line lenght:%u, Written: %u", line.index, bytesWritten);
        line.claimed = false;
        return false;
    }
    #if MX_FLASH_DEBUG
        NRF_LOG_INFO("The number of bytes written to the flash is: %u, %u databytes", bytesWritten, line.dataBytes);
        NRF_LOG_HEXDUMP_INFO(line.buff, line.index);
        NRF_LOG_FLUSH();
        nrf_delay_ms(10);
    #endif

    // Release the line buffer. Will be cleared upon claiming.
    line.claimed = false;
    return true;
}


bool flash_WriteStartUpLine(void)
{
    bool retBool;
    BEEP_protocol_s prot[6];
    BEEP_CID cmds[6] = {READ_FIRMWARE_VERSION, READ_HARDWARE_VERSION, READ_ATECC_READ_ID, READ_BOOT_COUNT, READ_DS18B20_STATE, READ_APPLICATION_CONFIG};
    uint8_t i;

    // Retrieve all command parameters from the EEPROM
    for(i=0; i<6; i++)
    {
        memset(&prot[i], 0, sizeof(BEEP_protocol_s));
        prot[i].command = cmds[i];
        nvm_fds_eeprom_get(&prot[i]);
    }

    return flash_Write_BeepProtocol(BEEP_SENSOR_ON, prot, 6);
}



FRESULT flash_info(FILINFO * fno)
{
    FRESULT retVal;

    if(fno == NULL)
    {
        return FR_INVALID_PARAMETER;
    }
    
    retVal = f_stat(fileName, fno);
    return retVal;
}


uint32_t flash_size(void)
{
    FRESULT retVal;
    FILINFO stat;
    
    retVal = f_stat(fileName, &stat);
    return stat.fsize;
}




bool flash_Write_BeepProtocol(BEEP_STATUS status, BEEP_protocol_s * param, uint8_t lenght)
{
    bool retBool;
    uint8_t i;

    if(busy){
        return false;
    }

    // Initialize the filesystem and the QSPI interface. Disable them on error or on completion.
    fatfs_init();

    if(!flash_newLineStart(status))
    {
        MXflash_FSdeinit();
        return false;
    }

    // Retrieve all command parameters from the EEPROM and add them to the newLine 
    for(i=0; i<lenght; i++)
    {
        retBool = flash_newLineAppendBeep(&param[i]);
    }

    flash_newLineEnd();

    MXflash_FSdeinit();
    return true;
}


void flashApp_cmd_handler(app_flash_evt_type_t type, BEEP_protocol_s * prot)
{
    uint32_t err;
    uint32_t retVal = NRF_SUCCESS;

	if(whileCMD != UINT8_MAX)
	{
		return;
	}

    switch(type)
    {
        //--------------------------------------------------------------------------------------------------------------------------------------------------------------
        case APP_FLASH_READ:
            #if MX_FLASH_APP_LOG_ENABLED
                NRF_LOG_INFO("READ_START Handler");  
            #endif
            whileCMD        = APP_FLASH_READ;
            whileCMDsource  = prot->source;
            read_offset     = prot->param.size;
            break;

        //--------------------------------------------------------------------------------------------------------------------------------------------------------------
        case APP_FLASH_ERASE:
            #if MX_FLASH_APP_LOG_ENABLED
                NRF_LOG_INFO("ERASE_FS");
            #endif
            whileCMDsource  = prot->source;
            whileCMD        = APP_FLASH_ERASE;
            eraseType		= prot->param.status.statusflag;
            erasing         = false;
            break;

        //--------------------------------------------------------------------------------------------------------------------------------------------------------------
        case APP_FLASH_SIZE_FLASH:
            whileCMDsource  = prot->source;
            whileCMD        = APP_FLASH_SIZE_FLASH;
            break;

        //--------------------------------------------------------------------------------------------------------------------------------------------------------------
        case APP_TX_RDY_FLASH:
            #if MX_FLASH_APP_LOG_ENABLED
                NRF_LOG_INFO("APP_TX_RDY_FLASH, bytes read: %u, transferred: %u", bytesRead, bytesTransferred);
            #endif
            readNewBufferData = true;
            break;

        //--------------------------------------------------------------------------------------------------------------------------------------------------------------
        case APP_FLASH_COMM_CONNECTED:
            #if MX_FLASH_APP_LOG_ENABLED
                NRF_LOG_INFO("APP_FLASH_COMM_CONNECTED");
            #endif
            clientDisconnected = false;
            break;

        //--------------------------------------------------------------------------------------------------------------------------------------------------------------
        case APP_FLASH_COMM_DISCONNECTED:
            #if MX_FLASH_APP_LOG_ENABLED
                NRF_LOG_INFO("APP_FLASH_COMM_DISCONNECTED");
            #endif
            clientDisconnected = true;
            break;

        //--------------------------------------------------------------------------------------------------------------------------------------------------------------
        case APP_FLASH_COMM_STARTED:
            TXnotificationEnabled = true;
            #if MX_FLASH_APP_LOG_ENABLED
                NRF_LOG_INFO("APP_FLASH_COMM_STARTED");
            #endif
            break;

        //--------------------------------------------------------------------------------------------------------------------------------------------------------------
        case APP_FLASH_COMM_STOPPED:
            TXnotificationEnabled = false;
            #if MX_FLASH_APP_LOG_ENABLED
                NRF_LOG_INFO("APP_FLASH_COMM_STOPPED");
            #endif
            break;

        //--------------------------------------------------------------------------------------------------------------------------------------------------------------
        default:
            break;
    }
}


bool beep_flash_busy(void)
{
    return (readState == READ_ACCESS_FILE) ? true : false;
}



static void beep_fileReadTransferStatemachine(void)
{
    FRESULT retfs;
    uint32_t retNRF;

    switch(readState)
    {
            
        //########################################################################################################################################################################################
        case READ_OPEN_FILE:
        {
            // Check if the client has been disconnected
            if(clientDisconnected || !TXnotificationEnabled)
            {
                #if MX_FLASH_APP_LOG_ENABLED    
                    NRF_LOG_INFO("Error: client disconnected: %u, TX notifications: %u, offset: %u", (uint32_t)clientDisconnected, (uint32_t)TXnotificationEnabled);
                    NRF_LOG_FLUSH();
                #endif
                clientDisconnected = false;
                readState = READ_CLOSE_FILE;
                return;
            }
            busy                = true;
            bytesTransferred    = 0;
            bufferCounter       = 0;
            startTimestamp      = app_timer_cnt_get();

            // Initialize the filesystem and the QSPI interface. Disable them on error or on completion.
            fatfs_init();
            retfs = fatfs_open(&fp, fileName, read_offset);
            if(retfs != FR_OK)
            {
                #if MX_FLASH_APP_LOG_ENABLED    
                    NRF_LOG_INFO("Opening Log failed: 0x%04X/%u, offset: %u", retfs, retfs, read_offset);
                    NRF_LOG_FLUSH();
                #endif
                
                readState = READ_CLOSE_FILE;
                sendResponse(whileCMDsource, READ_MX_FLASH, NRF_ERROR_INVALID_LENGTH);
                break;
            }

            readNewBufferData   = true;
            readState           = READ_ACCESS_FILE;
            
            #if MX_FLASH_APP_LOG_ENABLED    
                NRF_LOG_INFO("new state: READ_ACCESS_FILE: 0x%04X/%u, offset: %u", retfs, retfs, (uint32_t)read_offset);
                NRF_LOG_FLUSH();
            #endif
            break;
        }

        //########################################################################################################################################################################################
        case READ_ACCESS_FILE:
        {
            
            UINT size = MX_FLASH_LINE_LENGHT_MAX;

            // Check if the client has been disconnected
            if(clientDisconnected || !TXnotificationEnabled)
            {
                #if MX_FLASH_APP_LOG_ENABLED    
                    NRF_LOG_INFO("new state: READ_ACCESS_FILE: 0x%04X/%u, offset: %u", retfs, retfs, (uint32_t)read_offset);
                    NRF_LOG_FLUSH();
                #endif
                clientDisconnected = false;
                readState = READ_CLOSE_FILE;
                return;
            }

            // If no buffer needs to be read, don't go further.
            if(!readNewBufferData)
            {
                return;
            }

            retfs = fatfs_readBuffer(&fp, buff.data, size, &bytesRead);
            buff.lenght = bytesRead;
            bytesTransferred += bytesRead;

            if(retfs != FR_OK)
            {
                #if MX_FLASH_APP_LOG_ENABLED
                    NRF_LOG_INFO("readBuffer error: 0x%04X/%u, bufsize: %u, read: %u", retfs, retfs, size, bytesRead);
                #endif
                readNewBufferData = false;
                readState = READ_CLOSE_FILE;
            }
            else
            {
                // Transmit the newly read data.
                readNewBufferData = false;

                // When no more data is available close the file
                if(bytesRead == 0)
                {
                    readState = READ_CLOSE_FILE;
                }
                else
                {
                    readState = READ_TRANSMIT_FILE;
                }
            }
            break;
        }
        
        //########################################################################################################################################################################################
        case READ_TRANSMIT_FILE:
        {
            // Check if TX notifications have been enabled, otherwise stop the statemachine
            if(!TXnotificationEnabled)
            {
                readState = READ_CLOSE_FILE;
            }

            retNRF = ble_TXdata_send(&bufferCounter, buff.data, buff.lenght);
            #if MX_FLASH_APP_LOG_ENABLED
                NRF_LOG_INFO("ble_bus_data_send[%u] TX err: 0x%04X/%u", bufferCounter, retNRF, retNRF);
            #endif

            if(retNRF == NRF_SUCCESS || retNRF == NRF_ERROR_RESOURCES)
            {
                readState = READ_ACCESS_FILE;
            }
            else if(retNRF == NRF_ERROR_BUSY)
            {
                // Wait for the BLE stack to finish transmission or to return a different error on the next transmission attempt.
                return;
            }
            else
            {
                // Shutdown the read operation
                sendResponse(whileCMDsource, READ_MX_FLASH, NRF_SUCCESS);
                readState = READ_CLOSE_FILE;
            }
            break;
        }

        //########################################################################################################################################################################################
        case READ_CLOSE_FILE:
        {
            uint32_t time_ms = app_timer_time_since_start_ms(startTimestamp);
            
            retfs = fatfs_close(&fp);
            readState = READ_IDLE;

            #if MX_FLASH_APP_LOG_ENABLED    
                float bytesPerSecond = (time_ms == 0) ? 0.0 : (((float)bytesTransferred * 1000.0) / (float)time_ms);
                NRF_LOG_INFO("Bytes transferred: %u, Time: %u ms, average: "NRF_LOG_FLOAT_MARKER " bytes per second", bytesTransferred, time_ms, NRF_LOG_FLOAT(bytesPerSecond));
                NRF_LOG_INFO("closed file: 0x%04X/%u, new state: READ_IDLE", retfs, retfs);
                NRF_LOG_FLUSH();
            #endif

            MXflash_FSdeinit();

            busy = false;
            break;
        }

        //########################################################################################################################################################################################
        default:
        case READ_IDLE:
            break;
    }
}


/*
 * Statemachine handler for the filetransfer service.
 */
void beep_fileTransfer_while(void)
{
    uint32_t ret;
    // When a valid command is set from the BLE RX handler, excute it in the while loop
    if(whileCMD != UINT8_MAX)
    {
        switch(whileCMD)
        {
            case APP_FLASH_READ:
            {
                uint8_t i;

                #if MX_FLASH_APP_LOG_ENABLED   
                    NRF_LOG_INFO("READ_START while, size: %u, offset: %u", flash_size(), (uint32_t)read_offset);
                #endif
                
                // Check the origin of the read command, LoRaWAN is not supported.
                if(whileCMDsource == BLE_SOURCE)
                {
                    if(readState == READ_IDLE)
                    {
                        readState = READ_OPEN_FILE;
                    }
                }
                else
                {
                    // When send from any other source then BLE, reply invalid parameter.
                    sendResponse(whileCMDsource, READ_MX_FLASH, NRF_ERROR_INVALID_PARAM);
                }
                whileCMD = UINT8_MAX;
                break;
            }

            case APP_FLASH_ERASE:
            {
				// Handle the MX flash erase of the fatfs erase here.
				if(eraseType)
				{
					if(!erasing)
					{
						#if MX_FLASH_APP_LOG_ENABLED
							NRF_LOG_INFO("ERASE_FS Clear MX");
						#endif

						busy = true;
						FRESULT err_fs = 0;
						flash_ReleasePowerDown();
						flash_sendInstructionCode(0x06); // Write Enable command
						flash_sendInstructionCode(0xC7); // Erase 
						timestamp = app_timer_cnt_get();;
						erasing = true;
						NRF_LOG_INFO("Flash erase MX started");
					}
					else
					{
						uint16_t status = flash_ReadStatusRegister();
						const uint32_t timeSinceStart = app_timer_time_since_start_ms(timestamp);

						// Wait until 250 seconds have passed or the Write In Progress Bit is cleared
						if((status & 0x01) && timeSinceStart < (250 * 1000))
						{
							break;
						}

						if(timeSinceStart < (250 * 1000))
						{
							NRF_LOG_INFO("Flash erase MX completed");
						}
						else
						{
							NRF_LOG_INFO("Flash erase MX timed-out");
						}

						//Init the MX25R6435F Flash IC
						MXflash_FSinit();

                        flash_WriteStartUpLine();

						// Return a status code when erasing is completed.
						sendResponse(whileCMDsource, ERASE_MX_FLASH, (timeSinceStart < (250 * 1000)) ? NRF_SUCCESS : NRF_ERROR_TIMEOUT);
						busy = false;
						whileCMD = UINT8_MAX;
					}
				}
				else
				{
					#if MX_FLASH_APP_LOG_ENABLED
						NRF_LOG_INFO("ERASE_FS Clear FATfs");
					#endif

					FRESULT err_fs;
					busy		= true;
					err_fs		= fatfs_clear();

					// Return a status code when erasing is completed.
					sendResponse(whileCMDsource, ERASE_MX_FLASH, err_fs);

					flash_WriteStartUpLine();
					busy = false;
					whileCMD = UINT8_MAX;
				}
                break;
            }

            case APP_FLASH_SIZE_FLASH:
            {
                BEEP_protocol_s prot;
                memset(&prot, 0, sizeof(BEEP_protocol_s));
                prot.command    = SIZE_MX_FLASH;
                prot.param.size = flash_size();
                sendProtocolField(&prot, whileCMDsource);

                // Clear the excuted command
                whileCMD = UINT8_MAX;
                break;
            }

            default:
                whileCMD = UINT8_MAX;
                break;
        }

    }

    // Check the Read Transfer state.    
    beep_fileReadTransferStatemachine();
}


