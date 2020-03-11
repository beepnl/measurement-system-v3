/*!
 * \file      LmHandlerMsgDisplay.h
 *
 * \brief     Common set of functions to display default messages from
 *            LoRaMacHandler.
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2019 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 */
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "utilities.h"
#include "timer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "LmHandlerMsgDisplay.h"

/*!
 * MAC status strings
 */
const char* MacStatusStrings[] =
{
    "OK",                            // LORAMAC_STATUS_OK
    "Busy",                          // LORAMAC_STATUS_BUSY
    "Service unknown",               // LORAMAC_STATUS_SERVICE_UNKNOWN
    "Parameter invalid",             // LORAMAC_STATUS_PARAMETER_INVALID
    "Frequency invalid",             // LORAMAC_STATUS_FREQUENCY_INVALID
    "Datarate invalid",              // LORAMAC_STATUS_DATARATE_INVALID
    "Frequency or datarate invalid", // LORAMAC_STATUS_FREQ_AND_DR_INVALID
    "No network joined",             // LORAMAC_STATUS_NO_NETWORK_JOINED
    "Length error",                  // LORAMAC_STATUS_LENGTH_ERROR
    "Region not supported",          // LORAMAC_STATUS_REGION_NOT_SUPPORTED
    "Skipped APP data",              // LORAMAC_STATUS_SKIPPED_APP_DATA
    "Duty-cycle restricted",         // LORAMAC_STATUS_DUTYCYCLE_RESTRICTED
    "No channel found",              // LORAMAC_STATUS_NO_CHANNEL_FOUND
    "No free channel found",         // LORAMAC_STATUS_NO_FREE_CHANNEL_FOUND
    "Busy beacon reserved time",     // LORAMAC_STATUS_BUSY_BEACON_RESERVED_TIME
    "Busy ping-slot window time",    // LORAMAC_STATUS_BUSY_PING_SLOT_WINDOW_TIME
    "Busy uplink collision",         // LORAMAC_STATUS_BUSY_UPLINK_COLLISION
    "Crypto error",                  // LORAMAC_STATUS_CRYPTO_ERROR
    "FCnt handler error",            // LORAMAC_STATUS_FCNT_HANDLER_ERROR
    "MAC command error",             // LORAMAC_STATUS_MAC_COMMAD_ERROR
    "ClassB error",                  // LORAMAC_STATUS_CLASS_B_ERROR
    "Confirm queue error",           // LORAMAC_STATUS_CONFIRM_QUEUE_ERROR
    "Multicast group undefined",     // LORAMAC_STATUS_MC_GROUP_UNDEFINED
    "Unknown error",                 // LORAMAC_STATUS_ERROR
};

/*!
 * MAC event info status strings.
 */
const char* EventInfoStatusStrings[] =
{ 
    "OK",                            // LORAMAC_EVENT_INFO_STATUS_OK
    "Error",                         // LORAMAC_EVENT_INFO_STATUS_ERROR
    "Tx timeout",                    // LORAMAC_EVENT_INFO_STATUS_TX_TIMEOUT
    "Rx 1 timeout",                  // LORAMAC_EVENT_INFO_STATUS_RX1_TIMEOUT
    "Rx 2 timeout",                  // LORAMAC_EVENT_INFO_STATUS_RX2_TIMEOUT
    "Rx1 error",                     // LORAMAC_EVENT_INFO_STATUS_RX1_ERROR
    "Rx2 error",                     // LORAMAC_EVENT_INFO_STATUS_RX2_ERROR
    "Join failed",                   // LORAMAC_EVENT_INFO_STATUS_JOIN_FAIL
    "Downlink repeated",             // LORAMAC_EVENT_INFO_STATUS_DOWNLINK_REPEATED
    "Tx DR payload size error",      // LORAMAC_EVENT_INFO_STATUS_TX_DR_PAYLOAD_SIZE_ERROR
    "Downlink too many frames loss", // LORAMAC_EVENT_INFO_STATUS_DOWNLINK_TOO_MANY_FRAMES_LOSS
    "Address fail",                  // LORAMAC_EVENT_INFO_STATUS_ADDRESS_FAIL
    "MIC fail",                      // LORAMAC_EVENT_INFO_STATUS_MIC_FAIL
    "Multicast fail",                // LORAMAC_EVENT_INFO_STATUS_MULTICAST_FAIL
    "Beacon locked",                 // LORAMAC_EVENT_INFO_STATUS_BEACON_LOCKED
    "Beacon lost",                   // LORAMAC_EVENT_INFO_STATUS_BEACON_LOST
    "Beacon not found"               // LORAMAC_EVENT_INFO_STATUS_BEACON_NOT_FOUND
};


const char * getMacStatusStringPointer(LoRaMacStatus_t status)
{
	if(status > LORAMAC_STATUS_ERROR)
	{
		return NULL;
	}
	return MacStatusStrings[status];
}


const char * getEventInfoStatusStringPointer(LoRaMacEventInfoStatus_t Status)
{
	if(Status > LORAMAC_EVENT_INFO_STATUS_BEACON_NOT_FOUND)
	{
		return NULL;
	}
	return EventInfoStatusStrings[Status];
}

/*!
 * Prints the provided buffer in HEX
 * 
 * \param buffer Buffer to be printed
 * \param size   Buffer size to be printed
 */
void PrintHexBuffer( uint8_t *buffer, uint8_t size )
{
    uint8_t newline = 0;

    for( uint8_t i = 0; i < size; i++ )
    {
        if( newline != 0 )
        {
            NRF_LOG_INFO( "\r\n" );
            newline = 0;
        }

        NRF_LOG_INFO( "%02X ", buffer[i] );

        if( ( ( i + 1 ) % 16 ) == 0 )
        {
            newline = 1;
        }
    }
    NRF_LOG_INFO( "\r\n" );
}

void DisplayNvmContextChange( LmHandlerNvmContextStates_t state )
{
    if( state == LORAMAC_HANDLER_NVM_STORE )
    {
        NRF_LOG_INFO( "\r\n###### ============ CTXS STORED ============ ######\r\n\r\n" );
    }
    else
    {
        NRF_LOG_INFO( "\r\n###### =========== CTXS RESTORED =========== ######\r\n\r\n" );
    }
}

void DisplayNetworkParametersUpdate( CommissioningParams_t *commissioningParams )
{
    NRF_LOG_INFO("Network Parameters Update");
    printStringWithHexArray("DevEui      : %02X",  8, commissioningParams->DevEui);
    printStringWithHexArray("AppEui      : %02X",  8, commissioningParams->JoinEui);

    // For 1.0.x devices the AppKey corresponds to NwkKey
    printStringWithHexArray("AppEui      : %02X", 16, commissioningParams->NwkKey);
}

void DisplayMacMcpsRequestUpdate( LoRaMacStatus_t status, McpsReq_t *mcpsReq )
{
    switch( mcpsReq->Type )
    {
        case MCPS_CONFIRMED:
        {
            NRF_LOG_INFO( "\r\n###### =========== MCPS-Request ============ ######\r\n" );
            NRF_LOG_INFO( "######            MCPS_CONFIRMED             ######\r\n");
            NRF_LOG_INFO( "###### ===================================== ######\r\n");
            break;
        }
        case MCPS_UNCONFIRMED:
        {
            NRF_LOG_INFO( "\r\n###### =========== MCPS-Request ============ ######\r\n" );
            NRF_LOG_INFO( "######           MCPS_UNCONFIRMED            ######\r\n");
            NRF_LOG_INFO( "###### ===================================== ######\r\n");
            break;
        }
        case MCPS_PROPRIETARY:
        {
            NRF_LOG_INFO( "\r\n###### =========== MCPS-Request ============ ######\r\n" );
            NRF_LOG_INFO( "######           MCPS_PROPRIETARY            ######\r\n");
            NRF_LOG_INFO( "###### ===================================== ######\r\n");
            break;
        }
        default:
        {
            NRF_LOG_INFO( "\r\n###### =========== MCPS-Request ============ ######\r\n" );
            NRF_LOG_INFO( "######                MCPS_ERROR             ######\r\n");
            NRF_LOG_INFO( "###### ===================================== ######\r\n");
            break;
        }
    }
    NRF_LOG_INFO( "STATUS      : %s\r\n", getMacStatusStringPointer(status) );
}

void DisplayMacMlmeRequestUpdate( LoRaMacStatus_t status, MlmeReq_t *mlmeReq )
{
    switch( mlmeReq->Type )
    {
        case MLME_JOIN:
        {
            NRF_LOG_INFO( "\r\n###### =========== MLME-Request ============ ######\r\n" );
            NRF_LOG_INFO( "######               MLME_JOIN               ######\r\n");
            NRF_LOG_INFO( "###### ===================================== ######\r\n");
            break;
        }
        case MLME_LINK_CHECK:
        {
            NRF_LOG_INFO( "\r\n###### =========== MLME-Request ============ ######\r\n" );
            NRF_LOG_INFO( "######            MLME_LINK_CHECK            ######\r\n");
            NRF_LOG_INFO( "###### ===================================== ######\r\n");
            break;
        }
        case MLME_DEVICE_TIME:
        {
            NRF_LOG_INFO( "\r\n###### =========== MLME-Request ============ ######\r\n" );
            NRF_LOG_INFO( "######            MLME_DEVICE_TIME           ######\r\n");
            NRF_LOG_INFO( "###### ===================================== ######\r\n");
            break;
        }
        case MLME_TXCW:
        {
            NRF_LOG_INFO( "\r\n###### =========== MLME-Request ============ ######\r\n" );
            NRF_LOG_INFO( "######               MLME_TXCW               ######\r\n");
            NRF_LOG_INFO( "###### ===================================== ######\r\n");
            break;
        }
        case MLME_TXCW_1:
        {
            NRF_LOG_INFO( "\r\n###### =========== MLME-Request ============ ######\r\n" );
            NRF_LOG_INFO( "######               MLME_TXCW_1             ######\r\n");
            NRF_LOG_INFO( "###### ===================================== ######\r\n");
            break;
        }
        default:
        {
            NRF_LOG_INFO( "\r\n###### =========== MLME-Request ============ ######\r\n" );
            NRF_LOG_INFO( "######              MLME_UNKNOWN             ######\r\n");
            NRF_LOG_INFO( "###### ===================================== ######\r\n");
            break;
        }
    }
    NRF_LOG_INFO( "STATUS      : %s\r\n", getMacStatusStringPointer(status) );
}

void DisplayJoinRequestUpdate( LmHandlerJoinParams_t *params )
{
    if( params->CommissioningParams->IsOtaaActivation == true )
    {
        if( params->Status == LORAMAC_HANDLER_SUCCESS )
        {
            NRF_LOG_INFO( "###### ===========   JOINED     ============ ######\r\n" );
            NRF_LOG_INFO( "\r\nOTAA\r\n\r\n" );
            NRF_LOG_INFO( "DevAddr     :  %08lX\r\n", params->CommissioningParams->DevAddr );
            NRF_LOG_INFO( "\r\n\r\n" );
            NRF_LOG_INFO( "DATA RATE   : DR_%d\r\n\r\n", params->Datarate );
        }
    }
#if ( OVER_THE_AIR_ACTIVATION == 0 )
    else
    {
        NRF_LOG_INFO( "###### ===========   JOINED     ============ ######\r\n" );
        NRF_LOG_INFO( "\r\nABP\r\n\r\n" );
        NRF_LOG_INFO( "DevAddr     : %08lX\r\n", params->CommissioningParams->DevAddr );
        NRF_LOG_INFO( "NwkSKey     : %02X", params->CommissioningParams->FNwkSIntKey[0] );
        for( int i = 1; i < 16; i++ )
        {
            NRF_LOG_INFO( " %02X", params->CommissioningParams->FNwkSIntKey[i] );
        }
        NRF_LOG_INFO( "\r\n" );
        NRF_LOG_INFO( "AppSKey     : %02X", params->CommissioningParams->AppSKey[0] );
        for( int i = 1; i < 16; i++ )
        {
            NRF_LOG_INFO( " %02X", params->CommissioningParams->AppSKey[i] );
        }
        NRF_LOG_INFO( "\n\r\n" );
    }
#endif
}

void DisplayTxUpdate( LmHandlerTxParams_t *params )
{
    MibRequestConfirm_t mibGet;

    if( params->IsMcpsConfirm == 0 )
    {
        NRF_LOG_INFO( "\r\n###### =========== MLME-Confirm ============ ######" );
        NRF_LOG_INFO( "STATUS      : %s\r\n", EventInfoStatusStrings[params->Status] );
        return;
    }

    NRF_LOG_INFO( "\r\n###### =========== MCPS-Confirm ============ ######" );
    NRF_LOG_INFO( "STATUS      : %s\r\n", EventInfoStatusStrings[params->Status] );

    NRF_LOG_INFO( "\r\n###### =====   UPLINK FRAME %8lu   ===== ######", params->UplinkCounter );

    NRF_LOG_INFO( "CLASS       : %c", "ABC"[LmHandlerGetCurrentClass( )] );
    NRF_LOG_INFO( "TX PORT     : %d", params->AppData.Port );

    if( params->AppData.BufferSize != 0 )
    {
        NRF_LOG_INFO( "TX DATA     : " );
        if( params->MsgType == LORAMAC_HANDLER_CONFIRMED_MSG )
        {
            NRF_LOG_INFO( "CONFIRMED - %s", ( params->AckReceived != 0 ) ? "ACK" : "NACK" );
        }
        else
        {
            NRF_LOG_INFO( "UNCONFIRMED" );
        }
        PrintHexBuffer( params->AppData.Buffer, params->AppData.BufferSize );
    }

    NRF_LOG_INFO( "DATA RATE   : DR_%d", params->Datarate );

    mibGet.Type  = MIB_CHANNELS;
    if( LoRaMacMibGetRequestConfirm( &mibGet ) == LORAMAC_STATUS_OK )
    {
        NRF_LOG_INFO( "U/L FREQ    : %lu", mibGet.Param.ChannelList[params->Channel].Frequency );
    }

    NRF_LOG_INFO( "TX POWER    : %d", params->TxPower );

    mibGet.Type  = MIB_CHANNELS_MASK;
    if( LoRaMacMibGetRequestConfirm( &mibGet ) == LORAMAC_STATUS_OK )
    {
        NRF_LOG_INFO("CHANNEL MASK: ");
        switch( LmHandlerGetActiveRegion( ) )
        {
            case LORAMAC_REGION_AS923:
            case LORAMAC_REGION_CN779:
            case LORAMAC_REGION_EU868:
            case LORAMAC_REGION_IN865:
            case LORAMAC_REGION_KR920:
            case LORAMAC_REGION_EU433:
            case LORAMAC_REGION_RU864:
            {
                NRF_LOG_INFO( "%04X ", mibGet.Param.ChannelsMask[0] );
                break;
            }
            case LORAMAC_REGION_AU915:
            case LORAMAC_REGION_CN470:
            case LORAMAC_REGION_US915:
            {
                for( uint8_t i = 0; i < 5; i++)
                {
                    NRF_LOG_INFO( "%04X ", mibGet.Param.ChannelsMask[i] );
                }
                break;
            }
            default:
            {
                NRF_LOG_INFO( "\r\n###### ========= Unknown Region ============ ######" );
                break;
            }
        }
        NRF_LOG_INFO("\r\n");
    }

    NRF_LOG_INFO( "\r\n" );
}

void DisplayRxUpdate( LmHandlerAppData_t *appData, LmHandlerRxParams_t *params )
{
    const char *slotStrings[] = { "1", "2", "C", "C Multicast", "B Ping-Slot", "B Multicast Ping-Slot" };

    if( params->IsMcpsIndication == 0 )
    {
        NRF_LOG_INFO( "\r\n###### ========== MLME-Indication ========== ######\r\n" );
        NRF_LOG_INFO( "STATUS      : %s\r\n", EventInfoStatusStrings[params->Status] );
        return;
    }

    NRF_LOG_INFO( "\r\n###### ========== MCPS-Indication ========== ######\r\n" );
    NRF_LOG_INFO( "STATUS      : %s\r\n", EventInfoStatusStrings[params->Status] );
    NRF_LOG_INFO( "\r\n###### =====  DOWNLINK FRAME %8lu  ===== ######\r\n", params->DownlinkCounter );
    NRF_LOG_INFO( "RX WINDOW   : %s\r\n", slotStrings[params->RxSlot] );
    NRF_LOG_INFO( "RX PORT     : %d\r\n", appData->Port );

    if( appData->BufferSize != 0 )
    {
        NRF_LOG_INFO( "RX DATA[%u]: ", appData->BufferSize);
        PrintHexBuffer( appData->Buffer, appData->BufferSize );
    }

    NRF_LOG_INFO( "\r\n" );
    NRF_LOG_INFO( "DATA RATE   : DR_%d\r\n", params->Datarate );
    NRF_LOG_INFO( "RX RSSI     : %d\r\n", params->Rssi );
    NRF_LOG_INFO( "RX SNR      : %d\r\n", params->Snr );

    NRF_LOG_INFO( "\r\n" );
}

void DisplayBeaconUpdate( LoRaMAcHandlerBeaconParams_t *params )
{
    switch( params->State )
    {
        default:
        case LORAMAC_HANDLER_BEACON_ACQUIRING:
        {
            NRF_LOG_INFO( "\r\n###### ========= BEACON ACQUIRING ========== ######\r\n" );
            break;
        }
        case LORAMAC_HANDLER_BEACON_LOST:
        {
            NRF_LOG_INFO( "\r\n###### ============ BEACON LOST ============ ######\r\n" );
            break;
        }
        case LORAMAC_HANDLER_BEACON_RX:
        {
            NRF_LOG_INFO( "\r\n###### ===== BEACON %8lu ==== ######\r\n", params->Info.Time.Seconds );
            NRF_LOG_INFO( "GW DESC     : %d\r\n", params->Info.GwSpecific.InfoDesc );
            NRF_LOG_INFO( "GW INFO     : " );
            PrintHexBuffer( params->Info.GwSpecific.Info, 6 );
            NRF_LOG_INFO( "\r\n" );
            NRF_LOG_INFO( "FREQ        : %lu\r\n", params->Info.Frequency );
            NRF_LOG_INFO( "DATA RATE   : DR_%d\r\n", params->Info.Datarate );
            NRF_LOG_INFO( "RX RSSI     : %d\r\n", params->Info.Rssi );
            NRF_LOG_INFO( "RX SNR      : %d\r\n", params->Info.Snr );
            NRF_LOG_INFO( "\r\n" );
            break;
        }
        case LORAMAC_HANDLER_BEACON_NRX:
        {
            NRF_LOG_INFO( "\r\n###### ======== BEACON NOT RECEIVED ======== ######\r\n" );
            break;
        }
    }
}

void DisplayClassUpdate( DeviceClass_t deviceClass )
{
    NRF_LOG_INFO( "\r\n\r\n###### ===== Switch to Class %c done.  ===== ######\r\n\r\n", "ABC"[deviceClass] );
}

void DisplayAppInfo( const char* appName, const Version_t* appVersion, const Version_t* gitHubVersion )
{
    NRF_LOG_INFO( "\r\n###### ===================================== ######\r\n\r\n" );
    NRF_LOG_INFO( "Application name   : %s\r\n", appName );
    NRF_LOG_INFO( "Application version: %d.%d.%d\r\n", appVersion->Fields.Major, appVersion->Fields.Minor, appVersion->Fields.Revision );
    NRF_LOG_INFO( "GitHub base version: %d.%d.%d\r\n", gitHubVersion->Fields.Major, gitHubVersion->Fields.Minor, gitHubVersion->Fields.Revision );
    NRF_LOG_INFO( "\r\n###### ===================================== ######\r\n\r\n" );
}
