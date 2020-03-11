/*!
 * \file      LoRaWAN_imp.c
 *
 * \brief     LoRaMac classA device implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 */

/*! \file classA/nRF52840/main.c */

#include <stdio.h>
#define NRF_LOG_MODULE_NAME LoRaWAN
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "utilities.h"
#include "board.h"
#include "gpio.h"
#include "gpio-board.h"
#include "LoRaMac.h"
#include "Commissioning.h"
#include "LoRaWAN_imp.h"
#include "NvmCtxMgmt.h"
#include "nrf.h"
#include "nrf_log_ctrl.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_delay.h"
#include "nrf_log_default_backends.h"
#include "LmHandlerMsgDisplay.h"
#include "boards.h"
#include "nrf_drv_clock.h"
#include "app_timer.h"
#include "app_error.h"
#include "rng_nrf.h"
#include "nvm_fs.h"
#include "utilities.h"
#include "SQ.h"
#include "CayenneLpp.h"
#include "beep_protocol.h"
#include "nRF_ADC.h"
#include "HX711_app.h"
#include "DS18B20_app.h"
#include "audio_app.h"
#include "alarm_app.h"
#include "bme_app.h"

static bool joinedOTAA			= false;
static bool	adrEnable			= LORAWAN_ADR_ON;
static bool	dutycycleOn			= LORAWAN_DUTYCYCLE_ON;
static bool validKeys			= false;
static bool AppEnabled			= false;
static bool lorawanIsEnabled	= false;
static bool lorawanReset		= false;
static uint8_t DevEui[8]		= LORAWAN_DEVICE_EUI;
static uint8_t JoinEui[]		= LORAWAN_JOIN_EUI;
#if( ABP_ACTIVATION_LRWAN_VERSION == ABP_ACTIVATION_LRWAN_VERSION_V10x )
static uint8_t GenAppKey[]	= LORAWAN_GEN_APP_KEY;
#else
static uint8_t AppKey[]			= LORAWAN_APP_KEY;
#endif
static uint8_t NwkKey[]			= LORAWAN_NWK_KEY;


#if( OVER_THE_AIR_ACTIVATION == 0 )

static uint8_t FNwkSIntKey[]	= LORAWAN_F_NWK_S_INT_KEY;
static uint8_t SNwkSIntKey[]	= LORAWAN_S_NWK_S_INT_KEY;
static uint8_t NwkSEncKey[]		= LORAWAN_NWK_S_ENC_KEY;
static uint8_t AppSKey[]		= LORAWAN_APP_S_KEY;

/*!
 * Device address
 */
static uint32_t DevAddr = LORAWAN_DEVICE_ADDRESS;

#endif

protocol_cb_p main_callback;
BEEP_STATUS msgtTrigger = BEEP_UNKNOWN;

/*!
 * Application port
 */
static uint8_t AppPort = LORAWAN_APP_PORT;

/*!
 * User application data size
 */
static uint8_t AppDataSize = 1;
static uint8_t AppDataSizeBackup = 1;

/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_MAX_SIZE                           242

/*!
 * User application data
 */
static uint8_t AppDataBuffer[LORAWAN_APP_DATA_MAX_SIZE];
static uint8_t downlinkResponseBuffer[BEEP_LORAWAN_MAX_LENGHT];
static uint8_t downlinkResponseLenght = 0;

/*!
 * Indicates if the node is sending confirmed or unconfirmed messages
 */
static uint8_t IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;

/*!
 * Defines the application data transmission duty cycle
 */
static uint32_t TxDutyCycleTime;

/*!
 * Timer to handle the application data transmission duty cycle
 */
APP_TIMER_DEF(TxNextPacketTimer);

/*!
 * Specifies the state of the application LED
 */
static bool AppLedStateOn = false;


/*!
 * Indicates if a new packet can be sent
 */
static bool NextTx = false;

/*!
 * Indicates if LoRaMacProcess call is pending.
 * 
 * \warning If variable is equal to 0 then the MCU can be set in low power mode
 */
static uint8_t IsMacProcessPending = 0;


static uint32_t app_join_counter	= 0;
static uint32_t app_tx_interval		= APP_TX_DUTYCYCLE;

/*!
 * Device states
 */
static enum eDeviceState
{
    DEVICE_STATE_RESTORE,
    DEVICE_STATE_START,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_SEND,
    DEVICE_STATE_CYCLE,
    DEVICE_STATE_SLEEP
}DeviceState;

/*!
 * LoRaWAN compliance tests support data
 */
struct ComplianceTest_s
{
    bool Running;
    uint8_t State;
    bool IsTxConfirmed;
    uint8_t AppPort;
    uint8_t AppDataSize;
    uint8_t *AppDataBuffer;
    uint16_t DownLinkCounter;
    bool LinkCheck;
    uint8_t DemodMargin;
    uint8_t NbGateways;
}ComplianceTest;


/*!
 * Application data structure
 */
typedef struct LoRaMacHandlerAppData_s
{
    LmHandlerMsgTypes_t MsgType;
    uint8_t Port;
    uint8_t BufferSize;
    uint8_t *Buffer;
}LoRaMacHandlerAppData_t;

LoRaMacHandlerAppData_t AppData =
{
    .MsgType = LORAMAC_HANDLER_UNCONFIRMED_MSG,
    .Buffer = NULL,
    .BufferSize = 0,
    .Port = 0
};


/*!
 * Prints the provided buffer in HEX
 * 
 * \param buffer Buffer to be printed
 * \param size   Buffer size to be printed
 */
extern void PrintHexBuffer( uint8_t *buffer, uint8_t size );

const char * BeepMessagestatus_str(BEEP_STATUS status)
{
	const char* messagestatusToString [] =
	{
		"BEEP_LORAWAN_RESERVED",
		"BEEP_SENSOR_OFF",
		"BEEP_SENSOR_ON",
		"BEEP_KEEP_ALIVE",
		"BEEP_BLE_CUSTOM",
		"BEEP_DOWNLINK_RESPONSE",
		"BEEP_UNKNOWN",
	};

	return (status > BEEP_BLE_CUSTOM) ? messagestatusToString[BEEP_BLE_CUSTOM + 1] : messagestatusToString[status];
}


void setTrigger(BEEP_STATUS triggerValue)
{
	msgtTrigger = triggerValue;
}

static uint32_t GetJoinInterval(void)
{
	uint32_t interval_min = 1;

	if(app_join_counter < 10)
	{
		interval_min = 1;	// 1 minute interval. 
	}
	else if(app_join_counter >= 10 && app_join_counter < 20)
	{
		interval_min = 10;	// 10 minute interval. 10 minutes have passed
	}
	else if(app_join_counter >= 20 && app_join_counter < 50)
	{
		interval_min = 60;	// 1 hour interval. 210 minutes have passed
	}
	else
	{
		interval_min = 24;	// 1 day interval. 1910 minutes have passed 
	}

	return (interval_min * 60 * 1000); // Multiply by 60 and 1000 in order to calculate the interval in ms.
}


/*!
 * Executes the network Join request
 */
static void JoinNetwork( void )
{
    LoRaMacStatus_t status;
    MlmeReq_t mlmeReq;
    mlmeReq.Type				= MLME_JOIN;
    mlmeReq.Req.Join.DevEui		= DevEui;
    mlmeReq.Req.Join.JoinEui	= JoinEui;
    mlmeReq.Req.Join.Datarate	= LORAWAN_DEFAULT_DATARATE;

    if(!lorawanIsEnabled || !AppEnabled)
	{
		return;
	}

    // Starts the join procedure
    status = LoRaMacMlmeRequest( &mlmeReq );

	#if 0
		NRF_LOG_INFO( "###### ===== MLME-Request - MLME_JOIN ==== ######" );
		NRF_LOG_INFO( "JOIN STATUS      : %s", getMacStatusStringPointer(status) );
	#endif

    if( status == LORAMAC_STATUS_OK )
    {
        NRF_LOG_INFO( "###### ===== JOINING ==== ######" );
        DeviceState = DEVICE_STATE_SLEEP;
		if(app_join_counter < UINT32_MAX)
		{
			app_join_counter++;
		}

    }
    else
    {
		//NRF_LOG_INFO( "JOIN STATUS      : %s = DEVICE_STATE_CYCLE", getMacStatusStringPointer(status) );
        DeviceState = DEVICE_STATE_CYCLE;
    }
}

/*!
 * \brief   Prepares the payload of the frame
 */
static void PrepareTxFrame( uint8_t port )
{
    switch( port )
    {
    case 2:
        {
            AppDataSizeBackup = AppDataSize = 1;
            AppDataBuffer[0] = AppLedStateOn;
        }
        break;
    case 224:
        if( ComplianceTest.LinkCheck == true )
        {
            ComplianceTest.LinkCheck = false;
            AppDataSize				= 3;
            AppDataBuffer[0]		= 5;
            AppDataBuffer[1]		= ComplianceTest.DemodMargin;
            AppDataBuffer[2]		= ComplianceTest.NbGateways;
            ComplianceTest.State	= 1;
        }
        else
        {
            switch( ComplianceTest.State )
            {
            case 4:
                ComplianceTest.State = 1;
                break;
            case 1:
                AppDataSize = 2;
                AppDataBuffer[0] = ComplianceTest.DownLinkCounter >> 8;
                AppDataBuffer[1] = ComplianceTest.DownLinkCounter;
                break;
            }
        }
        break;
    default:
        break;
    }
}

/*!
 * \brief   Prepares the payload of the frame
 *
 * \retval  [0: frame could be send, 1: error]
 */
static bool SendFrame( void )
{
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;

	if(!lorawanIsEnabled || !AppEnabled)
	{
		return false;
	}

    if( LoRaMacQueryTxPossible( AppDataSize, &txInfo ) != LORAMAC_STATUS_OK )
    {
        // Send empty frame in order to flush MAC commands
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
    }
    else
    {
        if( IsTxConfirmed == false )
        {
            mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fPort		= AppPort;
            mcpsReq.Req.Unconfirmed.fBuffer		= AppDataBuffer;
            mcpsReq.Req.Unconfirmed.fBufferSize	= AppDataSize;
            mcpsReq.Req.Unconfirmed.Datarate	= LORAWAN_DEFAULT_DATARATE;
        }
        else
        {
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fPort			= AppPort;
            mcpsReq.Req.Confirmed.fBuffer		= AppDataBuffer;
            mcpsReq.Req.Confirmed.fBufferSize	= AppDataSize;
            mcpsReq.Req.Confirmed.Datarate		= LORAWAN_DEFAULT_DATARATE;
        }
    }

    // Update global variable
    AppData.MsgType		= ( mcpsReq.Type == MCPS_CONFIRMED ) ? LORAMAC_HANDLER_CONFIRMED_MSG : LORAMAC_HANDLER_UNCONFIRMED_MSG;
    AppData.Port		= mcpsReq.Req.Unconfirmed.fPort;
    AppData.Buffer		= mcpsReq.Req.Unconfirmed.fBuffer;
    AppData.BufferSize	= mcpsReq.Req.Unconfirmed.fBufferSize;

    LoRaMacStatus_t status;
    status = LoRaMacMcpsRequest( &mcpsReq );
    NRF_LOG_INFO( "###### ===== SendFrame ==== ######" );
    NRF_LOG_INFO( "STATUS      : %s", getMacStatusStringPointer(status) );

    if( status == LORAMAC_STATUS_OK )
    {
        return false;
    }
    return true;
}


uint32_t Beep_SendFormat(BEEP_STATUS index, MEASUREMENT_RESULT_s * alarm)
{
	uint8_t payload[PAYLOAD_SIZE_MAX] = {0};
	uint8_t payloadLenght = 0;
	uint32_t ret;

    BEEP_protocol_s	get;
	memset(&get, 0, sizeof(get));

	switch(index)
	{
		case 0:
			return NRF_ERROR_INVALID_DATA;
			break;

		case BEEP_SENSOR_OFF:
		case BEEP_SENSOR_ON:
		{
			// Add the firmware version: major.minor.sub
			get.command = READ_FIRMWARE_VERSION;
            nvm_fds_eeprom_get(&get);
            beep_protocol_encode(true, &get, payload, &payloadLenght, PAYLOAD_SIZE_MAX);

            // Add the Hardare version: major, minor, PCB ID
			get.command = READ_HARDWARE_VERSION;
            nvm_fds_eeprom_get(&get);
            beep_protocol_encode(true, &get, payload, &payloadLenght, PAYLOAD_SIZE_MAX);

            // Add the unique serial number of the ATTEC
            get.command = READ_ATECC_READ_ID;
            nvm_fds_eeprom_get(&get);
            beep_protocol_encode(true, &get, payload, &payloadLenght, PAYLOAD_SIZE_MAX);
			break;
		}

        case BEEP_ALARM:
        {
            uint8_t alarmFlags      = getAlarmsTransmit(true);
            NRF_LOG_INFO("LoRaWAN Alarm: flags=0x%02X", alarmFlags);
            payload[payloadLenght]  = alarmFlags;
            payloadLenght++;
            // Intentional fall-through to BEEP_KEEP_ALIVE
        }
		case BEEP_KEEP_ALIVE:
		{
            MEASUREMENT_RESULT_s hx711;

			// Add battery voltage
            get.command = READ_nRF_ADC_CONVERSION;
            get_saadc_result(&get.param.meas_result);
            beep_protocol_encode(true, &get, payload, &payloadLenght, PAYLOAD_SIZE_MAX);
            memset(&get, 0, sizeof(get));

            // Add HX711 value
            get.command = READ_HX711_CONVERSION;
            HX711_app_getLastResult(&get.param.meas_result);
            beep_protocol_encode(true, &get, payload, &payloadLenght, PAYLOAD_SIZE_MAX);
            memset(&get, 0, sizeof(get));

            // Add DS18B20 temperature
            get.command = READ_DS18B20_CONVERSION;
            DS18B20_App_getTemp(&get.param.meas_result);
            beep_protocol_encode(true, &get, payload, &payloadLenght, PAYLOAD_SIZE_MAX);
            memset(&get, 0, sizeof(get));

            // Add FFT data
            get.command = READ_AUDIO_ADC_CONVERSION;
            audio_app_get_result(&get.param.meas_result);
            beep_protocol_encode(true, &get, payload, &payloadLenght, PAYLOAD_SIZE_MAX);
            memset(&get, 0, sizeof(get));

            // Add the temperature, Relative Humidity and Barometric pressure to the log
            get.command = BME280_CONVERSION_READ;
            bme_app_get_result(&get.param.meas_result);
            beep_protocol_encode(true, &get, payload, &payloadLenght, PAYLOAD_SIZE_MAX);
            memset(&get, 0, sizeof(get));
			break;
		}

		case BEEP_DOWNLINK_RESPONSE:
		{
			// Attempt to send the buffer with the downlink response value
            ret = BEEP_Send(BEEP_DOWNLINK_RESPONSE, downlinkResponseBuffer, downlinkResponseLenght);
            NRF_LOG_INFO("Send Downlink response: lenght%u, err: 0x%04X", downlinkResponseLenght, ret);
			if(ret == NRF_SUCCESS || (ret != LORAMAC_STATUS_BUSY) && (ret != LORAMAC_STATUS_LENGTH_ERROR) && (ret != LORAMAC_STATUS_DUTYCYCLE_RESTRICTED) && (ret != LORAMAC_STATUS_NO_CHANNEL_FOUND))
			{
				downlinkResponseLenght = 0;
				memset(downlinkResponseBuffer, 0, BEEP_LORAWAN_MAX_LENGHT);
			}
			return ret; // Prevent double transmissions or overwriting of the current message.
			break;
		}

		case BEEP_BLE_CUSTOM:
			return NRF_ERROR_INVALID_STATE;
			break;

		default:
			return NRF_ERROR_INVALID_PARAM;
			break;
	}

	ret = BEEP_Send((uint8_t) index, payload, payloadLenght);
	return ret;
}

uint32_t Beep_downlink(uint8_t fport, uint8_t * payload, uint8_t lenght)
{
	BEEP_protocol_s prot;
	uint8_t offset		= 0;
	uint8_t remaining	= 0;
	uint32_t ret		= NRF_SUCCESS;

	if(payload == NULL || fport == 0 || lenght == 0){
		return NRF_ERROR_INVALID_PARAM;	
	}

	while(offset < lenght)
	{
		// Loop until the message has been decoded or an error occurs.
        remaining = lenght - offset;
		ret = beep_protocol_decode(&prot, &payload[offset], remaining, &offset);
		if(ret != NRF_SUCCESS)
		{
			lorawanDownlinkResponseAppend(prot.command, ret);
			return ret;
		}

		// Report the command back to the main handler
		if(main_callback != NULL)
		{
            prot.source = LORAWAN_SOURCE;
			main_callback(LORAWAN_SOURCE, &prot);
		}

	}
	return NRF_SUCCESS;
}



/*!
 * \brief   Prepares the payload of the frame
 *
 * \retval  [0: frame could be send, 1: error]
 */
uint32_t BEEP_Send( uint8_t fport, uint8_t * payload, uint8_t payload_lenght )
{
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;
	uint8_t sqState = 0;
    uint8_t SFIdataSize = 6;


    if( LoRaMacQueryTxPossible( SFIdataSize, &txInfo ) != LORAMAC_STATUS_OK )
    {
        // Send empty frame in order to flush MAC commands
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer		= NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate	= LORAWAN_DEFAULT_DATARATE;
    }
    else
    {
		mcpsReq.Type = MCPS_UNCONFIRMED;
		mcpsReq.Req.Unconfirmed.fPort		= ((fport == 0) ? 1 : fport);
		mcpsReq.Req.Unconfirmed.fBuffer		= payload;
		mcpsReq.Req.Unconfirmed.fBufferSize	= payload_lenght;
		mcpsReq.Req.Unconfirmed.Datarate	= LORAWAN_DEFAULT_DATARATE;
    }

    // Update global variable
    AppData.MsgType		= ( mcpsReq.Type == MCPS_CONFIRMED ) ? LORAMAC_HANDLER_CONFIRMED_MSG : LORAMAC_HANDLER_UNCONFIRMED_MSG;
    AppData.Port		= mcpsReq.Req.Unconfirmed.fPort;
    AppData.Buffer		= mcpsReq.Req.Unconfirmed.fBuffer;
    AppData.BufferSize	= mcpsReq.Req.Unconfirmed.fBufferSize;

    LoRaMacStatus_t status;
    status = LoRaMacMcpsRequest( &mcpsReq );
    NRF_LOG_INFO( "###### ===== BEEP SendFrame ==== ######" );
    NRF_LOG_INFO( "Trigger: %s, STATUS      : %s",  BeepMessagestatus_str(msgtTrigger), getMacStatusStringPointer(status) );
    msgtTrigger = BEEP_UNKNOWN; // Clear the message trigger

    switch(status)
    {
        case LORAMAC_STATUS_OK:                 return NRF_SUCCESS;                 break;
        case LORAMAC_STATUS_BUSY:               return NRF_ERROR_BUSY;              break;
        case LORAMAC_STATUS_PARAMETER_INVALID:  return NRF_ERROR_INVALID_PARAM;     break;
        case LORAMAC_STATUS_LENGTH_ERROR:       return NRF_ERROR_INVALID_LENGTH;    break;    
        default:                                return NRF_ERROR_INTERNAL;          break;
    }
    return NRF_ERROR_FORBIDDEN;
}



/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
static void OnTxNextPacketTimerEvent( void * context )
{
    MibRequestConfirm_t mibReq;
    LoRaMacStatus_t status;

    app_timer_stop(TxNextPacketTimer);

    mibReq.Type = MIB_NETWORK_ACTIVATION;
    status = LoRaMacMibGetRequestConfirm( &mibReq );

    if( status == LORAMAC_STATUS_OK )
    {
        if( mibReq.Param.NetworkActivation == ACTIVATION_TYPE_NONE )
        {
            // Network not joined yet. Try to join again
            JoinNetwork( );
        }
        else
        {
            DeviceState = DEVICE_STATE_CYCLE;
        }
    }
}



/*!
 * \brief   MCPS-Confirm event function
 *
 * \param   [IN] mcpsConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void McpsConfirm( McpsConfirm_t *mcpsConfirm )
{
#if !LORAMAC_LOG_ENABLED
	return;
#else


    NRF_LOG_INFO( "###### ===== MCPS-Confirm ==== ######" );
    NRF_LOG_INFO( "STATUS      : %s", getEventInfoStatusStringPointer(mcpsConfirm->Status) );
    if( mcpsConfirm->Status != LORAMAC_EVENT_INFO_STATUS_OK )
    {
    }
    else
    {
        switch( mcpsConfirm->McpsRequest )
        {
            case MCPS_UNCONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                break;
            }
            case MCPS_CONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                // Check AckReceived
                // Check NbTrans
                break;
            }
            case MCPS_PROPRIETARY:
            {
                break;
            }
            default:
                break;
        }
    }
    MibRequestConfirm_t mibGet;
    MibRequestConfirm_t mibReq;

    mibReq.Type = MIB_DEVICE_CLASS;
    LoRaMacMibGetRequestConfirm( &mibReq );

    NRF_LOG_INFO( "###### ===== UPLINK FRAME %lu ==== ######", mcpsConfirm->UpLinkCounter );
    NRF_LOG_INFO( "" );

    NRF_LOG_INFO( "CLASS       : %c", "ABC"[mibReq.Param.Class] );
    NRF_LOG_INFO( "TX PORT     : %d", AppData.Port );

    if( AppData.BufferSize != 0 )
    {
        NRF_LOG_INFO( "TX DATA     : " );
        if( AppData.MsgType == LORAMAC_HANDLER_CONFIRMED_MSG )
        {
            NRF_LOG_INFO( "CONFIRMED - %s", ( mcpsConfirm->AckReceived != 0 ) ? "ACK" : "NACK" );
        }
        else
        {
            NRF_LOG_INFO( "UNCONFIRMED" );
        }
        PrintHexBuffer( AppData.Buffer, AppData.BufferSize );
    }

    NRF_LOG_INFO( "" );
    NRF_LOG_INFO( "DATA RATE   : DR_%d", mcpsConfirm->Datarate );

    mibGet.Type  = MIB_CHANNELS;
    if( LoRaMacMibGetRequestConfirm( &mibGet ) == LORAMAC_STATUS_OK )
    {
        NRF_LOG_INFO( "U/L FREQ    : %lu", mibGet.Param.ChannelList[mcpsConfirm->Channel].Frequency );
    }

    NRF_LOG_INFO( "TX POWER    : %d", mcpsConfirm->TxPower );

    mibGet.Type  = MIB_CHANNELS_MASK;
    if( LoRaMacMibGetRequestConfirm( &mibGet ) == LORAMAC_STATUS_OK )
    {
        NRF_LOG_INFO("CHANNEL MASK: ");
		#if defined( REGION_AS923 ) || defined( REGION_CN779 ) || \
			defined( REGION_EU868 ) || defined( REGION_IN865 ) || \
			defined( REGION_KR920 ) || defined( REGION_EU433 ) || \
			defined( REGION_RU864 )

			for( uint8_t i = 0; i < 1; i++)

		#elif defined( REGION_AU915 ) || defined( REGION_US915 ) || defined( REGION_CN470 )
			for( uint8_t i = 0; i < 5; i++)
		#else
			#error "Please define a region in the compiler options."
		#endif
        {
            NRF_LOG_INFO("%04X ", mibGet.Param.ChannelsMask[i] );
        }
        NRF_LOG_INFO("");
    }

    NRF_LOG_INFO( "" );
#endif
}

/*!
 * \brief   MCPS-Indication event function
 *
 * \param   [IN] mcpsIndication - Pointer to the indication structure,
 *               containing indication attributes.
 */
static void McpsIndication( McpsIndication_t *mcpsIndication )
{
    NRF_LOG_INFO( "\r\n###### ===== MCPS-Indication ==== ######" );
    NRF_LOG_INFO( "STATUS      : %s", getEventInfoStatusStringPointer(mcpsIndication->Status) );
    if( mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK )
    {
        return;
    }

    switch( mcpsIndication->McpsIndication )
    {
        case MCPS_UNCONFIRMED:
        {
            break;
        }
        case MCPS_CONFIRMED:
        {
            break;
        }
        case MCPS_PROPRIETARY:
        {
            break;
        }
        case MCPS_MULTICAST:
        {
            break;
        }
        default:
            break;
    }

    // Check Multicast
    // Check Port
    // Check Datarate
    // Check FramePending
    if( mcpsIndication->FramePending == true )
    {
        // The server signals that it has pending data to be sent.
        // We schedule an uplink as soon as possible to flush the server.
        OnTxNextPacketTimerEvent( NULL );
    }
    // Check Buffer
    // Check BufferSize
    // Check Rssi
    // Check Snr
    // Check RxSlot

    if( ComplianceTest.Running == true )
    {
        ComplianceTest.DownLinkCounter++;
    }

    if( mcpsIndication->RxData == true )
    {
        switch( mcpsIndication->Port )
        {
        case 1: // The application LED can be controlled on port 1 or 2
        case 2:
            if( mcpsIndication->BufferSize == 1 )
            {
                AppLedStateOn = mcpsIndication->Buffer[0] & 0x01;
            }
            break;
        case 224:
            if( ComplianceTest.Running == false )
            {
                // Check compliance test enable command (i)
                if( ( mcpsIndication->BufferSize == 4 ) &&
                    ( mcpsIndication->Buffer[0] == 0x01 ) &&
                    ( mcpsIndication->Buffer[1] == 0x01 ) &&
                    ( mcpsIndication->Buffer[2] == 0x01 ) &&
                    ( mcpsIndication->Buffer[3] == 0x01 ) )
                {
                    IsTxConfirmed = false;
                    AppPort = 224;
                    AppDataSizeBackup = AppDataSize;
                    AppDataSize = 2;
                    ComplianceTest.DownLinkCounter = 0;
                    ComplianceTest.LinkCheck = false;
                    ComplianceTest.DemodMargin = 0;
                    ComplianceTest.NbGateways = 0;
                    ComplianceTest.Running = true;
                    ComplianceTest.State = 1;

                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_ADR;
                    mibReq.Param.AdrEnable = true;
                    LoRaMacMibSetRequestConfirm( &mibReq );

		    #if defined( REGION_EU868 ) || defined( REGION_RU864 ) || defined( REGION_CN779 ) || defined( REGION_EU433 )
			LoRaMacTestSetDutyCycleOn( false );
		    #endif
                }
            }
            else
            {
                ComplianceTest.State = mcpsIndication->Buffer[0];
                switch( ComplianceTest.State )
                {
                case 0: // Check compliance test disable command (ii)
                    IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;
                    AppPort = LORAWAN_APP_PORT;
                    AppDataSize = AppDataSizeBackup;
                    ComplianceTest.DownLinkCounter = 0;
                    ComplianceTest.Running = false;

                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_ADR;
                    mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                    LoRaMacMibSetRequestConfirm( &mibReq );

		    #if defined( REGION_EU868 ) || defined( REGION_RU864 ) || defined( REGION_CN779 ) || defined( REGION_EU433 )
			LoRaMacTestSetDutyCycleOn( dutycycleOn );
		    #endif
                    break;
                case 1: // (iii, iv)
                    AppDataSize = 2;
                    break;
                case 2: // Enable confirmed messages (v)
                    IsTxConfirmed = true;
                    ComplianceTest.State = 1;
                    break;
                case 3:  // Disable confirmed messages (vi)
                    IsTxConfirmed = false;
                    ComplianceTest.State = 1;
                    break;
                case 4: // (vii)
                    AppDataSize = mcpsIndication->BufferSize;

                    AppDataBuffer[0] = 4;
                    for( uint8_t i = 1; i < MIN( AppDataSize, LORAWAN_APP_DATA_MAX_SIZE ); i++ )
                    {
                        AppDataBuffer[i] = mcpsIndication->Buffer[i] + 1;
                    }
                    break;
                case 5: // (viii)
                    {
                        MlmeReq_t mlmeReq;
                        mlmeReq.Type = MLME_LINK_CHECK;
                        LoRaMacStatus_t status = LoRaMacMlmeRequest( &mlmeReq );
                        NRF_LOG_INFO( "###### ===== MLME-Request - MLME_LINK_CHECK ==== ######" );
                        NRF_LOG_INFO( "STATUS      : %s", getMacStatusStringPointer(status) );
                    }
                    break;
                case 6: // (ix)
                    {
                        // Disable TestMode and revert back to normal operation
                        IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;
                        AppPort = LORAWAN_APP_PORT;
                        AppDataSize = AppDataSizeBackup;
                        ComplianceTest.DownLinkCounter = 0;
                        ComplianceTest.Running = false;

                        MibRequestConfirm_t mibReq;
                        mibReq.Type = MIB_ADR;
                        mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                        LoRaMacMibSetRequestConfirm( &mibReq );

			#if defined( REGION_EU868 ) || defined( REGION_RU864 ) || defined( REGION_CN779 ) || defined( REGION_EU433 )
			    LoRaMacTestSetDutyCycleOn( dutycycleOn );
			#endif

                        JoinNetwork( );
                    }
                    break;
                case 7: // (x)
                    {
                        if( mcpsIndication->BufferSize == 3 )
                        {
                            MlmeReq_t mlmeReq;
                            mlmeReq.Type = MLME_TXCW;
                            mlmeReq.Req.TxCw.Timeout = ( uint16_t )( ( mcpsIndication->Buffer[1] << 8 ) | mcpsIndication->Buffer[2] );
                            LoRaMacStatus_t status = LoRaMacMlmeRequest( &mlmeReq );
                            NRF_LOG_INFO( "###### ===== MLME-Request - MLME_TXCW ==== ######" );
                            NRF_LOG_INFO( "STATUS      : %s", getMacStatusStringPointer(status) );
                        }
                        else if( mcpsIndication->BufferSize == 7 )
                        {
                            MlmeReq_t mlmeReq;
                            mlmeReq.Type = MLME_TXCW_1;
                            mlmeReq.Req.TxCw.Timeout = ( uint16_t )( ( mcpsIndication->Buffer[1] << 8 ) | mcpsIndication->Buffer[2] );
                            mlmeReq.Req.TxCw.Frequency = ( uint32_t )( ( mcpsIndication->Buffer[3] << 16 ) | ( mcpsIndication->Buffer[4] << 8 ) | mcpsIndication->Buffer[5] ) * 100;
                            mlmeReq.Req.TxCw.Power = mcpsIndication->Buffer[6];
                            LoRaMacStatus_t status = LoRaMacMlmeRequest( &mlmeReq );
                            NRF_LOG_INFO( "###### ===== MLME-Request - MLME_TXCW1 ==== ######" );
                            NRF_LOG_INFO( "STATUS      : %s", getMacStatusStringPointer(status) );
                        }
                        ComplianceTest.State = 1;
                    }
                    break;
                default:
                    break;
                }
            }
            break;
        default:
            break;
        }
    }

    const char *slotStrings[] = { "1", "2", "C", "C Multicast", "B Ping-Slot", "B Multicast Ping-Slot" };
    NRF_LOG_INFO( "###### ===== DOWNLINK FRAME %lu ==== ######", mcpsIndication->DownLinkCounter );
    NRF_LOG_INFO( "RX WINDOW   : %s", slotStrings[mcpsIndication->RxSlot] );
    NRF_LOG_INFO( "RX PORT     : %d", mcpsIndication->Port );

    if( mcpsIndication->BufferSize != 0 )
    {
        NRF_LOG_INFO( "RX DATA[%u]: ", mcpsIndication->BufferSize);
        PrintHexBuffer( mcpsIndication->Buffer, mcpsIndication->BufferSize );
        Beep_downlink(mcpsIndication->Port, mcpsIndication->Buffer, mcpsIndication->BufferSize);
    }

    NRF_LOG_INFO( "" );
    NRF_LOG_INFO( "DATA RATE   : DR_%d",	mcpsIndication->RxDatarate );
    NRF_LOG_INFO( "RX RSSI     : %d",		mcpsIndication->Rssi );
    NRF_LOG_INFO( "RX SNR      : %d",		mcpsIndication->Snr );
    NRF_LOG_INFO( "" );
}

/*!
 * \brief   MLME-Confirm event function
 *
 * \param   [IN] mlmeConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void MlmeConfirm( MlmeConfirm_t *mlmeConfirm )
{
    NRF_LOG_INFO( "###### ===== MLME-Confirm ==== ######" );
    NRF_LOG_INFO( "STATUS      : %s", getEventInfoStatusStringPointer(mlmeConfirm->Status) );
    if( mlmeConfirm->Status != LORAMAC_EVENT_INFO_STATUS_OK )
    {
    }
    switch( mlmeConfirm->MlmeRequest )
    {
        case MLME_JOIN:
        {
			uint8_t key[KEY_SIZE] = {0};
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
				LoRaMacStatus_t status;
                MibRequestConfirm_t mibGet;
                NRF_LOG_INFO( "###### ===== OTAA JOINED ==== ######" );

                mibGet.Type = MIB_DEV_ADDR;
                status = LoRaMacMibGetRequestConfirm( &mibGet );
                if(status == LORAMAC_STATUS_OK)
				{
					NRF_LOG_INFO( "Session DevAddr:         %08lX", mibGet.Param.DevAddr );
				}

                mibGet.Type = MIB_APP_S_KEY;
                mibGet.Param.AppSKey = key;
                status = LoRaMacMibGetRequestConfirm( &mibGet );
                if(status == LORAMAC_STATUS_OK)
				{
					printStringWithHexArray("App Session Key:     ", KEY_SIZE, mibGet.Param.AppSKey);
				}

                mibGet.Type = MIB_NWK_S_ENC_KEY;
                mibGet.Param.NwkSEncKey = key;
                status = LoRaMacMibGetRequestConfirm( &mibGet );
                if(status == LORAMAC_STATUS_OK)
				{
					printStringWithHexArray("Network Session Key:", KEY_SIZE, mibGet.Param.NwkSEncKey);
				}

                // Status is OK, node has joined the network
                joinedOTAA = true;
                DeviceState = DEVICE_STATE_SEND;
            }
            else
            {
                // Join was not successful. Try to join again
                JoinNetwork( );
            }
            break;
        }
        case MLME_LINK_CHECK:
        {
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                // Check DemodMargin
                // Check NbGateways
                if( ComplianceTest.Running == true )
                {
                    ComplianceTest.LinkCheck = true;
                    ComplianceTest.DemodMargin = mlmeConfirm->DemodMargin;
                    ComplianceTest.NbGateways = mlmeConfirm->NbGateways;
                }
            }
            break;
        }
        default:
            break;
    }
}

/*!
 * \brief   MLME-Indication event function
 *
 * \param   [IN] mlmeIndication - Pointer to the indication structure.
 */
static void MlmeIndication( MlmeIndication_t *mlmeIndication )
{
    if( mlmeIndication->Status != LORAMAC_EVENT_INFO_STATUS_BEACON_LOCKED )
    {
        NRF_LOG_INFO( "###### ===== MLME-Indication ==== ######" );
        NRF_LOG_INFO( "STATUS      : %s", getEventInfoStatusStringPointer(mlmeIndication->Status) );
    }
    if( mlmeIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK )
    {
    }
    switch( mlmeIndication->MlmeIndication )
    {
        case MLME_SCHEDULE_UPLINK:
        {// The MAC signals that we shall provide an uplink as soon as possible
            OnTxNextPacketTimerEvent( NULL );
            break;
        }
        default:
            break;
    }
}

void OnMacProcessNotify( void )
{
    IsMacProcessPending = 1;
}



uint32_t				ret;
LoRaMacStatus_t			retValLoRaMAC;
LoRaMacPrimitives_t	    macPrimitives;
LoRaMacCallback_t	    macCallbacks;
MibRequestConfirm_t	    mibReq;
LoRaMacStatus_t			status;
NvmCtxMgmtStatus_t		retNVM;



bool lorawan_valid_Keys(void)
{
	bool retval = false;

	NRF_LOG_FLUSH();
	NRF_LOG_INFO("DEVEUI:");
	NRF_LOG_HEXDUMP_INFO(DevEui, 8);
    NRF_LOG_FLUSH();
    NRF_LOG_INFO("APPEUI:");
	NRF_LOG_HEXDUMP_INFO(JoinEui, 8);
    NRF_LOG_FLUSH();
    NRF_LOG_INFO("AppKey:");
	NRF_LOG_HEXDUMP_INFO(AppKey, 16);
    NRF_LOG_FLUSH();

	if( is_array_filled_with(0, DevEui,  8) || is_array_filled_with(0xFF, DevEui,  8) ||
		is_array_filled_with(0, JoinEui, 8) || is_array_filled_with(0xFF, JoinEui, 8) ||
		is_array_filled_with(0, AppKey, 16) || is_array_filled_with(0xFF, AppKey, 16))
	{
		// invalid keys detected.
		return false;
	}
	return true;
}

bool lorawan_joined_network(void)
{
	return joinedOTAA;
}

void lorawan_set_status(uint8_t status)
{
	BEEP_protocol_s prot;
	lorawanIsEnabled = (status & BITMASK_ENABLE) ?		true : false;
    dutycycleOn		 = (status & BITMASK_DUTYCYCLE) ?	true : false;
    adrEnable		 = (status & BITMASK_ADR) ?			true : false;
    lorawanReset	 = true;

	// Store the LoRaWAN configuration
	prot.command = READ_LORAWAN_STATE;
	prot.param.status.statusflag = status;
    nvm_fds_eeprom_set(&prot);
}

uint8_t Lorawan_get_status(void)
{
	uint8_t status = 0;

	status |= (lorawanIsEnabled) ?			BITMASK_ENABLE		: 0;
    status |= (lorawan_joined_network()) ?	BITMASK_JOINED		: 0;
	status |= (dutycycleOn) ?				BITMASK_DUTYCYCLE	: 0;
	status |= (adrEnable) ?					BITMASK_ADR			: 0;
	status |= (validKeys) ?					BITMASK_VALID_KEYS	: 0;
	return status;
}

bool lorawanDownlinkResponseEmpty(void)
{
	return (downlinkResponseLenght == 0) ? true : false;
}

uint32_t lorawanDownlinkAppend(BEEP_protocol_s * prot)
{
	uint32_t err_code;
	if(prot == NULL)
	{
		return NRF_ERROR_INVALID_PARAM;	
	}

	// Clear the buffer if the message lenght is larger than the array size.
	if(downlinkResponseLenght >= BEEP_LORAWAN_MAX_LENGHT)
	{
		memset(downlinkResponseBuffer, 0, BEEP_LORAWAN_MAX_LENGHT);
		downlinkResponseLenght = 0;
	}

	err_code = beep_protocol_encode(true, prot, downlinkResponseBuffer, &downlinkResponseLenght, BEEP_LORAWAN_MAX_LENGHT);
	NRF_LOG_INFO("LoRaWAN Protocol Enc: cmd:%u/0x%02X, err=%u", prot->command, prot->command, err_code);
	return err_code;
}

void lorawanDownlinkResponseAppend(BEEP_CID cmd, uint32_t error_code)
{
	BEEP_protocol_s encode;
	encode.command					= RESPONSE;
	encode.param.error.ErrorCmd		= cmd;
    encode.param.error.errorCode	= error_code;

    beep_protocol_encode(true, &encode, downlinkResponseBuffer, &downlinkResponseLenght, BEEP_LORAWAN_MAX_LENGHT);
}


/**
 * Main application entry point.
 */
void lorawan_implementation_init(protocol_cb_p callback_p)
{
	uint8_t status = 0;
	BEEP_protocol_s prot;

	if(callback_p != NULL)
	{
		main_callback = callback_p;
	}


	// Clear response parameters.	
    downlinkResponseLenght = 0;
	memset(downlinkResponseBuffer, 0, BEEP_LORAWAN_MAX_LENGHT);

	#if !RFM_ENABLE
		return;
	#endif

	prot.command = READ_LORAWAN_DEVEUI;
	nvm_fds_eeprom_get(&prot);
	memcpy(DevEui, &prot.param.lorawan_key.data, prot.param.lorawan_key.lenght);

	prot.command = READ_LORAWAN_APPEUI;
	nvm_fds_eeprom_get(&prot);
	memcpy(JoinEui, &prot.param.lorawan_key.data, prot.param.lorawan_key.lenght);

	prot.command = READ_LORAWAN_APPKEY;
	nvm_fds_eeprom_get(&prot);
	memcpy(AppKey, &prot.param.lorawan_key.data, prot.param.lorawan_key.lenght);
	memcpy(NwkKey, &prot.param.lorawan_key.data, prot.param.lorawan_key.lenght);


    // Store the LoRaWAN configuration
	prot.command = READ_LORAWAN_STATE;
    nvm_fds_eeprom_get(&prot);
	status = prot.param.status.statusflag;

    lorawanIsEnabled	= (status & BITMASK_ENABLE) ?		true : false;
    dutycycleOn			= (status & BITMASK_DUTYCYCLE) ?	true : false;
    adrEnable			= (status & BITMASK_ADR) ?			true : false;
    joinedOTAA			= false;

    validKeys = lorawan_valid_Keys();
	NRF_LOG_INFO("%s LoRaWAN keys found in the EEPROM", validKeys ? "Valid" : "Invalid");
	NRF_LOG_FLUSH();

    macPrimitives.MacMcpsConfirm		= McpsConfirm;
    macPrimitives.MacMcpsIndication		= McpsIndication;
    macPrimitives.MacMlmeConfirm		= MlmeConfirm;
    macPrimitives.MacMlmeIndication		= MlmeIndication;
    macCallbacks.GetBatteryLevel		= BoardGetBatteryLevel;
    macCallbacks.GetTemperatureLevel	= NULL;
    macCallbacks.NvmContextChange		= NvmCtxMgmtEvent;
    macCallbacks.MacProcessNotify		= OnMacProcessNotify;

    retValLoRaMAC = LoRaMacInitialization( &macPrimitives, &macCallbacks, ACTIVE_REGION );
    if(retValLoRaMAC != LORAMAC_STATUS_OK)
    {
		NRF_LOG_INFO("LoRaMacInitialization failed: %u", retValLoRaMAC);
    }

    DeviceState = DEVICE_STATE_RESTORE;

    // Create an timer for join and tx intervals.
	ret = app_timer_create(&TxNextPacketTimer, APP_TIMER_MODE_SINGLE_SHOT, OnTxNextPacketTimerEvent);
	if(ret != NRF_SUCCESS)
	{
		app_timer_stop(TxNextPacketTimer);
	}

	// When the keys are invalid, disable LoRaWAN
	if(!validKeys)
	{
		lorawanIsEnabled = false;
	}
}




bool lorawan_busy(void)
{
	#if RFM_ENABLE
		return (DeviceState == DEVICE_STATE_SLEEP) ? false : true;
	#else
		return false;
	#endif
}


void lorawan_AppEnable(bool enabled)
{
	if(enabled)
	{
		AppEnabled	= true;
        DeviceState = DEVICE_STATE_JOIN;
	}
	else
	{
		app_timer_stop(TxNextPacketTimer);
		AppEnabled = false;
	}
}

/**
 * LoRaWAN while loop .
 */
void lorawan_implementation_while(void)
{
	#if !RFM_ENABLE
		return;
	#endif

	// Tick the RTC to execute callback in context of the main loop (in stead of the IRQ)
	TimerProcess( );

	// Process Radio IRQ
	if( Radio.IrqProcess != NULL )
	{
		Radio.IrqProcess( );
	}

	// Processes the LoRaMac events
	LoRaMacProcess( );

	switch( DeviceState )
	{
		case DEVICE_STATE_RESTORE:
		{
			// Try to restore from NVM and query the mac if possible.
			retNVM = NvmCtxMgmtRestore( );
			if( retNVM == NVMCTXMGMT_STATUS_SUCCESS )
			{
				NRF_LOG_INFO( "###### ===== NVM CTXS RESTORED ==== ######" );
			}
			else
			{
				#if( OVER_THE_AIR_ACTIVATION == 0 )
					// Tell the MAC layer which network server version are we connecting too.
					mibReq.Type = MIB_ABP_LORAWAN_VERSION;
					mibReq.Param.AbpLrWanVersion.Value = ABP_ACTIVATION_LRWAN_VERSION;
					LoRaMacMibSetRequestConfirm( &mibReq );
				#endif

				#if( ABP_ACTIVATION_LRWAN_VERSION == ABP_ACTIVATION_LRWAN_VERSION_V10x )
					mibReq.Type = MIB_GEN_APP_KEY;
					mibReq.Param.GenAppKey = GenAppKey;
					LoRaMacMibSetRequestConfirm( &mibReq );
				#else
					mibReq.Type = MIB_APP_KEY;
					mibReq.Param.AppKey = AppKey;
					LoRaMacMibSetRequestConfirm( &mibReq );
				#endif

				mibReq.Type = MIB_NWK_KEY;
				mibReq.Param.NwkKey = NwkKey;
				LoRaMacMibSetRequestConfirm( &mibReq );

				// Initialize LoRaMac device unique ID if not already defined in Commissioning.h
				#ifndef LORAWAN_DEVICE_EUI
					if( ( DevEui[0] == 0 ) && ( DevEui[1] == 0 ) &&
						( DevEui[2] == 0 ) && ( DevEui[3] == 0 ) &&
						( DevEui[4] == 0 ) && ( DevEui[5] == 0 ) &&
						( DevEui[6] == 0 ) && ( DevEui[7] == 0 ) )
					{
						BoardGetUniqueId( DevEui );
					}
				#endif

				#if( OVER_THE_AIR_ACTIVATION == 0 )
					// Choose a random device address if not already defined in Commissioning.h
					if( DevAddr == 0 )
					{
						// Random seed initialization
						srand1( BoardGetRandomSeed( ) );

						// Choose a random device address
						DevAddr = randr( 0, 0x01FFFFFF );
					}

					mibReq.Type = MIB_NET_ID;
					mibReq.Param.NetID = LORAWAN_NETWORK_ID;
					LoRaMacMibSetRequestConfirm( &mibReq );

					mibReq.Type = MIB_DEV_ADDR;
					mibReq.Param.DevAddr = DevAddr;
					LoRaMacMibSetRequestConfirm( &mibReq );

					mibReq.Type = MIB_F_NWK_S_INT_KEY;
					mibReq.Param.FNwkSIntKey = FNwkSIntKey;
					LoRaMacMibSetRequestConfirm( &mibReq );

					mibReq.Type = MIB_S_NWK_S_INT_KEY;
					mibReq.Param.SNwkSIntKey = SNwkSIntKey;
					LoRaMacMibSetRequestConfirm( &mibReq );

					mibReq.Type = MIB_NWK_S_ENC_KEY;
					mibReq.Param.NwkSEncKey = NwkSEncKey;
					LoRaMacMibSetRequestConfirm( &mibReq );

					mibReq.Type = MIB_APP_S_KEY;
					mibReq.Param.AppSKey = AppSKey;
					LoRaMacMibSetRequestConfirm( &mibReq );
				#endif
			}
			DeviceState = DEVICE_STATE_START;
			break;
		}

		case DEVICE_STATE_START:
		{
			mibReq.Type = MIB_PUBLIC_NETWORK;
			mibReq.Param.EnablePublicNetwork = LORAWAN_PUBLIC_NETWORK;
			LoRaMacMibSetRequestConfirm( &mibReq );

			mibReq.Type = MIB_ADR;
			mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
			LoRaMacMibSetRequestConfirm( &mibReq );

			#if defined( REGION_EU868 ) || defined( REGION_RU864 ) || defined( REGION_CN779 ) || defined( REGION_EU433 )
				LoRaMacTestSetDutyCycleOn( dutycycleOn );

				#if( USE_TTN_NETWORK == 1 )
					mibReq.Type = MIB_RX2_DEFAULT_CHANNEL;
					mibReq.Param.Rx2DefaultChannel = ( RxChannelParams_t ){ 869525000, DR_3 };
					LoRaMacMibSetRequestConfirm( &mibReq );

					mibReq.Type = MIB_RX2_CHANNEL;
					mibReq.Param.Rx2Channel = ( RxChannelParams_t ){ 869525000, DR_3 };
					LoRaMacMibSetRequestConfirm( &mibReq );

					mibReq.Type = MIB_RXC_DEFAULT_CHANNEL;
					mibReq.Param.RxCDefaultChannel = ( RxChannelParams_t ){ 869525000, DR_3 };
					LoRaMacMibSetRequestConfirm( &mibReq );

					mibReq.Type = MIB_RXC_CHANNEL;
					mibReq.Param.RxCChannel = ( RxChannelParams_t ){ 869525000, DR_3 };
					LoRaMacMibSetRequestConfirm( &mibReq );
				#endif
			#endif

			mibReq.Type = MIB_SYSTEM_MAX_RX_ERROR;
			mibReq.Param.SystemMaxRxError = 20;
			LoRaMacMibSetRequestConfirm( &mibReq );

			LoRaMacStart( );

			mibReq.Type = MIB_NETWORK_ACTIVATION;
			status = LoRaMacMibGetRequestConfirm( &mibReq );

			if( status == LORAMAC_STATUS_OK )
			{
				if( mibReq.Param.NetworkActivation == ACTIVATION_TYPE_NONE )
				{
					DeviceState = DEVICE_STATE_JOIN;
				}
				else
				{
					DeviceState = DEVICE_STATE_SEND;
					NextTx = true;
				}
			}
			break;
		}
		case DEVICE_STATE_JOIN:
		{
			NRF_LOG_FLUSH();
			NRF_LOG_INFO("DEVICE_STATE_JOIN");
			printStringWithHexArray("DevEui      : ", 8, DevEui);
			printStringWithHexArray("AppEui      : ", 8, JoinEui);
			printStringWithHexArray("AppKey      : ", 16, NwkKey);

			#if( OVER_THE_AIR_ACTIVATION == 0 )
				NRF_LOG_INFO( "###### ===== ABP JOINED ==== ######\r\n" );
				NRF_LOG_INFO( "\r\nABP\r\n\r\n" );
				NRF_LOG_INFO( "DevAddr     : %08lX\r\n", DevAddr );
				printStringWithHexArray("NwkSKey      : ", 16, FNwkSIntKey);
				printStringWithHexArray("AppSKey      : ", 16, AppSKey);

				mibReq.Type = MIB_NETWORK_ACTIVATION;
				mibReq.Param.NetworkActivation = ACTIVATION_TYPE_ABP;
				LoRaMacMibSetRequestConfirm( &mibReq );

				DeviceState = DEVICE_STATE_SEND;
			#else
				// Check if the keys are valid and the top level application has enabled the LoRaWAN functionality
				if(lorawanIsEnabled && AppEnabled)
				{
					JoinNetwork( );
				}
				else
				{
					DeviceState = DEVICE_STATE_SLEEP;
				}
				
			#endif
			break;
		}
		case DEVICE_STATE_SEND:
		{
			DeviceState = DEVICE_STATE_CYCLE;
			break;
		}
		case DEVICE_STATE_CYCLE:
		{

			DeviceState = DEVICE_STATE_SLEEP;
			if( ComplianceTest.Running == true )
			{
				// Schedule next packet transmission
				TxDutyCycleTime = 5000; // 5000 ms
			}
			else
			{
				mibReq.Type = MIB_NETWORK_ACTIVATION;
				status = LoRaMacMibGetRequestConfirm( &mibReq );

				if( status == LORAMAC_STATUS_OK )
				{
					if( mibReq.Param.NetworkActivation == ACTIVATION_TYPE_NONE )
					{
						// Network not joined yet. Try to join again
						TxDutyCycleTime = GetJoinInterval();
					}
					else
					{
						// If joined schedule the next packet transmission
						TxDutyCycleTime = app_tx_interval + randr( -app_tx_interval, app_tx_interval );
					}
				}
				else
				{
					// Error case
					TxDutyCycleTime = app_tx_interval;
				}
			}

			// Schedule next packet transmission
			app_timer_stop(TxNextPacketTimer);
			app_timer_start(TxNextPacketTimer, APP_TIMER_TICKS(TxDutyCycleTime), NULL);
			break;
		}
		case DEVICE_STATE_SLEEP:
		{
			if( NvmCtxMgmtStore( ) == NVMCTXMGMT_STATUS_SUCCESS )
			{
				NRF_LOG_INFO( "###### ===== CTXS STORED ==== ######" );
			}


			// Check if the LoRaWAN statemachine must be reset.
			if(lorawanReset)
			{
                NRF_LOG_INFO("LoRaWAN reset");
                NRF_LOG_FLUSH();
				lorawanReset = false;

				// Re-init the lorawan stack.
                lorawan_implementation_init(NULL);
			}
			else if( IsMacProcessPending == 1 )
			{
				// Clear flag and prevent MCU to go into low power modes.
				IsMacProcessPending = 0;
			}
			else if(lorawanIsEnabled && AppEnabled)
			{
				#if 0
				if(msgtTrigger != BEEP_UNKNOWN)
				{
					if(SendFrame(msgtTrigger) == LORAMAC_STATUS_OK)
					{
						// Clear the trigger when the message is send succesfully.
						msgtTrigger = BEEP_UNKNOWN; 

						TxDutyCycleTime = app_tx_interval + randr( -app_tx_interval, app_tx_interval );
						app_timer_stop(TxNextPacketTimer);
						app_timer_start(TxNextPacketTimer, APP_TIMER_TICKS(app_tx_interval), NULL );
					}
				}
				#endif
			}
			break;
		}
		default:
		{
			DeviceState = DEVICE_STATE_START;
			break;
		}
	}
}
