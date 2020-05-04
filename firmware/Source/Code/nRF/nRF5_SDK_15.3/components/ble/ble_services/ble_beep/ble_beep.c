
#include "ble_beep.h"
#include <string.h>
#include "nordic_common.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "ble_setup.h"

#define NRF_LOG_MODULE_NAME BEEP_SERVICE
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#define MAX_CSCM_LEN  (BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH)    /**< Maximum size of a transmitted Cycling Speed and Cadence Measurement. */

#define BLE_BUS_LOG_ENABLED 1


/**@brief check if the cccd is configured
 *
 * @param[in]   p_sc_ctrlpt      SC Ctrlpt structure.
 * @return  true if the sc_control point's cccd is correctly configured, false otherwise.
 */
static bool is_cccd_configured(uint16_t conn_handle, uint16_t cccd_handle)
{
    uint32_t err_code;
    uint8_t  cccd_value_buf[BLE_CCCD_VALUE_LEN];
    bool     is_sccp_indic_enabled = false;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = BLE_CCCD_VALUE_LEN;
    gatts_value.offset  = 0;
    gatts_value.p_value = cccd_value_buf;

    err_code = sd_ble_gatts_value_get(conn_handle, cccd_handle, &gatts_value);
    if (err_code != NRF_SUCCESS)
    {

    }

    is_sccp_indic_enabled = ble_srv_is_indication_enabled(cccd_value_buf);

    return is_sccp_indic_enabled;
}





/**@brief sends a control point indication.
 *
 * @param[in]   p_sc_ctrlpt      SC Ctrlpt structure.
 */
static uint32_t beep_ctrlpt_send(ble_beep_t * p_beep)
{
    uint16_t               hvx_len;
    ble_gatts_hvx_params_t hvx_params;
    uint32_t               err_code;

	if(p_beep->control_rsp.status != BLE_CONTROL_INDICATION_PENDING)
	{
		return NRF_ERROR_INVALID_STATE;
	}

	hvx_len = p_beep->control_rsp.len;
	memset(&hvx_params, 0, sizeof(hvx_params));

	hvx_params.handle = p_beep->control_char.value_handle; // p_sc_ctrlpt->sc_ctrlpt_handles.value_handle;
	hvx_params.type   = BLE_GATT_HVX_INDICATION;
	hvx_params.offset = 0;
	hvx_params.p_len  = &hvx_len;
	hvx_params.p_data = p_beep->control_rsp.encoded_ctrl_rsp; //reponse;

	err_code = sd_ble_gatts_hvx(p_beep->conn_handle, &hvx_params);

	// Error handling
	if ((err_code == NRF_SUCCESS) && (hvx_len != p_beep->control_rsp.len))
	{
		err_code = NRF_ERROR_DATA_SIZE;
	}

	switch (err_code)
	{
		case NRF_SUCCESS:
			p_beep->control_rsp.status = BLE_CONTROL_IND_CONFIRM_PENDING;
			// Wait for HVC event
			break;

		case NRF_ERROR_RESOURCES:
			// Wait for TX_COMPLETE event to retry transmission.
			p_beep->control_rsp.status = BLE_CONTROL_INDICATION_PENDING;
			break;

		default:
			// Report error to application.
			p_beep->control_rsp.status = BLE_CONTROL_NO_PROC_IN_PROGRESS;
			break;
	}
	return err_code;
}

uint32_t beep_ctrlpt_send_error(ble_beep_t * p_beep, BEEP_CID cmd, uint32_t error)
{
	BEEP_protocol_s encode;

	encode.command					= RESPONSE;
	encode.param.error.ErrorCmd		= cmd;
    encode.param.error.errorCode	= error;
	p_beep->control_rsp.len = 0;
    if(!beep_protocol_encode(true, &encode, p_beep->control_rsp.encoded_ctrl_rsp, &p_beep->control_rsp.len, BEEP_MAX_LENGHT))
	{
		beep_ctrlpt_send(p_beep);
	}
}


#define SC_CTRLPT_NACK_PROC_ALREADY_IN_PROGRESS   (BLE_GATT_STATUS_ATTERR_APP_BEGIN + 0)
#define SC_CTRLPT_NACK_CCCD_IMPROPERLY_CONFIGURED (BLE_GATT_STATUS_ATTERR_APP_BEGIN + 1)

/**@brief Handle a write event to the Speed and Cadence Control Point.
 *
 * @param[in]   p_beep      SC Ctrlpt structure.
 * @param[in]   p_evt_write      WRITE event to be handled.
 */
static void on_ctrlpt_write(ble_beep_t * p_beep, ble_gatts_evt_write_t const * p_evt_write)
{
	uint8_t									offset = 0;
    uint32_t								err_code;
    ble_gatts_rw_authorize_reply_params_t	auth_reply;

    auth_reply.type                     = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
    auth_reply.params.write.offset      = 0;
    auth_reply.params.write.len         = 0;
    auth_reply.params.write.p_data      = NULL;
    auth_reply.params.write.gatt_status = BLE_GATT_STATUS_SUCCESS;
    auth_reply.params.write.update      = 1;

    if (is_cccd_configured(p_beep->conn_handle, p_beep->control_char.cccd_handle))
    {
		// Check if the control point is still attempting to finish a previous command
        if (p_beep->control_rsp.status == BLE_CONTROL_NO_PROC_IN_PROGRESS)
        {
            auth_reply.params.write.gatt_status = BLE_GATT_STATUS_SUCCESS;
        }
        else
        {
            auth_reply.params.write.gatt_status = SC_CTRLPT_NACK_PROC_ALREADY_IN_PROGRESS;
        }
    }
    else
    {
        auth_reply.params.write.gatt_status = SC_CTRLPT_NACK_CCCD_IMPROPERLY_CONFIGURED;
    }

    err_code = sd_ble_gatts_rw_authorize_reply(p_beep->conn_handle, &auth_reply);
    if (err_code != NRF_SUCCESS)
    {

    }

    if (auth_reply.params.write.gatt_status != BLE_GATT_STATUS_SUCCESS)
    {
        return;
    }

    p_beep->control_rsp.status	= BLE_CONTROL_INDICATION_PENDING;

	NRF_LOG_INFO("BLE control point lenght: %u", p_evt_write->len);
	NRF_LOG_HEXDUMP_INFO(p_evt_write->data, p_evt_write->len);

	// Decode the message received
    BEEP_protocol_s decode;
   
    err_code = beep_protocol_decode(&decode, (uint8_t *)p_evt_write->data, p_evt_write->len, &offset);
    if(err_code != NRF_SUCCESS)
    {
		beep_ctrlpt_send_error(p_beep, decode.command, err_code);
    }
    else
    {
		// Call the callback
		if(p_beep->callback_p != NULL)
		{
            decode.source = BLE_SOURCE;
			p_beep->callback_p(BLE_SOURCE, &decode);
		}
	}
}




/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_beep      Cycling Speed and Cadence Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_beep_t * p_beep, ble_evt_t const * p_ble_evt)
{
    p_beep->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    p_beep->control_rsp.status = BLE_CONTROL_NO_PROC_IN_PROGRESS;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_beep      Cycling Speed and Cadence Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_beep_t * p_beep, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_beep->conn_handle = BLE_CONN_HANDLE_INVALID;
    p_beep->control_rsp.status = BLE_CONTROL_NO_PROC_IN_PROGRESS;

    if(p_beep->isRX_notification_enabled || p_beep->isTX_notification_enabled)
    {
        p_beep->isRX_notification_enabled    = false;
        p_beep->isTX_notification_enabled    = false;
        p_beep->txLenght                     = 0;
        p_beep->txMsgLenght                  = 0;
        p_beep->packetCounter                = NULL;
        p_beep->txEventHandler(APP_FLASH_COMM_DISCONNECTED, NULL);
    }
}


/**@brief Function for handling write events to the beep Measurement characteristic.
 *
 * @param[in]   p_beep        Cycling Speed and Cadence Service structure.
 * @param[in]   p_evt_write   Write event received from the BLE stack.
 */
static void on_meas_cccd_write(ble_beep_t * p_beep, ble_gatts_evt_write_t const * p_evt_write)
{
    if (p_evt_write->len == 2)
    {
        // CCCD written, update notification state
        if (p_beep->evt_handler != NULL)
        {
            ble_beep_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_beep_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_beep_EVT_NOTIFICATION_DISABLED;
            }

            p_beep->evt_handler(p_beep, &evt);
        }
    }
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_beep      Cycling Speed and Cadence Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_beep_t * p_beep, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if ((p_evt_write->handle == p_beep->tx_handles.cccd_handle) && (p_evt_write->len == 2))
    {
        p_beep->isTX_notification_enabled = ble_srv_is_notification_enabled(p_evt_write->data);
        #if BLE_BUS_LOG_ENABLED
            NRF_LOG_INFO("TX notification: %s, change Err: 0x%04X/%u", (p_beep->isTX_notification_enabled) ? "On" : "Off");
        #endif

        // On TX notification change
        p_beep->txEventHandler((p_beep->isTX_notification_enabled) ? APP_FLASH_COMM_STARTED : APP_FLASH_COMM_STOPPED, NULL);
    }
    else if ((p_evt_write->handle == p_beep->control_char.cccd_handle) && (p_evt_write->len == 2))
    {
        p_beep->isRX_notification_enabled = ble_srv_is_indication_enabled(p_evt_write->data);
        #if BLE_BUS_LOG_ENABLED
            NRF_LOG_INFO("RX notification: %s", (p_beep->isRX_notification_enabled) ? "On" : "Off");
        #endif
    }
    else
    {
        // Do Nothing. This event is not relevant for this service.
    }
}

/**@brief Authorize WRITE request event handler.
 *
 * @details Handles WRITE events from the BLE stack.
 *
 * @param[in]   p_sc_ctrlpt  SC Ctrlpt structure.
 * @param[in]   p_gatts_evt  GATTS Event received from the BLE stack.
 *
 */
static void on_rw_authorize_request(ble_beep_t * p_beep,  ble_gatts_evt_t const * p_gatts_evt)
{
    ble_gatts_evt_rw_authorize_request_t const * p_auth_req = &p_gatts_evt->params.authorize_request;

    if (p_auth_req->type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
    {
        if (   (p_gatts_evt->params.authorize_request.request.write.op != BLE_GATTS_OP_PREP_WRITE_REQ)
            && (p_gatts_evt->params.authorize_request.request.write.op != BLE_GATTS_OP_EXEC_WRITE_REQ_NOW)
            && (p_gatts_evt->params.authorize_request.request.write.op != BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL)
           )
        {
            if (p_auth_req->request.write.handle == p_beep->control_char.value_handle) //  ->sc_ctrlpt_handles.value_handle
            {
                on_ctrlpt_write(p_beep, &p_auth_req->request.write);
            }
        }
    }
}



static void on_sc_hvc_confirm(ble_beep_t * p_beep, ble_evt_t const * p_ble_evt)
{
    if (p_ble_evt->evt.gatts_evt.params.hvc.handle == p_beep->control_char.value_handle)
    {
        if (p_beep->control_rsp.status == BLE_CONTROL_IND_CONFIRM_PENDING)
        {
            p_beep->control_rsp.status = BLE_CONTROL_NO_PROC_IN_PROGRESS;
        }
    }
}

static void on_hvx_tx_complete(ble_beep_t * p_beep, ble_evt_t const * p_ble_evt)
{
    ret_code_t err_code;

    if (p_beep->control_rsp.status == BLE_CONTROL_INDICATION_PENDING)
    {
        beep_ctrlpt_send(p_beep);
    }
    
    /*
     * Check if the notifications are enabled and if the message lenght is not zero and the message lenght is not zero.
     * Since there's no source indication with the HVX event there's no way to check if the transmission of the flash 
     * data was succesfull without adding another layer of flow control.
     */
    if (p_beep->isTX_notification_enabled && p_beep->txLenght != 0 && p_beep->txMsgLenght != 0)
    {
        // Check whether data has to be transmitted or if the buffer is empty  
        if((p_beep->txOffset < p_beep->txLenght) && (p_beep->txLenght != 0))
        {
            err_code = ble_beep_send_TXdata_fragment(p_beep);
            #if BLE_BUS_LOG_ENABLED
                NRF_LOG_INFO("HVX[%u]_fragment", *p_beep->packetCounter);
            #endif
        }
        else
        {
            p_beep->txLenght         = 0;
            p_beep->txMsgLenght      = 0;
            p_beep->packetCounter    = NULL; 
            p_beep->txEventHandler(APP_TX_RDY_FLASH, NULL);
        }
    }
}




void ble_beep_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_beep_t * p_beep = (ble_beep_t *)p_context;

    if (p_beep == NULL || p_ble_evt == NULL)
    {
        return;
    }


    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_beep, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_beep, p_ble_evt);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
            on_rw_authorize_request(p_beep, &p_ble_evt->evt.gatts_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_beep, p_ble_evt);
            break;

        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
            on_hvx_tx_complete(p_beep, p_ble_evt);
            break;

		case BLE_GATTS_EVT_HVC:
            on_sc_hvc_confirm(p_beep, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}



static uint32_t ble_beep_tx_char(ble_beep_t * service, ble_beep_init_t const * service_init)
{
   ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = service->uuid_type;
    ble_uuid.uuid = BEEP_TX_UUID;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_NUS_MAX_MSG_LEN;

    return sd_ble_gatts_characteristic_add(service->service_handle, &char_md, &attr_char_value, &service->tx_handles);
}




static uint32_t ble_beep_ds18B20_char(ble_beep_t * service, ble_beep_init_t const * service_init)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_md_t user_desc_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

	if(service == NULL || service_init == NULL)
	{
		return NRF_ERROR_INVALID_PARAM;
	}

    // Add Custom Value characteristic
    memset(&cccd_md, 0, sizeof(cccd_md));

    //  Read  operation on cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.write_perm	= service_init->security_level.cccd_write_perm;
    cccd_md.vloc		= BLE_GATTS_VLOC_STACK;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&user_desc_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&user_desc_md.write_perm);
    user_desc_md.vloc	= BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read			= 1;
    char_md.char_props.write		= 0;
    char_md.char_props.notify		= 1;
    char_md.p_char_user_desc		= "DS18B20 Temperature";
	char_md.char_user_desc_size		= strlen(char_md.p_char_user_desc);
	char_md.char_user_desc_max_size = char_md.char_user_desc_size;
    char_md.p_char_pf				= NULL;
    char_md.p_user_desc_md			= &user_desc_md;
    char_md.p_cccd_md				= &cccd_md; 
    char_md.p_sccd_md				= NULL;
		
    ble_uuid.type					= service->uuid_type;
    ble_uuid.uuid					= BEEP_DS18B20_UUID;

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm				= service_init->security_level.read_perm;
    attr_md.write_perm				= service_init->security_level.write_perm;
    attr_md.vloc					= BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth					= 0;
    attr_md.wr_auth					= 0;
    attr_md.vlen					= 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid			= &ble_uuid;
    attr_char_value.p_attr_md		= &attr_md;
    attr_char_value.init_offs		= 0;
    attr_char_value.max_len			= 20;
    attr_char_value.init_len		= 0;
    attr_char_value.p_value			= NULL;
	
    err_code = sd_ble_gatts_characteristic_add(service->service_handle, &char_md, &attr_char_value, &service->ds18B20_char);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}


uint32_t ble_beep_ctrlpt_init(ble_beep_t * p_beep, ble_beep_init_t const * service_init)
{
    if (p_beep == NULL || service_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    ble_add_char_params_t	add_char_params;  

    // Add Control Point characteristic
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid					= BEEP_CP_UUID;
    add_char_params.max_len					= BEEP_MAX_LENGHT;
    add_char_params.is_var_len				= true;
    add_char_params.char_props.indicate		= 1;
    add_char_params.char_props.write		= 1;
    add_char_params.cccd_write_access		= SEC_OPEN;
    add_char_params.write_access			= SEC_OPEN;
    add_char_params.is_defered_write		= true;

    return characteristic_add(p_beep->service_handle, &add_char_params, &p_beep->control_char);
}



uint32_t ble_beep_init(ble_beep_t * p_beep, ble_beep_init_t const * p_beep_init)
{
    if (p_beep == NULL || p_beep_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t              err_code;
    ble_uuid_t			  service_uuid;
    ble_add_char_params_t add_char_params;
    

    // Initialize service structure
    p_beep->evt_handler		= p_beep_init->evt_handler;
    p_beep->conn_handle		= BLE_CONN_HANDLE_INVALID;
    p_beep->txEventHandler  = p_beep_init->txCallback;
    p_beep->callback_p      = p_beep_init->callback_p;

    // Add service UUID to the database
	ble_uuid128_t beep_uuid = BEEP_SERVICE_UUID_BASE;
	err_code				= sd_ble_uuid_vs_add(&beep_uuid, &p_beep->uuid_type);
    service_uuid.type		= p_beep->uuid_type;
    service_uuid.uuid		= BEEP_SERVICE_UUID;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &service_uuid, &p_beep->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

	// Data characteristic DS18B20
	err_code = ble_beep_ds18B20_char(p_beep, p_beep_init);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}

    // Add TX characteristic
    err_code = ble_beep_tx_char(p_beep, p_beep_init);
    if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}

	// Add control point characteristic
    err_code = ble_beep_ctrlpt_init(p_beep, p_beep_init);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}

    return NRF_SUCCESS;
}


static bool ble_beep_measurment_encode(ble_beep_t * p_beep, uint16_t * handle, MEASUREMENT_RESULT_s * const measurement, uint8_t * data, uint16_t * lenght)
{
	uint16_t len = 0, offset = 0;;


	if(p_beep == NULL || handle == NULL || measurement == NULL || data == NULL || lenght == NULL)
	{
		return NRF_ERROR_NULL;
	}


	// Depending on the measurement type encode the data differently.
	switch(measurement->type)
	{
		//------------------------------------------------------------------------------------------------------------------
		case DS18B20:
		{
			uint8_t i;
			DS18B20_RESULTS_s * ds = &measurement->result.ds18B20;

			if(ds->devices == 0)
			{
				return NRF_ERROR_INVALID_LENGTH;
			}

			if(ds->devices > MAX_TEMP_SENSORS)
			{
				ds->devices = MAX_TEMP_SENSORS;
			}
			data[offset] = ds->devices;

			offset = 1;
			for(i = 0; i < ds->devices; i++)
			{
				offset += uint16_big_encode((uint16_t)ds->temperatures[i], &data[offset]);
			}
			*handle = p_beep->ds18B20_char.value_handle;
			*lenght = offset;
			break;
		}

        //------------------------------------------------------------------------------------------------------------------
		case BME280:
		case HX711:
		case AUDIO_ADC:
		case nRF_ADC:
		
        //------------------------------------------------------------------------------------------------------------------
		default:
			*handle = NULL;
			*lenght = 0;
			return NRF_ERROR_INVALID_STATE;
			break;
	
	}

	return NRF_SUCCESS;
}


uint32_t ble_beep_controlPoint_send(ble_beep_t * p_beep, BEEP_protocol_s * prot)
{
	uint32_t err_code;

    if (p_beep == NULL || prot == NULL)
    {
        return NRF_ERROR_NULL;
    }
    p_beep->control_rsp.len = 0;

    err_code = beep_protocol_encode(true, prot, p_beep->control_rsp.encoded_ctrl_rsp, &p_beep->control_rsp.len, BEEP_MAX_LENGHT);
	NRF_LOG_INFO("Protocol Encode: cmd:%u/0x%02X, err=%u, bytes:%u", prot->command, prot->command, err_code, p_beep->control_rsp.len);
	NRF_LOG_HEXDUMP_INFO(p_beep->control_rsp.encoded_ctrl_rsp, p_beep->control_rsp.len);
	
	if(err_code == NRF_SUCCESS)
	{
		p_beep->control_rsp.status = BLE_CONTROL_INDICATION_PENDING;
		err_code = beep_ctrlpt_send(p_beep);
	}

	return err_code;
}





uint32_t ble_beep_measurement_send(ble_beep_t * p_beep, MEASUREMENT_RESULT_s * measurement)
{
    if (p_beep == NULL || measurement == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t				err_code;
	uint8_t					encoded_meas[BEEP_MAX_LENGHT];
	uint16_t				len;
	uint16_t				hvx_len;
	ble_gatts_hvx_params_t	hvx_params;

	err_code = ble_beep_measurment_encode(p_beep, &hvx_params.handle, measurement, encoded_meas, &len);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}

	// Send value if connected and notifying, otherwise update the value in the BLE stack.
	if (p_beep->conn_handle != BLE_CONN_HANDLE_INVALID)
	{
		hvx_len = len;
		hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
		hvx_params.offset = 0;
		hvx_params.p_len  = &hvx_len;
		hvx_params.p_data = encoded_meas;

		err_code = sd_ble_gatts_hvx(p_beep->conn_handle, &hvx_params);
		if ((err_code == NRF_SUCCESS) && (hvx_len != len))
		{
			err_code = NRF_ERROR_DATA_SIZE;
		}
	}
	else
	{
		ble_gatts_value_t gatts_value;
		gatts_value.len		= len;
		gatts_value.offset	= 0;
		gatts_value.p_value	= encoded_meas;
		sd_ble_gatts_value_set(BLE_CONN_HANDLE_INVALID, hvx_params.handle, &gatts_value);
	}

    return err_code;
}



uint32_t ble_beep_TXdata_send(ble_beep_t * p_nus, uint16_t * buffCount, uint8_t * p_data, uint16_t length)
{
    ret_code_t err_code;

    VERIFY_PARAM_NOT_NULL(p_nus);
    VERIFY_PARAM_NOT_NULL(p_data);

    if (p_nus->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    if(length == 0 || p_nus->dataLenghtMax == 0)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    p_nus->txData_p         = p_data;
    p_nus->txLenght         = length;
    p_nus->txOffset         = 0;
    p_nus->packetCounter    = buffCount;
    p_nus->mtuDatalenght    = BLE_NUS_MAX_DATA_LEN;


    return ble_beep_send_TXdata_fragment(p_nus);
}


uint32_t ble_beep_send_TXdata_fragment(ble_beep_t * p_nus)
{
    ret_code_t                  err_code = NRF_SUCCESS;
    ble_gatts_hvx_params_t      hvx_params;
    static uint8_t              msgData [NRF_SDH_BLE_GATT_MAX_MTU_SIZE] = {0};
    static uint16_t             lenght;

    VERIFY_PARAM_NOT_NULL(p_nus);

    while(err_code == NRF_SUCCESS)
    {
        if ((p_nus->conn_handle == BLE_CONN_HANDLE_INVALID))
        {
            return NRF_ERROR_NOT_FOUND;
        }

        if (!p_nus->isTX_notification_enabled)
        {
            return NRF_ERROR_INVALID_STATE;
        }

        if((p_nus->txOffset >= p_nus->txLenght) || (p_nus->txLenght == 0))
        {
            return NRF_ERROR_INVALID_LENGTH;
        }

        p_nus->txMsgLenght = p_nus->txLenght - p_nus->txOffset;
        if(p_nus->txMsgLenght > p_nus->mtuDatalenght)
        {
            p_nus->txMsgLenght = p_nus->mtuDatalenght;
        }

        memset(&hvx_params, 0, sizeof(hvx_params));

        memset(msgData, 0, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
        uint16_big_encode((*p_nus->packetCounter), msgData);
        memcpy(&msgData[2], &p_nus->txData_p[p_nus->txOffset], p_nus->txMsgLenght);
    
    
        lenght = p_nus->txMsgLenght + 2;
        hvx_params.handle = p_nus->tx_handles.value_handle;
        hvx_params.p_data = msgData;
        hvx_params.p_len  = &lenght;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;

        err_code = sd_ble_gatts_hvx(p_nus->conn_handle, &hvx_params);
        NRF_LOG_INFO("ble_bus_send_fragment[%u]: len=%u, err: %u", (*p_nus->packetCounter), lenght, err_code);

        if(err_code == NRF_SUCCESS)
        {
            if(p_nus->packetCounter != NULL)
            {
                (*p_nus->packetCounter)++;
            }

            p_nus->txOffset += p_nus->txMsgLenght;

            // check if there' still data to be transmitted.
            if(p_nus->txOffset >= p_nus->txLenght)
            {
                NRF_LOG_INFO("Done!");
                return err_code;
            }
        }
    }

    #if 0
        NRF_LOG_INFO("Escape!");
    #endif
    return err_code;
}




