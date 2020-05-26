/**
 * Copyright (c) 2012 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "sdk_common.h"
#include "ble.h"
#include "ble_bus.h"
#include "ble_srv_common.h"
#include "beep_protocol.h"
#include "app_util.h"
#include "ble_conn_params.h"

#define NRF_LOG_MODULE_NAME ble_bus
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();


#define BLE_UUID_NUS_TX_CHARACTERISTIC 0x0003               /**< The UUID of the TX Characteristic. */
#define BLE_UUID_NUS_RX_CHARACTERISTIC 0x0002               /**< The UUID of the RX Characteristic. */

#define BLE_NUS_MAX_RX_CHAR_LEN        BLE_NUS_MAX_DATA_LEN /**< Maximum length of the RX Characteristic (in bytes). */
#define BLE_NUS_MAX_TX_CHAR_LEN        BLE_NUS_MAX_DATA_LEN /**< Maximum length of the TX Characteristic (in bytes). */

#define NUS_BASE_UUID                  {{0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x00, 0x00, 0x40, 0x6E}} /**< Used vendor specific UUID. */
#define BLE_NUS_MAX_MSG_LEN 23
#define BLE_NUS_MAX_DATA_LEN 20

/**@brief Function for handling the @ref BLE_GAP_EVT_CONNECTED event from the SoftDevice.
 *
 * @param[in] p_nus     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_connect(ble_bus_t * p_nus, ble_evt_t const * p_ble_evt)
{
    ret_code_t                 err_code;
    ble_gatts_value_t          gatts_val;
    uint8_t                    cccd_value[2];
    ble_bus_client_context_t * p_client = NULL;

    err_code = blcm_link_ctx_get(p_nus->p_link_ctx_storage,  p_ble_evt->evt.gap_evt.conn_handle, (void *) &p_client);
    if (err_code != NRF_SUCCESS)
    {
        #if BLE_BUS_LOG_ENABLED
            NRF_LOG_ERROR("Link context for 0x%02X connection handle could not be fetched.", p_ble_evt->evt.gap_evt.conn_handle);
        #endif
    }

    /* Check the hosts CCCD value to inform of readiness to send data using the RX characteristic */
    memset(&gatts_val, 0, sizeof(ble_gatts_value_t));
    gatts_val.p_value = cccd_value;
    gatts_val.len     = sizeof(cccd_value);
    gatts_val.offset  = 0;

    err_code = sd_ble_gatts_value_get(p_ble_evt->evt.gap_evt.conn_handle, p_nus->rx_handles.cccd_handle, &gatts_val);
    p_nus->data_handler(BLE_NUS_EVT_COMM_CONNECTED);
}


static void on_disconnect(ble_bus_t * p_nus, ble_evt_t const * p_ble_evt)
{
    ret_code_t                 err_code;

    p_nus->conn_handle = BLE_CONN_HANDLE_INVALID;

    if(p_nus->isRX_notification_enabled || p_nus->isTX_notification_enabled)
    {
        p_nus->isRX_notification_enabled    = false;
        p_nus->isTX_notification_enabled    = false;
        p_nus->txLenght                     = 0;
        p_nus->txMsgLenght                  = 0;
        p_nus->packetCounter                = 0;
        p_nus->packetMax                    = 0; 
        p_nus->data_handler(BLE_NUS_EVT_COMM_DISCONNECTED);
    }
}


/**@brief Function for handling the @ref BLE_GATTS_EVT_WRITE event from the SoftDevice.
 *
 * @param[in] p_nus     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_write(ble_bus_t * p_nus, ble_evt_t const * p_ble_evt)
{
    ret_code_t                    err_code;
    ble_bus_client_context_t    * p_client;
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    err_code = blcm_link_ctx_get(p_nus->p_link_ctx_storage, p_ble_evt->evt.gatts_evt.conn_handle, (void *) &p_client);
    if (err_code != NRF_SUCCESS)
    {
        #if BLE_BUS_LOG_ENABLED
            NRF_LOG_ERROR("Link context for 0x%02X connection handle could not be fetched.", p_ble_evt->evt.gatts_evt.conn_handle);
        #endif
    }

    if ((p_evt_write->handle == p_nus->tx_handles.cccd_handle) && (p_evt_write->len == 2))
    {
        p_nus->isTX_notification_enabled = ble_srv_is_notification_enabled(p_evt_write->data);
        #if BLE_BUS_LOG_ENABLED
            NRF_LOG_INFO("TX notification: %s, change Err: 0x%04X/%u", (p_nus->isTX_notification_enabled) ? "On" : "Off", err_code, err_code);
        #endif

        // On TX notification change
        p_nus->data_handler((p_nus->isTX_notification_enabled) ? BLE_NUS_EVT_COMM_STARTED : BLE_NUS_EVT_COMM_STOPPED);
    }
    else if ((p_evt_write->handle == p_nus->rx_handles.cccd_handle) && (p_evt_write->len == 2))
    {
        p_nus->isRX_notification_enabled = ble_srv_is_notification_enabled(p_evt_write->data);
        #if BLE_BUS_LOG_ENABLED
            NRF_LOG_INFO("RX notification: %s", (p_nus->isRX_notification_enabled) ? "On" : "Off");
        #endif
    }
    else if (p_evt_write->handle == p_nus->rx_handles.value_handle)
    {
        #if BLE_BUS_LOG_ENABLED
            NRF_LOG_INFO("RX data");
            NRF_LOG_HEXDUMP_INFO(p_evt_write->data, p_evt_write->len);
        #endif

        ble_bus_evt_type_t cmd = (ble_bus_evt_type_t) p_evt_write->data[0];
        if(cmd > BLE_NUS_EVT_ERASE_FLASH)
        {
            return;
        }

        p_nus->data_handler(cmd);
    }
    else
    {
        // Do Nothing. This event is not relevant for this service.
    }
}


/**@brief Function for handling the @ref BLE_GATTS_EVT_HVN_TX_COMPLETE event from the SoftDevice.
 *
 * @param[in] p_nus     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_hvx_tx_complete(ble_bus_t * p_nus, ble_evt_t const * p_ble_evt)
{
    ret_code_t                 err_code;
    
    /*
     * Check if the notifications are enabled and if the message lenght is not zero and the message lenght is not zero.
     * Since there's no source indication with the HVX event there's no way to check if the transmission of the flash 
     * data was succesfull without adding another layer of flow control.
     */
    if (p_nus->isTX_notification_enabled && p_nus->txLenght != 0 && p_nus->txMsgLenght != 0)
    {
        // Increment the offset counter with the last message lenght
        p_nus->txOffset += p_nus->txMsgLenght;
        p_nus->packetCounter++;

        // Check whether data has to be transmitted or if the buffer is empty  
        if((p_nus->txOffset < p_nus->txLenght) && (p_nus->txLenght != 0))
        {
            err_code = ble_bus_send_data_fragment(p_nus);
            #if BLE_BUS_LOG_ENABLED
                NRF_LOG_INFO("ble_bus_send_data[%u]_fragment: %u/%u", p_nus->bufferCounter, p_nus->packetCounter, p_nus->packetMax);
            #endif
        }
        else
        {
            p_nus->txLenght         = 0;
            p_nus->txMsgLenght      = 0;
            p_nus->packetCounter    = 0;
            p_nus->packetMax        = 0;    
            p_nus->data_handler(BLE_NUS_EVT_TX_RDY);
        }
    }
}


void ble_bus_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    if ((p_context == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    ble_bus_t * p_nus = (ble_bus_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            p_nus->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            on_connect(p_nus, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_nus, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_nus, p_ble_evt);
            break;

        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
            on_hvx_tx_complete(p_nus, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


static uint32_t addRX_char(ble_bus_t * service)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_md_t user_desc_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

	if(service == NULL)
	{
		return NRF_ERROR_INVALID_PARAM;
	}

    // Add Custom Value characteristic
    memset(&cccd_md, 0, sizeof(cccd_md));

    //  Read  operation on cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc                    = BLE_GATTS_VLOC_STACK;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&user_desc_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&user_desc_md.write_perm);
    user_desc_md.vloc	= BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read			= 1;
    char_md.char_props.write		= 1;
    char_md.char_props.notify		= 1;
    char_md.p_char_user_desc		= "RX";
	char_md.char_user_desc_size		= strlen(char_md.p_char_user_desc);
	char_md.char_user_desc_max_size = char_md.char_user_desc_size;
    char_md.p_char_pf				= NULL;
    char_md.p_user_desc_md			= &user_desc_md;
    char_md.p_cccd_md				= &cccd_md; 
    char_md.p_sccd_md				= NULL;
    ble_uuid.type					= service->uuid_type;
    ble_uuid.uuid					= BLE_UUID_NUS_RX_CHARACTERISTIC;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc					= BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth					= 0;
    attr_md.wr_auth					= 0;
    attr_md.vlen					= 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid			= &ble_uuid;
    attr_char_value.p_attr_md		= &attr_md;
    attr_char_value.init_offs		= 0;
    attr_char_value.max_len			= BLE_NUS_MAX_MSG_LEN;
    attr_char_value.init_len		= 0;
    attr_char_value.p_value			= NULL;
	
    err_code = sd_ble_gatts_characteristic_add(service->service_handle, &char_md, &attr_char_value, &service->rx_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}


uint32_t ble_bus_init(ble_bus_t * p_nus, ble_bus_init_t const * p_nus_init)
{
    ret_code_t            err_code;
    ble_uuid_t            ble_uuid;
    ble_uuid128_t         nus_base_uuid = NUS_BASE_UUID;
    ble_add_char_params_t add_char_params;

    VERIFY_PARAM_NOT_NULL(p_nus);
    VERIFY_PARAM_NOT_NULL(p_nus_init);

    // Initialize the service structure.
    p_nus->data_handler = p_nus_init->data_handler;

    /**@snippet [Adding proprietary Service to the SoftDevice] */
    // Add a custom base UUID.
    err_code = sd_ble_uuid_vs_add(&nus_base_uuid, &p_nus->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_nus->uuid_type;
    ble_uuid.uuid = BLE_UUID_NUS_SERVICE;

    // Add the service.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid,  &p_nus->service_handle);
    /**@snippet [Adding proprietary Service to the SoftDevice] */
    VERIFY_SUCCESS(err_code);

    err_code = addRX_char(p_nus);
    VERIFY_SUCCESS(err_code);

    // Add the TX Characteristic.
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid              = BLE_UUID_NUS_TX_CHARACTERISTIC;
    add_char_params.uuid_type         = p_nus->uuid_type;
    add_char_params.max_len           = BLE_NUS_MAX_MSG_LEN;
    add_char_params.init_len          = sizeof(uint8_t);
    add_char_params.is_var_len        = true;
    add_char_params.char_props.notify = 1;
    add_char_params.read_access       = SEC_OPEN;
    add_char_params.write_access      = SEC_OPEN;
    add_char_params.cccd_write_access = SEC_OPEN;

    return characteristic_add(p_nus->service_handle, &add_char_params, &p_nus->tx_handles);
}


uint32_t ble_bus_send_data_fragment(ble_bus_t * p_nus)
{
    ret_code_t                  err_code;
    ble_gatts_hvx_params_t      hvx_params;
    static uint8_t              msgData [NRF_SDH_BLE_GATT_MAX_MTU_SIZE] = {0};
    static uint16_t             lenght;

    VERIFY_PARAM_NOT_NULL(p_nus);


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
    if(p_nus->txMsgLenght > BLE_NUS_MAX_DATA_LEN)
    {
        p_nus->txMsgLenght = BLE_NUS_MAX_DATA_LEN;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));

    memset(msgData, 0, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);

    msgData[0]  = p_nus->bufferCounter;
    msgData[1]  = ((p_nus->packetCounter & 0x0F) << 4) | (p_nus->packetMax & 0x0F);
    memcpy(&msgData[2], &p_nus->txData_p[p_nus->txOffset], p_nus->txMsgLenght);
    
    
    lenght = p_nus->txMsgLenght + 2;
    hvx_params.handle = p_nus->tx_handles.value_handle;
    hvx_params.p_data = msgData;
    hvx_params.p_len  = &lenght;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;

    err_code = sd_ble_gatts_hvx(p_nus->conn_handle, &hvx_params);
    return err_code;
}



uint32_t ble_bus_data_send(ble_bus_t * p_nus, uint8_t buffCount, uint8_t * p_data, uint16_t length)
{
    ret_code_t err_code;

    VERIFY_PARAM_NOT_NULL(p_nus);
    VERIFY_PARAM_NOT_NULL(p_data);

    if (p_nus->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    if(length == 0)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    p_nus->txData_p         = p_data;
    p_nus->txLenght         = length;
    p_nus->txOffset         = 0;
    p_nus->bufferCounter    = buffCount;
    p_nus->packetCounter    = 1;
    p_nus->packetMax        = length / BLE_NUS_MAX_DATA_LEN;

    if((length - (BLE_NUS_MAX_DATA_LEN * p_nus->packetMax)) != 0)
    {
        p_nus->packetMax++;
    }

    return ble_bus_send_data_fragment(p_nus);
}



uint32_t ble_nus_response_send(ble_bus_t * p_nus, uint8_t cmd, uint32_t err)
{
    ret_code_t                 err_code;
    ble_gatts_hvx_params_t     hvx_params;
    uint16_t lenght             = 6;

    VERIFY_PARAM_NOT_NULL(p_nus);

    if ((p_nus->conn_handle == BLE_CONN_HANDLE_INVALID))
    {
        return NRF_ERROR_NOT_FOUND;
    }

    if(!p_nus->isRX_notification_enabled)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));
    uint8_t buff[10];
    buff[0] = RESPONSE;
    buff[1] = cmd;
    uint32_big_encode(err, &buff[2]);


    hvx_params.handle = p_nus->rx_handles.value_handle;
    hvx_params.p_data = buff;
    hvx_params.p_len  = &lenght;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(p_nus->conn_handle, &hvx_params);
}



