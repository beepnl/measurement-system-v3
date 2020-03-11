
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "BLE_setup.h"
#define NRF_LOG_MODULE_NAME BLE_SETUP
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "boards.h"
#include "nrf_pwr_mgmt.h"
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "fds.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "ble_dis.h"
#include "ble_bas.h"
#include "ble_dfu.h"
#include "ble_beep.h"
#include "nvm_fs.h"
#include "beep_protocol.h"
#include "sdk_config.h"
#include "mx_flash_app.h"
#include "nrf_log_ctrl.h"



NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/
BLE_BAS_DEF(m_bas);                                                             /**< Structure used to identify the battery service. */
BLE_BEEP_DEF(m_beep);
BLE_ADVERTISING_DEF(m_advertising);                                             /**< Advertising module instance. */
static bool advDataChanged = false;
static pm_peer_id_t m_peer_to_be_deleted = PM_PEER_ID_INVALID;
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

/* YOUR_JOB: Declare all services structure your application is using
 *  BLE_XYZ_DEF(m_xyz);
 */

// YOUR_JOB: Use UUIDs for service(s) used in your application.
static ble_uuid_t m_adv_uuids[] =                                               /**< Universally unique service identifiers. */
{
    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
};


uint32_t getMTUdataLenght(uint16_t handle, uint8_t * len)
{
    uint32_t retval = NRF_SUCCESS;
    *len = nrf_ble_gatt_eff_mtu_get(&m_gatt, handle);
    return retval;
}

static void disconnect(uint16_t conn_handle, void * p_context)
{
    UNUSED_PARAMETER(p_context);

    ret_code_t err_code = sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("Failed to disconnect connection. Connection handle: %d Error: %d", conn_handle, err_code);
    }
    else
    {
        NRF_LOG_DEBUG("Disconnected connection handle %d", conn_handle);
    }
}

static void advertising_config_get(ble_adv_modes_config_t * p_config)
{
    memset(p_config, 0, sizeof(ble_adv_modes_config_t));

    p_config->ble_adv_fast_enabled  = true;
    p_config->ble_adv_fast_interval = APP_ADV_INTERVAL;
    p_config->ble_adv_fast_timeout  = APP_ADV_DURATION;
}



// YOUR_JOB: Update this code if you want to do anything given a DFU event (optional).
/**@brief Function for handling dfu events from the Buttonless Secure DFU service
 *
 * @param[in]   event   Event from the Buttonless Secure DFU service.
 */
static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event)
{
    switch (event)
    {
        case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
        {
            NRF_LOG_INFO("Device is preparing to enter bootloader mode.");

            // Prevent device from advertising on disconnect.
            ble_adv_modes_config_t config;
            advertising_config_get(&config);
            config.ble_adv_on_disconnect_disabled = true;
            ble_advertising_modes_config_set(&m_advertising, &config);

            // Disconnect all other bonded devices that currently are connected.
            // This is required to receive a service changed indication
            // on bootup after a successful (or aborted) Device Firmware Update.
            uint32_t conn_count = ble_conn_state_for_each_connected(disconnect, NULL);
            NRF_LOG_INFO("Disconnected %d links.", conn_count);
            break;
        }

        case BLE_DFU_EVT_BOOTLOADER_ENTER:
            // YOUR_JOB: Write app-specific unwritten data to FLASH, control finalization of this
            //           by delaying reset by reporting false in app_shutdown_handler
            NRF_LOG_INFO("Device will enter bootloader mode.");
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
            NRF_LOG_ERROR("Request to enter bootloader mode failed asynchroneously.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            break;

        case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
            NRF_LOG_ERROR("Request to send a response to client failed.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            APP_ERROR_CHECK(false);
            break;

        default:
            NRF_LOG_ERROR("Unknown event from ble_dfu_buttonless.");
            break;
    }
}



/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start(false);
            break;

        /* Work around for refreshing a bond
         * https://devzone.nordicsemi.com/f/nordic-q-a/38663/keep-getting-failed-to-secure-connection-error
         */
        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Allow pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = true};
            pm_conn_sec_config_reply(m_conn_handle, &conn_sec_config);
        } break;

        default:
            break;
    }
}


void ble_setAdvertisingName(void)
{
	ret_code_t              err_code;
    BEEP_protocol_s			deviceEUI;
    ble_gap_conn_sec_mode_t sec_mode;
    
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    // Retrieve the DEVEUI from the flash.
    deviceEUI.command = READ_LORAWAN_DEVEUI;
    nvm_fds_eeprom_get(&deviceEUI);

        // Device Name String
    uint8_t device_name_string [20] = {0};
	uint8_t nameLenght = 0;
    nameLenght = sprintf(device_name_string, "BEEPBASE-%02X%02X",
												deviceEUI.param.lorawan_key.data[6],
												deviceEUI.param.lorawan_key.data[7]);

    err_code = sd_ble_gap_device_name_set(&sec_mode, device_name_string, nameLenght);
    APP_ERROR_CHECK(err_code);
}




/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;


	ble_setAdvertisingName();

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    #if DEBUG
        gap_conn_params.min_conn_interval = MIN_NUS_CONN_INTERVAL;
        gap_conn_params.max_conn_interval = MAX_NUS_CONN_INTERVAL;
    #else
        gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
        gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    #endif
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    uint16_t m_ble_nus_max_data_len;

    switch(p_evt->evt_id)
    {
        case NRF_BLE_GATT_EVT_ATT_MTU_UPDATED:
        {
            m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
            m_beep.dataLenghtMax = m_ble_nus_max_data_len;
            NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
            break;
        }

        case NRF_BLE_GATT_EVT_DATA_LENGTH_UPDATED:
        {
            NRF_LOG_INFO("NRF_BLE_GATT_EVT_DATA_LENGTH_UPDATED %u", p_evt->params.data_length);
            break;
        }

        default:
            break;
    }
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}




static void beep_bas_init(void)
{
	ret_code_t         err_code;
	ble_bas_init_t     bas_init;

    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.
    bas_init.bl_rd_sec				= SEC_OPEN;
    bas_init.bl_cccd_wr_sec			= SEC_OPEN;
    bas_init.bl_report_rd_sec		= SEC_OPEN;

    bas_init.evt_handler			= NULL;
    bas_init.support_notification	= true;
    bas_init.p_report_ref			= NULL;
    bas_init.initial_batt_level		= 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);
}

static void beep_dis_init(void)
{
	#define DESC_LENGHT_MAX 10
	uint8_t i;
	char hwDescription[DESC_LENGHT_MAX] = {0};
	char fwDescription[DESC_LENGHT_MAX] = {0};
    char snDescription[19] = {0};

	ret_code_t			err_code;
	ble_dis_init_t		dis_init;
    BEEP_protocol_s		fw, hw, sn;

    fw.command = READ_FIRMWARE_VERSION;
	hw.command = READ_HARDWARE_VERSION;
    sn.command = READ_ATECC_READ_ID;


    nvm_fds_eeprom_get(&fw);
    nvm_fds_eeprom_get(&hw);
    nvm_fds_eeprom_get(&sn);

	snprintf(hwDescription, DESC_LENGHT_MAX, "%u.%u", hw.param.version.major, hw.param.version.minor);
    snprintf(fwDescription, DESC_LENGHT_MAX, "%u.%u.%u", fw.param.version.major, fw.param.version.minor, fw.param.version.sub);

	for(i=0; i<ATECC_ID_LENGHT; i++)
	{
		snprintf(&snDescription[i * 2], DESC_LENGHT_MAX, "%02X", sn.param.atecc_id.ROM[i]);
	}

	memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str,	BLE_DIS_MANUFACTURER_NAME);
    ble_srv_ascii_to_utf8(&dis_init.model_num_str,		BLE_DIS_MODEL_NUMBER);
    ble_srv_ascii_to_utf8(&dis_init.serial_num_str,		snDescription);
    ble_srv_ascii_to_utf8(&dis_init.hw_rev_str,			hwDescription);
    ble_srv_ascii_to_utf8(&dis_init.fw_rev_str,			fwDescription);
    dis_init.dis_char_rd_sec = SEC_OPEN;

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}




static void Beep_service_init(void * beep_callback_p)
{
	ret_code_t			err_code;
	ble_beep_init_t		init;

    memset(&init, 0, sizeof(init));

    init.evt_handler = NULL;

    // Here the sec level for the Cycling Speed and Cadence Service can be changed/increased.
	#if BLE_PASSKEY_ENABLED
		BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&init.security_level.cccd_write_perm);
		BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&init.security_level.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&init.security_level.write_perm);
	#else
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&init.security_level.cccd_write_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&init.security_level.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&init.security_level.write_perm);
	#endif

    // Lazy method of connecting the data handler for the TX characteristic.
    extern void flashApp_cmd_handler(app_flash_evt_type_t type, BEEP_protocol_s * prot);

	init.callback_p     = (protocol_cb_p) beep_callback_p;
    init.txCallback     = flashApp_cmd_handler;

	ble_beep_init(&m_beep, &init);
}

static void beep_dfu_init(void)
{
    uint32_t                  err_code;
    ble_dfu_buttonless_init_t dfus_init = {0};

	// Use lazy extern method to connect the handler in the main.c file to the dfu service.
	extern bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event);
    dfus_init.evt_handler = ble_dfu_evt_handler;

    err_code = ble_dfu_buttonless_init(&dfus_init);
    APP_ERROR_CHECK(err_code);
}




/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void * beep_callback_p)
{
    ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

	// Initialize the Device Information Service
    beep_dis_init();

	// Initialize the Battery Service
    beep_bas_init();

	// Initialize the DFU service
    beep_dfu_init();

	// Beep service and control point
    Beep_service_init(beep_callback_p);

    APP_ERROR_CHECK(nrf_ble_gatt_data_length_get(&m_gatt, BLE_CONN_HANDLE_INVALID, &m_beep.dataLenghtMax));
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;


	#if 0
		// Prepare wakeup buttons.
		err_code = bsp_btn_ble_sleep_mode_prepare();
		APP_ERROR_CHECK(err_code);
	#else
		nrf_gpio_cfg_sense_input(SQ_SEN_645B_PIN,	NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
        nrf_gpio_cfg_sense_input(BUTTON_1,			NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
	#endif

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            break;

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
#if BLE_PASSKEY_ENABLED
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    pm_handler_secure_on_connection(p_ble_evt);

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
        {
            NRF_LOG_INFO("Disconnected");
            m_conn_handle = BLE_CONN_HANDLE_INVALID;

            #if BLE_ADV_START_UP_ENABLE
            if(advDataChanged)
			{
				advDataChanged = false;
                advertising_start(false);
			}
            #endif

            // Check if the last connected peer had not used MITM, if so, delete its bond information.
            if (m_peer_to_be_deleted != PM_PEER_ID_INVALID)
            {
                err_code = pm_peer_delete(m_peer_to_be_deleted);
                APP_ERROR_CHECK(err_code);
                NRF_LOG_DEBUG("Collector's bond deleted");
                m_peer_to_be_deleted = PM_PEER_ID_INVALID;
            }
        } break;

        case BLE_GAP_EVT_CONNECTED:
        {
            NRF_LOG_INFO("Connected");
            m_peer_to_be_deleted = PM_PEER_ID_INVALID;
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            // Start Security Request timer.
        } break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            NRF_LOG_DEBUG("BLE_GAP_EVT_SEC_PARAMS_REQUEST");
            break;

        case BLE_GAP_EVT_PASSKEY_DISPLAY:
        {
            char passkey[PASSKEY_LENGTH + 1];
            memcpy(passkey, p_ble_evt->evt.gap_evt.params.passkey_display.passkey, PASSKEY_LENGTH);
            passkey[PASSKEY_LENGTH] = 0;

            NRF_LOG_INFO("Passkey: %s", nrf_log_push(passkey));
        } break;
        
        case BLE_GAP_EVT_AUTH_KEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_AUTH_KEY_REQUEST");
            break;

        case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_LESC_DHKEY_REQUEST");
            break;

         case BLE_GAP_EVT_AUTH_STATUS:
             NRF_LOG_INFO("BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x",
                          p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
                          p_ble_evt->evt.gap_evt.params.auth_status.bonded,
                          p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));
            break;

        default:
            // No implementation needed.
            break;
    }
}
#else
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
			if(advDataChanged)
			{
				advDataChanged = false;
                advertising_start(true);
			}
            // LED indication will be changed when advertising starts.
            break;

        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);

            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE:
            
            NRF_LOG_INFO("BLE_GAP_EVT_CONN_PARAM_UPDATE %u/%u",
                p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.min_conn_interval,
                p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.max_conn_interval);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}
#endif


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


void load_passKey(void)
{
	ret_code_t		err_code;
	ble_opt_t		ble_opt;
	BEEP_protocol_s passkey;

	// Retrieve the passkey from the EEPROM.
	memset(&passkey, 0, sizeof(BEEP_protocol_s));
    passkey.command = READ_PINCODE;
	nvm_fds_eeprom_get(&passkey);

	ble_opt.gap_opt.passkey.p_passkey = (uint8_t const *)&passkey.param.lorawan_key.data;
	err_code = sd_ble_opt_set(BLE_GAP_OPT_PASSKEY, &ble_opt);
	APP_ERROR_CHECK(err_code); 
}


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

#if BLE_PASSKEY_ENABLED
	// Load the passkey from EEPROM.
    load_passKey();

    // Security parameters to be used for all security procedures.
    sec_param.bond           = 1;
    sec_param.mitm           = 1;
    sec_param.lesc           = 1;
    sec_param.keypress       = 0;
    sec_param.io_caps        = BLE_GAP_IO_CAPS_DISPLAY_ONLY;
    sec_param.oob            = 0;
    sec_param.min_key_size   = PIN_CODE_LENGHT_MIN;
    sec_param.max_key_size   = PIN_CODE_LENGHT_MAX;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;
#else
    // Security parameters to be used for all security procedures.
    sec_param.bond           = 1;
    sec_param.mitm           = 0;
    sec_param.lesc           = 0;
    sec_param.keypress       = 0;
    sec_param.io_caps        = BLE_GAP_IO_CAPS_NONE;
    sec_param.oob            = 0;
    sec_param.min_key_size   = PIN_CODE_LENGHT_MIN;
    sec_param.max_key_size   = PIN_CODE_LENGHT_MAX;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;
#endif


    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Clear bond information from persistent storage.
 */
void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break; // BSP_EVENT_SLEEP

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break; // BSP_EVENT_DISCONNECT

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break; // BSP_EVENT_KEY_0

        default:
            break;
    }
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled	= true;
    init.config.ble_adv_fast_interval	= APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout	= APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

void ble_updateAdvertisingData(void)
{
	advertising_stop();

	// Reset the advertising name
    ble_setAdvertisingName();

	// Reinit the advertising data
	advertising_init();

	// restart advertising with the new name on disconnect
    advDataChanged = true;
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}





/**@brief Function for starting advertising.
 */
uint32_t advertising_start(bool erase_bonds)
{
	#if BLE_ADV_ENABLE
    if(erase_bonds)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETED_SUCEEDED event
    }

	ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
	//APP_ERROR_CHECK(err_code); // Ignore this check or the device will crash due to this statment when it' already advertising.
	#endif
    return err_code;
}


/**@brief Function for stopping advertising.
 */
void advertising_stop(void)
{
	sd_ble_gap_adv_stop(m_advertising.adv_handle);
}

void disconnect_clients(void)
{
	sd_ble_gap_connect_cancel();
}


void BLE_setup(void * beep_callback_p)
{
	ble_stack_init();
    gap_params_init();
    gatt_init();
    advertising_init();
    services_init(beep_callback_p);
    conn_params_init();
    peer_manager_init();
}

uint32_t ble_controlPoint_response(BEEP_CID cmd, uint32_t error_code)
{
	 return beep_ctrlpt_send_error(&m_beep, cmd, error_code);
}

uint32_t ble_controlPoint_send(BEEP_protocol_s * prot)
{
	return ble_beep_controlPoint_send(&m_beep, prot);
}


uint32_t ble_measurement_send(MEASUREMENT_RESULT_s * measurement)
{
	// Update the new battery percentage value when a value is available.
	if(measurement->type == nRF_ADC)
	{
		ble_bas_battery_level_update(&m_bas, measurement->result.saadc.battPercentage, m_conn_handle);
	}
	return ble_beep_measurement_send(&m_beep, measurement);
}

void bootloader_setupSVCI(void)
{
	uint32_t err_code;

    // Initialize the async SVCI interface to bootloader before any interrupts are enabled.
    err_code = ble_dfu_buttonless_async_svci_init();
    NRF_LOG_INFO("ble_dfu_buttonless_async_svci_init err: %u", err_code);
    NRF_LOG_FLUSH();
    APP_ERROR_CHECK(err_code);
}


uint32_t changeConnectionInterval(bool fast, uint16_t handle)
{
    uint32_t err_code;
    ble_gap_conn_params_t p_new_params = 
    {
        .slave_latency     = 0,
        .conn_sup_timeout  = CONN_SUP_TIMEOUT,
    };

    if(fast)
    {
        p_new_params.min_conn_interval = MIN_NUS_CONN_INTERVAL;
        p_new_params.max_conn_interval = MAX_NUS_CONN_INTERVAL;
    }
    else
    {
        p_new_params.min_conn_interval = MIN_CONN_INTERVAL;
        p_new_params.max_conn_interval = MAX_CONN_INTERVAL;   
    }

    err_code = ble_conn_params_change_conn_params(handle, &p_new_params);
    NRF_LOG_INFO("Changed connection Interval %u/%u ms Err: 0x%04X/%u", p_new_params.min_conn_interval, p_new_params.max_conn_interval, err_code, err_code);
    return err_code;
}

uint32_t changeMTUsize(uint16_t conn_handle, uint8_t mtu_desired)
{
    uint32_t err_code;
    
    err_code = nrf_ble_gatt_data_length_set(&m_gatt, conn_handle, mtu_desired);
    NRF_LOG_INFO("mtu of %d requested for handle %u, err: 0x%04X/%u", mtu_desired, conn_handle, err_code, err_code);
    return err_code;
}


uint32_t ble_TXdata_send(uint16_t * buffCount, uint8_t * p_data, uint16_t length)
{
    return ble_beep_TXdata_send(&m_beep, buffCount, p_data, length);
}