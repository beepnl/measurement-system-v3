
#ifndef BLE_BEEP_H__
#define BLE_BEEP_H__

#include <stdint.h>
#include <stdbool.h>
#include "sdk_config.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "beep_protocol.h"
#include "beep_types.h"


#ifdef __cplusplus
extern "C" {
#endif

/**@brief   Macro for defining a ble_beep instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_BEEP_DEF(_name)                                                                         \
static ble_beep_t _name;                                                                            \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_beep_BLE_OBSERVER_PRIO,                                                    \
                     ble_beep_on_ble_evt, &_name)




#define BEEP_SERVICE_UUID			0x68A1	
#define BEEP_DS18B20_UUID			0x68A2
#define BEEP_TX_UUID                0x68A3
#define BEEP_CP_UUID				0x68B0
#define BEEP_SERVICE_UUID_BASE		{0x1b, 0xc3, 0xf8, 0xc5, 0xeb, 0xc6, 0x40, 0x50, 0xad, 0x4b, 0x9f, 0x71, 0xd4, 0xa6, 0x47, 0xbe} // 1bc3f8c5-ebc6-4050-ad4b-9f71d4a647be

#define OPCODE_LENGTH 1                                                             /**< Length of opcode inside Cycling Speed and Cadence Measurement packet. */
#define HANDLE_LENGTH 2  
/**@brief   Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
#if defined(NRF_SDH_BLE_GATT_MAX_MTU_SIZE) && (NRF_SDH_BLE_GATT_MAX_MTU_SIZE != 0)
    #define BLE_NUS_MAX_MSG_LEN     (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH)
    #define BLE_NUS_MAX_DATA_LEN    (BLE_NUS_MAX_MSG_LEN - 2)
#else
    #define BLE_NUS_MAX_DATA_LEN (BLE_GATT_MTU_SIZE_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH)
    #warning NRF_SDH_BLE_GATT_MAX_MTU_SIZE is not defined.
#endif



/** Speed and Cadence Control Point procedure status (indicates is a procedure is in progress or not and which procedure is in progress*/
typedef enum {
    BLE_CONTROL_NO_PROC_IN_PROGRESS						= 0x00,                               /**< No procedure in progress. */
    BLE_CONTROL_PROC_IN_PROGRESS						= 0x01,                               /**< Procedure is in progress. */
    BLE_CONTROL_INDICATION_PENDING						= 0x02,                               /**< Control Point Indication is pending. */
    BLE_CONTROL_IND_CONFIRM_PENDING						= 0x03,                               /**< Waiting for the indication confirmation. */
}ble_control_status_t;


/**@brief Cycling Speed and Cadence Service event type. */
typedef enum
{
    BLE_beep_EVT_NOTIFICATION_ENABLED,
    BLE_beep_EVT_NOTIFICATION_DISABLED 
} ble_beep_evt_type_t;

/**@brief Cycling Speed and Cadence Service event. */
typedef struct
{
    ble_beep_evt_type_t evt_type;                                       /**< Type of event. */
} ble_beep_evt_t;

// Forward declaration of the ble_csc_t type.
typedef struct ble_beep_s ble_beep_t;

/**@brief Event handler type. */
typedef void (*ble_beep_evt_handler_t) (ble_beep_t * p_beep, ble_beep_evt_t * p_evt);
typedef void (* ble_tx_handler_t) (app_flash_evt_type_t eventType, BEEP_protocol_s * prot);


/**@brief Control Point response indication structure. */
typedef struct
{
    ble_control_status_t			status;                                                  /**< control point response status .*/
    uint8_t							len;                                                     /**< control point response length .*/
    uint8_t							encoded_ctrl_rsp[BEEP_MAX_LENGHT];						/**< control point encoded response.*/
}beep_ctrlpt_resp_t;

/**@brief Cycling Speed and Cadence Service init structure. This contains all options and data
*         needed for initialization of the service. */
typedef struct
{
    ble_srv_cccd_security_mode_t	security_level;							/**< Initial security level for Custom characteristics attribute */
    ble_beep_evt_handler_t			evt_handler;                           /**< Event handler to be called for handling events in the Cycling Speed and Cadence Service. */
    protocol_cb_p					callback_p;
    ble_tx_handler_t                txCallback;
} ble_beep_init_t;

/**@brief Cycling Speed and Cadence Service structure. This contains various status information for
 *        the service. */
struct ble_beep_s
{

    ble_beep_evt_handler_t			evt_handler;							/**< Event handler to be called for handling events in the Cycling Speed and Cadence Service. */
    uint16_t						service_handle;							/**< Handle of Cycling Speed and Cadence Service (as provided by the BLE stack). */
    uint16_t						conn_handle;							/**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint16_t						feature;								/**< Bit mask of features available on sensor. */
	ble_gatts_char_handles_t		ds18B20_char;	
    ble_gatts_char_handles_t        tx_handles;
    ble_gatts_char_handles_t		control_char;
    beep_ctrlpt_resp_t				control_rsp;
	uint8_t							uuid_type;
    protocol_cb_p					callback_p;

    // TX functionality
    uint8_t                         dataLenghtMax;
    ble_tx_handler_t                txEventHandler;               /**< Event handler to be called for handling received data. */
    bool                            isRX_notification_enabled;  /**< Variable to indicate if the peer has enabled notification of the RX characteristic.*/
    bool                            isTX_notification_enabled;  /**< Variable to indicate if the peer has enabled notification of the TX characteristic.*/
    uint8_t                         *txData_p;
    uint16_t                        txLenght;
    uint16_t                        txOffset;
    uint16_t                        txMsgLenght;
    uint8_t                         mtuDatalenght;
    uint16_t                       *packetCounter;
};



/**@brief Function for initializing the Cycling Speed and Cadence Service.
 *
 * @param[out]  p_beep      Cycling Speed and Cadence Service structure. This structure will have to
 *                          be supplied by the application. It will be initialized by this function,
 *                          and will later be used to identify this particular service instance.
 * @param[in]   p_beep_init Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_beep_init(ble_beep_t * p_beep, ble_beep_init_t const * p_beep_init);


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Cycling Speed and Cadence
 *          Service.
 *
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 * @param[in]   p_context   Cycling Speed and Cadence Service structure.
 */
void ble_beep_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief Function for sending cycling speed and cadence measurement if notification has been enabled.
 *
 * @details The application calls this function after having performed a Cycling Speed and Cadence
 *          Service measurement. If notification has been enabled, the measurement data is encoded
 *          and sent to the client.
 *
 * @param[in]   p_beep         Cycling Speed and Cadence Service structure.
 * @param[in]   p_measurement  Pointer to new cycling speed and cadence measurement.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */

uint32_t ble_beep_controlPoint_send	(ble_beep_t * p_beep, BEEP_protocol_s * prot);
uint32_t ble_beep_measurement_send	(ble_beep_t * p_beep, MEASUREMENT_RESULT_s * measurement);
uint32_t beep_ctrlpt_send_error		(ble_beep_t * p_beep, BEEP_CID cmd, uint32_t error);

// TX function
uint32_t ble_beep_TXdata_send              (ble_beep_t * p_nus, uint16_t * buffCount, uint8_t * p_data, uint16_t length);
uint32_t ble_beep_send_TXdata_fragment     (ble_beep_t * p_nus);

#ifdef __cplusplus
}
#endif

#endif // BLE_BEEP_H__

/** @} */
