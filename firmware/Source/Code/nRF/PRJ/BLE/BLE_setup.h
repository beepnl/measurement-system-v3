/*!
 * \file      BLE_setup.h
 *
 * \brief     BLE_setup and main BLE initialization of all the different services.
 *
 */
#ifndef __BLE_SETUP_H__
#define __BLE_SETUP_H__

	#include <stdbool.h>
	#include <stdint.h>
	#include "beep_types.h"
	#include "beep_protocol.h"


    
	#define DEVICE_NAME                     "BEEPBASE"								/**< Name of device. Will be included in the advertising data. */
	#define MANUFACTURER_NAME               "BEEP"									/**< Manufacturer. Will be passed to Device Information Service. */
	#define APP_ADV_INTERVAL                300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
	#define APP_ADV_DURATION                18000                                   /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
	#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
	#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

	#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
	#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
	#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

	#define PASSKEY_TXT                     "Passkey:"                              /**< Message to be displayed together with the pass-key. */
	#define PASSKEY_TXT_LENGTH              8                                       /**< Length of message to be displayed together with the pass-key. */
	#define PASSKEY_LENGTH                  6                                       /**< Length of pass-key received by the stack for display. */


    #define DEAD_BEEF                       0xDEADBEEF                         /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


	// Device Information Service	
	#define BLE_DIS_MANUFACTURER_NAME       "Stichting BEEP"						/**< Manufacturer Name String. */
	#define BLE_DIS_MODEL_NUMBER            "BEEPBASE"                              /**< Model Number String. */

	void		delete_bonds				(void);
	void		advertising_stop			(void);
	void		disconnect_clients			(void);
	uint32_t	advertising_start			(bool erase_bonds);
    void		ble_setAdvertisingName		(void);
    void		ble_updateAdvertisingData	(void);
    void        load_passKey                (void);
	void		BLE_setup					(void * beep_callback_p);
    uint32_t	ble_controlPoint_send		(BEEP_protocol_s * prot);
    uint32_t	ble_controlPoint_response	(BEEP_CID cmd, uint32_t error_code);
    uint32_t	ble_measurement_send		(MEASUREMENT_RESULT_s * measurement);
    void		bootloader_setupSVCI		(void);
    uint32_t    changeConnectionInterval    (bool fast, uint16_t handle);
    uint32_t    changeMTUsize               (uint16_t conn_handle, uint8_t mtu_desired);
    uint32_t    getMTUdataLenght            (uint16_t handle, uint8_t * len);
    uint32_t    ble_TXdata_send             (uint16_t * buffCount, uint8_t * p_data, uint16_t length);

#endif // __BLE_SETUP_H__
