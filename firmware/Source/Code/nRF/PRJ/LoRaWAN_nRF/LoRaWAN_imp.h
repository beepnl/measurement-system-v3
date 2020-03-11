/*!
 * \file      LoRaWAN_imp.h
 *
 * \brief     LoRaWAN implementation
 *
 */
#ifndef __LORAWAN_IMP_H__
#define __LORAWAN_IMP_H__

#include "Commissioning.h"
#include "beep_protocol.h"





/*!
 * Defines the application data transmission duty cycle. 60s, value in [ms].
 */
#ifdef DEBUG
	#define APP_TX_DUTYCYCLE                            (1 * 30 * 1000)
	#define APP_JOIN_DUTYCYCLE                          (1 * 30 * 1000)
#else
	#define APP_TX_DUTYCYCLE                            (1 * 60 * 1000)
	#define APP_JOIN_DUTYCYCLE                          (1 * 60 * 1000)
#endif
#define PAYLOAD_SIZE_MAX    52
/*!
 * Defines a random delay for application data transmission duty cycle. 10s,
 * value in [ms].
 */
#define APP_TX_DUTYCYCLE_RND                        10000

/*!
 * Default datarate
 */
#define LORAWAN_DEFAULT_DATARATE                    DR_0

/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_CONFIRMED_MSG_ON                    false

/*!
 * LoRaWAN Adaptive Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_ON                              true

#if defined( REGION_EU868 ) || defined( REGION_RU864 ) || defined( REGION_CN779 ) || defined( REGION_EU433 )

#include "LoRaMacTest.h"

/*!
 * LoRaWAN ETSI duty cycle control enable/disable
 *
 * \remark Please note that ETSI mandates duty cycled transmissions. Use only for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON                        true

#endif

/*!
 * LoRaWAN application port
 */
#define LORAWAN_APP_PORT            2
#define KEY_SIZE					16


	uint8_t		Lorawan_get_status				(void);
	void		lorawan_set_status				(uint8_t status);
	bool		lorawan_valid_Keys				(void);
	void		lorawan_AppEnable				(bool enabled);
	bool		lorawan_busy					(void);
    bool		lorawanDownlinkResponseEmpty	(void);
    uint32_t	lorawanDownlinkAppend			(BEEP_protocol_s * prot);
    uint32_t	Beep_downlink					(uint8_t fport, uint8_t * payload, uint8_t lenght);
    void		lorawanDownlinkResponseAppend	(BEEP_CID cmd, uint32_t error_code);
	void		lorawan_implementation_init		(protocol_cb_p callback_p);
	void		lorawan_implementation_while	(void);
	bool		lorawan_joined_network			(void);
	uint32_t	Beep_SendFormat					(BEEP_STATUS index, MEASUREMENT_RESULT_s * alarm);
	uint32_t	BEEP_Send						(uint8_t fport, uint8_t * payload, uint8_t payload_lenght);
	void		setTrigger						(BEEP_STATUS triggerValue);


#endif // __LORAWAN_IMP_H__
