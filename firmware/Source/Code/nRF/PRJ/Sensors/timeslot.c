#include "timeslot.h"
#define NRF_LOG_MODULE_NAME TIMESLOT
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
#include "nrf_delay.h"
#include "utilities.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_sdh.h"
#include "boards.h"


/**Constants for timeslot API
*/
static nrf_radio_request_t  m_timeslot_request;
static uint32_t             m_slot_length;

static nrf_radio_signal_callback_return_param_t signal_callback_return_param;

/**@brief Request next timeslot event in earliest configuration
 */
uint32_t request_next_event_earliest(void)
{
    m_slot_length                                  = 15000;
    m_timeslot_request.request_type                = NRF_RADIO_REQ_TYPE_EARLIEST;
    m_timeslot_request.params.earliest.hfclk       = NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED;
    m_timeslot_request.params.earliest.priority    = NRF_RADIO_PRIORITY_NORMAL;
    m_timeslot_request.params.earliest.length_us   = m_slot_length;
    m_timeslot_request.params.earliest.timeout_us  = 1000000;
    return sd_radio_request(&m_timeslot_request);
}


/**@brief Configure next timeslot event in earliest configuration
 */
void configure_next_event_earliest(void)
{
    m_slot_length                                  = 15000;
    m_timeslot_request.request_type                = NRF_RADIO_REQ_TYPE_EARLIEST;
    m_timeslot_request.params.earliest.hfclk       = NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED;
    m_timeslot_request.params.earliest.priority    = NRF_RADIO_PRIORITY_NORMAL;
    m_timeslot_request.params.earliest.length_us   = m_slot_length;
    m_timeslot_request.params.earliest.timeout_us  = 1000000;
}


/**@brief Configure next timeslot event in normal configuration
 */
void configure_next_event_normal(void)
{
    m_slot_length                                 = 15000;
    m_timeslot_request.request_type               = NRF_RADIO_REQ_TYPE_NORMAL;
    m_timeslot_request.params.normal.hfclk        = NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED;
    m_timeslot_request.params.normal.priority     = NRF_RADIO_PRIORITY_HIGH;
    m_timeslot_request.params.normal.distance_us  = 100000;
    m_timeslot_request.params.normal.length_us    = m_slot_length;
}


/**@brief Timeslot signal handler
 */
void nrf_evt_signal_handler(uint32_t evt_id)
{
    uint32_t err_code;
    
    switch (evt_id)
    {
        case NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN:
            //No implementation needed
            break;
        case NRF_EVT_RADIO_SESSION_IDLE:
            //No implementation needed
            break;
        case NRF_EVT_RADIO_SESSION_CLOSED:
            //No implementation needed, session ended
            break;
        case NRF_EVT_RADIO_BLOCKED:
            //Fall through
        case NRF_EVT_RADIO_CANCELED:
            err_code = request_next_event_earliest();
            APP_ERROR_CHECK(err_code);
            break;
        default:
            break;
    }
}


/**@brief Timeslot event handler
 */
nrf_radio_signal_callback_return_param_t * radio_callback(uint8_t signal_type)
{
    switch(signal_type)
    {
        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_START:
            //Start of the timeslot - set up timer interrupt
            signal_callback_return_param.params.request.p_next = NULL;
            signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
            
            NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
            NRF_TIMER0->CC[0] = m_slot_length - 1000;
            NVIC_EnableIRQ(TIMER0_IRQn);   
            
            nrf_gpio_pin_toggle(20); //Toggle LED4
            break;

        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO:
            signal_callback_return_param.params.request.p_next = NULL;
            signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
            break;

        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0:
            //Timer interrupt - do graceful shutdown - schedule next timeslot
            configure_next_event_normal();
            signal_callback_return_param.params.request.p_next = &m_timeslot_request;
            signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
            break;
        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED:
            //No implementation needed
            break;
        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED:
            //Try scheduling a new timeslot
            configure_next_event_earliest();
            signal_callback_return_param.params.request.p_next = &m_timeslot_request;
            signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
            break;
        default:
            //No implementation needed
            break;
    }
    return (&signal_callback_return_param);
}


/**@brief Function for initializing the timeslot API.
 */
uint32_t timeslot_sd_init(void)
{
    uint32_t err_code;
    
    err_code = sd_radio_session_open(radio_callback);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    err_code = request_next_event_earliest();
    if (err_code != NRF_SUCCESS)
    {
        (void)sd_radio_session_close();
        return err_code;
    }
    return NRF_SUCCESS;
}

