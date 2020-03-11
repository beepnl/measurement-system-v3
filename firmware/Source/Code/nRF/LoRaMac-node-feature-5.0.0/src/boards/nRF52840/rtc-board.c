/*!
 * \file      rtc-board.c
 *
 * \brief     Target board RTC timer and low power modes management
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
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 * \author    Marten Lootsma(TWTG) on behalf of Microchip/Atmel (c)2017
 */

#include "boards.h"
#include "board.h"
#include "timer.h"
#include "systime.h"
#include "gpio.h"
#include "rtc-board.h"

#include "nrf_drv_clock.h"
#include "app_timer.h"
#include "nrf.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "time.h"

#define RTC_DEBUG_ENABLE                            1
#define RTC_DEBUG_DISABLE                           0

#define RTC_DEBUG_GPIO_STATE                        RTC_DEBUG_DISABLE
#define RTC_DEBUG_PRINTF_STATE                      RTC_DEBUG_DISABLE

#define MIN_ALARM_DELAY                             3 // in ticks

/*!
 * \brief Indicates if the RTC is already Initialized or not
 */
static bool RtcInitialized = false;
static volatile bool RtcTimeoutPendingInterrupt = false;
static volatile bool RtcTimeoutPendingPolling = false;

APP_TIMER_DEF(RTCtimer);
APP_TIMER_DEF(minTimer);

uint32_t	prevRtcTickCount;	//	The RTC counter value from the last minute update
uint64_t	lifeTime_ms;		//	lifeTime of the device in milliseconds. Since there' no battery backed-up time RTC it can only remember the life-time if it's stored in the NVM
uint64_t	onTime_ms;			//	The on-time since start-up in milliseconds.

typedef enum AlarmStates_e
{
    ALARM_STOPPED = 0,
    ALARM_RUNNING = !ALARM_STOPPED
} AlarmStates_t;

/*!
 * RTC timer context 
 */
typedef struct
{
    uint32_t Time;  // Reference time
    uint32_t Delay; // Reference Timeout duration
    uint32_t AlarmState;
}RtcTimerContext_t;

/*!
 * Keep the value of the RTC timer when the RTC alarm is set
 * Set with the \ref RtcSetTimerContext function
 * Value is kept as a Reference to calculate alarm
 */
static RtcTimerContext_t RtcTimerContext;

#if( RTC_DEBUG_GPIO_STATE == RTC_DEBUG_ENABLE )
	Gpio_t DbgRtcPin0;
	Gpio_t DbgRtcPin1;
#endif

/*!
 * Used to store the Seconds and SubSeconds.
 * 
 * WARNING: Temporary fix fix. Should use MCU NVM internal
 *          registers
 */
uint32_t RtcBkupRegisters[] = { 0, 0 };

/*!
 * \brief Callback for the hw_timer when alarm expired
 */
static void RtcAlarmIrq( void * p_context );

/*!
 * \brief Callback for the hw_timer when counter overflows
 */
static void RtcOverflowIrq( void * p_context );


static uint32_t RtcTicksSinceMinCallback(bool update)
{
	uint32_t	milliSeconds;	
	uint32_t	currentRtcTickCount;
	uint32_t	diffTicks;

	currentRtcTickCount = app_timer_cnt_get();
	diffTicks = (currentRtcTickCount > prevRtcTickCount) ? (currentRtcTickCount - prevRtcTickCount):(prevRtcTickCount - currentRtcTickCount); // Get the absolute difference

	// Overwrite the previous Tickcount when true
	if(update)
	{
		prevRtcTickCount = currentRtcTickCount;
	}
	return diffTicks;
}


static void RtcMinuteCallback( void * p_context )
{
	UNUSED_VARIABLE(p_context);
	uint32_t	milliSeconds;	

	// Calculate the number of millisecond that have passed since the last callback.
    milliSeconds = RtcTick2Ms( RtcTicksSinceMinCallback(true) );

	// Append the passed milliseconds to the lifetime and onTime counters
	lifeTime_ms	 += milliSeconds;
    onTime_ms	 += milliSeconds;
}




static uint64_t RtcGetLifeTime_ms(void)
{
	uint32_t	milliSeconds;

    // Calculate the number of millisecond that have passed since the last callback. Do NOT update the prevRtcTickCount.
    milliSeconds = RtcTick2Ms( RtcTicksSinceMinCallback(false) );

	return lifeTime_ms + milliSeconds; 
}

static uint64_t RtcGetOnTime_ms(void)
{
	uint32_t	milliSeconds;

    // Calculate the number of millisecond that have passed since the last callback. Do NOT update the prevRtcTickCount.
    milliSeconds = RtcTick2Ms( RtcTicksSinceMinCallback(false) );

	return onTime_ms + milliSeconds; 
}


void RtcAppTimer(void)
{
    uint32_t retVal;

    // Create the RTC app timer.
    retVal = app_timer_create(&RTCtimer, APP_TIMER_MODE_SINGLE_SHOT, RtcAlarmIrq);
    APP_ERROR_CHECK(retVal);

	// Create the RTC app timer.
    retVal = app_timer_create(&minTimer, APP_TIMER_MODE_REPEATED, RtcMinuteCallback);
    APP_ERROR_CHECK(retVal);

	// Use the minute timer to keep track of time.
    retVal = app_timer_start(minTimer, APP_TIMER_TICKS(60000), NULL);
    APP_ERROR_CHECK(retVal);
}


void RtcInit( void )
{
    uint32_t ret;

    ret = nrf_drv_clock_init();
    APP_ERROR_CHECK(ret);

    ret = app_timer_init();
    APP_ERROR_CHECK(ret);
    
    nrf_drv_clock_lfclk_request(NULL);
    while(!nrf_drv_clock_lfclk_is_running())
    {
        /* Just waiting */
    }

    RtcAppTimer();
    RtcSetTimerContext( );
    RtcInitialized = true;
}

uint32_t RtcSetTimerContext( void )
{
    RtcTimerContext.Time = app_timer_cnt_get();
    return RtcTimerContext.Time;
}

uint32_t RtcGetTimerContext( void )
{
    return RtcTimerContext.Time;
}

uint32_t RtcGetMinimumTimeout( void )
{
    return( MIN_ALARM_DELAY );
}

uint32_t RtcGetTimerElapsedTicks( void )
{
    uint32_t current_time = RtcGetTimerValue();
    return RtcCalcAbsDiff(current_time, RtcTimerContext.Time);
}


uint32_t RtcCalcAbsDiff( TimerTime_t now, TimerTime_t previous)
{
	if(now > previous)
	{
		return now - previous;
	}
	return previous - now;
}

uint32_t RtcMs2Tick( TimerTime_t milliseconds )
{
    return ( uint32_t ) ((((uint64_t)milliseconds) << 15) / 1000UL);
}

TimerTime_t RtcTick2Ms( uint32_t tick )
{
    uint32_t seconds = tick >> 15;
    uint32_t msTicks = tick - (seconds << 15);
    return ( ( seconds * 1000 ) + ( ( msTicks * 1000 ) >> 15 ) );
}

void RtcDelayMs( TimerTime_t milliseconds )
{
    uint32_t delayTicks = 0;
    uint32_t refTicks = RtcGetTimerValue( );

    delayTicks = RtcMs2Tick( milliseconds );

    // Wait delay ms
    while( ( ( RtcGetTimerValue( ) - refTicks ) ) < delayTicks )
    {
    }
}

void RtcSetAlarm( uint32_t timeout )
{
    RtcStartAlarm( timeout );
}

void RtcStopAlarm( void )
{
    
    RtcTimerContext.AlarmState = ALARM_STOPPED;
    app_timer_stop(RTCtimer);
}

void RtcStartAlarm( uint32_t timeout )
{
    uint32_t retVal;

    RtcStopAlarm( );

    #if 0
		NRF_LOG_INFO("RTC start Alarm: %u Ticks / %u ms", timeout, RtcTick2Ms(timeout));
    #endif

    RtcTimerContext.Delay		= timeout;
    RtcTimeoutPendingInterrupt	= true;
    RtcTimeoutPendingPolling	= false;
    RtcTimerContext.AlarmState	= ALARM_RUNNING;
    retVal = app_timer_start(RTCtimer, RtcTimerContext.Delay, NULL);
}

uint32_t RtcGetTimerValue( void )
{
    return app_timer_cnt_get();
}



uint32_t RtcGetCalendarTime( uint16_t *milliseconds )
{
	uint64_t	onTime_ms = RtcGetOnTime_ms(); 
	uint32_t	seconds;

	// Calculate the milliseconds by performing a module operation of % 1000 to get only the remainder
    *milliseconds	= onTime_ms % 1000;
    seconds			= onTime_ms / 1000;
	NRF_LOG_INFO("On Time: seconds: %u, milliseconds: %u", seconds, (*milliseconds));

    return seconds;
}

void RtcBkupWrite( uint32_t data0, uint32_t data1 )
{
//    CRITICAL_SECTION_BEGIN( );
//    RtcBkupRegisters[0] = data0;
//    RtcBkupRegisters[1] = data1;
//    CRITICAL_SECTION_END( );
}

void RtcBkupRead( uint32_t* data0, uint32_t* data1 )
{
//    CRITICAL_SECTION_BEGIN( );
//    *data0 = RtcBkupRegisters[0];
//    *data1 = RtcBkupRegisters[1];
//    CRITICAL_SECTION_END( );
}

void RtcProcess( void )
{
//    CRITICAL_SECTION_BEGIN( );
//
//    if( (  RtcTimerContext.AlarmState == ALARM_RUNNING ) && ( RtcTimeoutPendingPolling == true ) )
//    {
//        if( RtcGetTimerElapsedTicks( ) >= RtcTimerContext.Delay )
//        {
//            RtcTimerContext.AlarmState = ALARM_STOPPED;
//
//            // Because of one shot the task will be removed after the callback
//            RtcTimeoutPendingPolling = false;
//			#if( RTC_DEBUG_GPIO_STATE == RTC_DEBUG_ENABLE )
//				GpioWrite( &DbgRtcPin0, 0 );
//				GpioWrite( &DbgRtcPin1, 1 );
//			#endif
//            // NOTE: The handler should take less then 1 ms otherwise the clock shifts
//            TimerIrqHandler( );
//
//			#if( RTC_DEBUG_GPIO_STATE == RTC_DEBUG_ENABLE )
//				GpioWrite( &DbgRtcPin1, 0 );
//			#endif
//        }
//    }
//    CRITICAL_SECTION_END( );
}

TimerTime_t RtcTempCompensation( TimerTime_t period, float temperature )
{
    return period;
}

static void RtcAlarmIrq( void * p_context )
{
    UNUSED_VARIABLE(p_context);
    #if 0
		NRF_LOG_INFO("RTC interrupt");
    #endif

    RtcTimerContext.AlarmState = ALARM_STOPPED;

    // Because of one shot the task will be removed after the callback
    RtcTimeoutPendingInterrupt = false;

	#if( RTC_DEBUG_GPIO_STATE == RTC_DEBUG_ENABLE )
		GpioWrite( &DbgRtcPin1, 1 );
	#endif
    // NOTE: The handler should take less then 1 ms otherwise the clock shifts
    TimerIrqHandler( );
	#if( RTC_DEBUG_GPIO_STATE == RTC_DEBUG_ENABLE )
		GpioWrite( &DbgRtcPin1, 0 );
	#endif
}

