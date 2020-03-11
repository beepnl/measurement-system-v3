/*!
 * \file      timer.c
 *
 * \brief     Timer objects and scheduling management implementation
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
 */
#include "utilities.h"
#include "board.h"
#include "rtc-board.h"
#include "timer.h"
#include "app_timer.h"
#include "nrf_log.h"

/*!
 * Safely execute call back
 */
#define ExecuteCallBack( _callback_, context ) \
    do                                         \
    {                                          \
        if( _callback_ == NULL )               \
        {                                      \
            while( 1 );                        \
        }                                      \
        else                                   \
        {                                      \
            _callback_( context );             \
        }                                      \
    }while( 0 );

/*!
 * Timers list head pointer
 */
static TimerEvent_t *TimerListHead = NULL;

uint32_t TimerInterruptTimestamp = 0;

/*!
 * \brief Adds or replace the head timer of the list.
 *
 * \remark The list is automatically sorted. The list head always contains the
 *         next timer to expire.
 *
 * \param [IN]  obj Timer object to be become the new head
 * \param [IN]  remainingTime Remaining time of the previous head to be replaced
 */
static void TimerInsertNewHeadTimer( TimerEvent_t *obj );

/*!
 * \brief Adds a timer to the list.
 *
 * \remark The list is automatically sorted. The list head always contains the
 *         next timer to expire.
 *
 * \param [IN]  obj Timer object to be added to the list
 * \param [IN]  remainingTime Remaining time of the running head after which the object may be added
 */
static void TimerInsertTimer( TimerEvent_t *obj );

/*!
 * \brief Sets a timeout with the duration "timestamp"
 *
 * \param [IN] timestamp Delay duration
 */
static void TimerSetTimeout( TimerEvent_t *obj );

/*!
 * \brief Check if the Object to be added is not already in the list
 *
 * \param [IN] timestamp Delay duration
 * \retval true (the object is already in the list) or false
 */
static bool TimerExists( TimerEvent_t *obj );

void TimerInit( TimerEvent_t *obj, void ( *callback )( void *context ) )
{
    obj->Timestamp = 0;
    obj->delay_ms = 0;
    obj->IsStarted = false;
    obj->IsNext2Expire = false;
    obj->Callback = callback;
    obj->Context = NULL;
    obj->Next = NULL;
}

static uint32_t TimerSetTickTimestamp(void)
{
	uint32_t current_timestamp = RtcGetTimerValue();
	return current_timestamp;
}

static uint32_t TimerGetTicksSinceLastInterrupt(void)
{
    uint32_t current_timestamp = RtcGetTimerValue();
    return RtcCalcAbsDiff(current_timestamp, TimerInterruptTimestamp);
}

static uint32_t TimerCalcTimestampValue(uint32_t ticks)
{
	uint32_t current_timestamp = RtcGetTimerValue();
	return (current_timestamp + ticks) & APP_TIMER_MAX_CNT_VAL;
}

void TimerSetContext( TimerEvent_t *obj, void* context )
{
    obj->Context = context;
}

uint32_t TimerGetRemainingTicks(uint32_t compareTimestamp, TimerEvent_t *obj)
{
	uint32_t ticksPassed = 0;

	ticksPassed = RtcCalcAbsDiff(compareTimestamp, obj->Timestamp);

	// Check if the delay has passed. return 0 when the delay is passed.
	if(ticksPassed >= obj->delay_ticks){
		return 0;
	}
	return obj->delay_ticks - ticksPassed;
}

void TimerStart( TimerEvent_t *obj )
{
	uint32_t nowTimestamp, remainingTicksObj, remainingTicksHead;
    uint8_t nested;
	app_util_critical_region_enter(&nested);

    if( ( obj == NULL ) || ( TimerExists( obj ) == true ) )
    {
        app_util_critical_region_exit(nested);
        return;
    }

	nowTimestamp		= RtcGetTimerValue();
	obj->Timestamp		= nowTimestamp; // Remember the RTC timestamp when the timer is started
    obj->IsStarted		= true;
    obj->IsNext2Expire	= false;

    if( TimerListHead == NULL )
    {
        TimerSetTickTimestamp( );
        // Inserts a timer at time now + obj->Timestamp
        TimerInsertNewHeadTimer( obj );
    }
    else
    {
		remainingTicksObj	= TimerGetRemainingTicks(nowTimestamp, obj);
        remainingTicksHead	= TimerGetRemainingTicks(nowTimestamp, TimerListHead);

		#if 0
			NRF_LOG_INFO("-Timerstart, %s, TicksObj=%u, TicksHead=%u",  (remainingTicksObj < remainingTicksHead) ? "insert Head Timer" : "insert new Timer", remainingTicksObj, remainingTicksHead);
		#endif

		// Determine the timer with the least number of ticks left to count down.
		if( remainingTicksObj < remainingTicksHead )
        {
            TimerInsertNewHeadTimer( obj );
        }
        else
        {
            TimerInsertTimer( obj );
        }
    }
    app_util_critical_region_exit(nested);
}



static void TimerInsertTimer( TimerEvent_t *obj )
{
    TimerEvent_t* cur = TimerListHead;
    TimerEvent_t* next = TimerListHead->Next;
	uint32_t nowTimestamp, remainingTicksObj, remainingTicksNext;

    nowTimestamp		= RtcGetTimerValue();
    remainingTicksObj	= TimerGetRemainingTicks(nowTimestamp, obj);

    while( cur->Next != NULL )
    {
		
        remainingTicksNext	= TimerGetRemainingTicks(nowTimestamp, next);
        if( remainingTicksObj > remainingTicksNext )
        {
            cur		= next;
            next	= next->Next;
        }
        else
        {
            cur->Next = obj;
            obj->Next = next;
            return;
        }
    }
    cur->Next = obj;
    obj->Next = NULL;
}

static void TimerInsertNewHeadTimer( TimerEvent_t *obj )
{
    TimerEvent_t* cur = TimerListHead;

    if( cur != NULL )
    {
        cur->IsNext2Expire = false;
    }

    obj->Next = cur;
    TimerListHead = obj;
    TimerSetTimeout( TimerListHead );
}

bool TimerIsStarted( TimerEvent_t *obj )
{
    return obj->IsStarted;
}

void TimerIrqHandler( void )
{
    TimerEvent_t* cur;
    TimerEvent_t* next;

    uint32_t old =  TimerInterruptTimestamp;
    uint32_t now =  TimerSetTickTimestamp( );
	uint32_t timeStampNow = 0;

    // Remove all the expired object from the list
    timeStampNow = RtcGetTimerValue();

	#if 0
		NRF_LOG_INFO("Timer callback timestamp %u", timeStampNow);
	#endif
    while( ( TimerListHead != NULL ) && ( (RtcCalcAbsDiff(timeStampNow, TimerListHead->Timestamp) + 5) >= TimerListHead->delay_ticks ) ) // Calculate the absolute number of ticks that have passed since the start of the timer and compare if that is larger or equal to the number of delay ticks
    {
        cur = TimerListHead;
        TimerListHead = TimerListHead->Next;
        cur->IsStarted = false;
        ExecuteCallBack( cur->Callback, cur->Context );
        timeStampNow = RtcGetTimerValue();
    }

    // Start the next TimerListHead if it exists AND NOT running
    if( ( TimerListHead != NULL ) && ( TimerListHead->IsNext2Expire == false ) )
    {
        TimerSetTimeout( TimerListHead );
    }
}

void TimerStop( TimerEvent_t *obj )
{
    uint8_t nested;
	app_util_critical_region_enter(&nested);

    TimerEvent_t* prev = TimerListHead;
    TimerEvent_t* cur = TimerListHead;

    // List is empty or the obj to stop does not exist
    if( ( TimerListHead == NULL ) || ( obj == NULL ) )
    {
        app_util_critical_region_exit(nested);
        return;
    }

    obj->IsStarted = false;

    if( TimerListHead == obj ) // Stop the Head
    {
        if( TimerListHead->IsNext2Expire == true ) // The head is already running
        {
            TimerListHead->IsNext2Expire = false;
            if( TimerListHead->Next != NULL )
            {
                TimerListHead = TimerListHead->Next;
                TimerSetTimeout( TimerListHead );
            }
            else
            {
                RtcStopAlarm( );
                TimerListHead = NULL;
            }
        }
        else // Stop the head before it is started
        {
            if( TimerListHead->Next != NULL )
            {
                TimerListHead = TimerListHead->Next;
            }
            else
            {
                TimerListHead = NULL;
            }
        }
    }
    else // Stop an object within the list
    {
        while( cur != NULL )
        {
            if( cur == obj )
            {
                if( cur->Next != NULL )
                {
                    cur = cur->Next;
                    prev->Next = cur;
                }
                else
                {
                    cur = NULL;
                    prev->Next = cur;
                }
                break;
            }
            else
            {
                prev = cur;
                cur = cur->Next;
            }
        }
    }
    app_util_critical_region_exit(nested);
}

static bool TimerExists( TimerEvent_t *obj )
{
    TimerEvent_t* cur = TimerListHead;

    while( cur != NULL )
    {
        if( cur == obj )
        {
            return true;
        }
        cur = cur->Next;
    }
    return false;
}

void TimerReset( TimerEvent_t *obj )
{
    TimerStop( obj );
    TimerStart( obj );
}

void TimerSetValue( TimerEvent_t *obj, uint32_t delay_ms )
{
    uint32_t minValue = 0;
    uint32_t ticks = RtcMs2Tick( delay_ms );

    TimerStop( obj );

    minValue = RtcGetMinimumTimeout( );

    if( ticks < minValue )
    {
        ticks = minValue;
    }

    obj->delay_ms		= delay_ms;
	obj->delay_ticks	= ticks;
}

TimerTime_t TimerGetCurrentTime( void )
{
    uint32_t now = RtcGetTimerValue( );
    return  RtcTick2Ms( now );
}

TimerTime_t TimerGetElapsedTime( TimerTime_t past )
{
	static uint32_t diffTicks = 0;
    uint32_t nowInTicks	 = RtcGetTimerValue( );
    uint32_t pastInTicks = RtcMs2Tick( past );

    // Intentional wrap around. Works Ok if tick duration below 1ms
	diffTicks = RtcCalcAbsDiff(nowInTicks, pastInTicks);
    return RtcTick2Ms( diffTicks );
}

uint32_t TimerGetTimestamp (void)
{
	return RtcGetTimerValue();
}

static void TimerSetTimeout( TimerEvent_t *obj )
{
	uint32_t remainingTicks;
	uint32_t nowTimestamp	= RtcGetTimerValue( );
    uint32_t minTicks		= RtcGetMinimumTimeout( );
    obj->IsNext2Expire		= true;
    remainingTicks			= TimerGetRemainingTicks( nowTimestamp, obj );

	#if 0
		NRF_LOG_INFO("TimerSetTimeout: stamp=%u, remainingTicks=%u, %u ms",
						nowTimestamp,
						remainingTicks,
						RtcTick2Ms(remainingTicks));
	#endif

	if( remainingTicks <= minTicks )
	{
		// The event is most likely already passed or about to within the minimum number of ticks.
        RtcSetAlarm( minTicks );
        NRF_LOG_INFO(">Timer toosoon stamp=%u, current stamp=%u, delayTicks=%u", obj->Timestamp, nowTimestamp, obj->delay_ticks);
	}
	else
	{
		// Start the RTC and wait for the number of ticks that are remaining for this timer.
		RtcSetAlarm( remainingTicks );
	}
}

TimerTime_t TimerTempCompensation( TimerTime_t period, float temperature )
{
    return RtcTempCompensation( period, temperature );
}

void TimerProcess( void )
{
    RtcProcess( );
}
