#line 1 "c:\\Projects\\rdfv\\Arduino\\WUR_Time.cpp"
//=================================================================================================
// File:            WUR_ArdTime.cpp
// Author:          Jeroen de Pagter
// Created date:    May 28, 2020, 4:21 PM
// Description:     Class used to support basic 1 us timer operations. The Arduino board have 
// different usec interval timers, s [16Mhz e.g. Nano = 4 usec resolution, 8 Mhz e.g. LilyPad = 8 usec resolution
//=================================================================================================

//=================================================================================================
// Includes
//=================================================================================================
#include "WUR_Global.h"
#include "WUR_Time.hpp"

//=================================================================================================
// File scope defines
//=================================================================================================
#ifdef QT_IS_AVAILABLE
WU32 micros(void) { return 0; }
#endif

//=================================================================================================
// public functions declaration
//=================================================================================================
//=========================================================================================
// Function:        CONSTRUCTOR
// PreCondition:    Running time arduino board
// Input:           New time value or default argument
// Output:          -
// Side Effects:    None
// Description:     None
//===================================================================================
WTIME::WTIME (WU32 TimerInterval_u32):
	m_TimeStamp_u32(0),
	m_TimeInterval_u32(TimerInterval_u32)
{	
	Set(TimerInterval_u32);
}

//=========================================================================================
// Function:        Expired
// PreCondition:    Running time arduino board
// Input:           AutoAdd_b - When timer has expired, use the last set value to increment the timer when expired has been read
// Output:          WU1
// Side Effects:    None
// Description:     None
//===================================================================================
WU1 WTIME::HasExpired (WU1 AutoAdd_b)
{
    WU1 Expired_b = false;
    
    if ( (micros() - m_TimeStamp_u32) <= TIMEREXPIRED_CNT)
    {
        Expired_b = true;
		if (AutoAdd_b == true)
		{
			Add(m_TimeInterval_u32);
		}
    }		
    
    return Expired_b;
}

//=========================================================================================
// Function:        Set
// PreCondition:    Running time arduino board
// Input:           None
// Output:          WU32 - returned value for timer expired value added to current micros value
// Side Effects:    None
// Description:     Calculate a new expired value for the given timer (advice: use only for first time
// initialization for timer, best to use Add function to avoid prolonged timer interval due to execution times
//===================================================================================
WU1 WTIME::Set(const WU32 uSec_cu32)
{
    if (uSec_cu32 <= MAX_TIME_VALUE)
    {
        m_TimeStamp_u32 = (micros() + uSec_cu32);
		m_TimeInterval_u32 = uSec_cu32;
    }

    return true;
}

//=================================================================================================
// Function:    Reset
// Arguments:   WU1 - true if reset was completed
// Return:
// Description:
//=================================================================================================
WU1 WTIME::Reset(void)
{
    // Reset the timer to a new expired value with the given TimeInterval (set/constructor)
    m_TimeStamp_u32 = (micros() + m_TimeInterval_u32);

    return true;
}

//=========================================================================================
// Function:        void Add(const WU32 Time_cu32)
// PreCondition:    Running time arduino board
// Input:           Time value for next expired indication
// Output:          WU1 - true if succesfull
// Side Effects:    None
// Description:     Increment the timer with the time value to support
// missed usec ticks if slow operations are executed, so interval is garantueed
//===================================================================================
WU1 WTIME::Add(const WU32 uSec_cu32)
{
    WU1 Succes_b = false;
    
    if (uSec_cu32 < MAX_TIME_VALUE)
    {
		// Increment current time value for next expired trigger
        m_TimeStamp_u32 = (m_TimeStamp_u32 + uSec_cu32);

        if ( (m_TimeStamp_u32 - micros()) > TIMEREXPIRED_CNT)
        {
            // The timer is not exceding current elapsed usec, so set to this usec value
            m_TimeStamp_u32 = micros();
        }
    
        Succes_b = true;
    }
    
    return Succes_b; 
}

//=================================================================================================
// Overloaded functions declaration
//=================================================================================================

//=================================================================================================
// private functions definitions
//=================================================================================================
