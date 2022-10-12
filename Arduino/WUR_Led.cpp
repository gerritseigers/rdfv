/*=================================================================================================
 * File:            WUR_Led.c
 * Author:          Jeroen de Pagter
 * Created date:    June 17, 2020, 3:55 PM
 * Description:     Control a LED output
=================================================================================================*/

//=================================================================================================
// Includes
//=================================================================================================
#include "WUR_Global.h"
#include "WUR_Time.hpp"

#include "WUR_Led.hpp"

//=================================================================================================
// File scope defines
//=================================================================================================

//=================================================================================================
// File scope typedefs
//=================================================================================================
static const WU32 LED_ACTIVE_TIME = WTIME::MSEC2WTIME(25);
static const WU32 BLINK_INTERVAL_SLOW = WTIME::SEC2WTIME(3);
static const WU32 BLINK_INTERVAL_FAST = WTIME::MSEC2WTIME(500);

//=================================================================================================
// File scope functions declaration
//=================================================================================================

//=================================================================================================
// File scope variable declaration
//=================================================================================================

//=================================================================================================
// Global functions definitions
//=================================================================================================
//=================================================================================================
// Arguments:   -
// Return:      -
// Description: Constructor
//=================================================================================================
WLED::WLED(const WU32 Interval_cu32)
{
    m_LedOutput_b = false;
    m_LedUpdate_b = false;
    m_Status_e = LED_OFF;
    m_BlinkTimer_t.Set(0);
    m_OnTimer_t.Set(0);
    m_BlinkInterval_u32 = Interval_cu32;
}

//=================================================================================================
// Arguments:   LedControl_ps - pointer to LedControl object
// Return:      WU1 - Return the Ledoutput indicator
// Description: Control the led operation and return the led output active/inactive
//=================================================================================================
WU1 WLED::Run(void)
{	
    switch (m_Status_e)
    {
        case LED_OFF:
        {
            m_LedOutput_b = false;
            break;
        }
        case LED_ON:
        {
            m_LedOutput_b = true;
            break;
        }
        case LED_FAST_BLINK:
        case LED_SLOW_BLINK:
		case LED_BLINK:
        case LED_SINGLE_BLINK:
        {
            if ( (m_LedUpdate_b == true) ||
                 (m_BlinkTimer_t.HasExpired() == true) )
            {
                if (m_LedUpdate_b == true)
                {
                    m_BlinkTimer_t.Set (m_BlinkInterval_u32);
                    m_OnTimer_t.Set(LED_ACTIVE_TIME);
                }
                else
                {
                    // Reset timer for interval
                    //(void)gfTIME_Add (&m_BlinkTimer_t, m_BlinkInterval_u16);
                    m_OnTimer_t.Set(LED_ACTIVE_TIME);
                }
                // Set output active at start of blink
                m_LedOutput_b = true;
            }
            else if (m_OnTimer_t.HasExpired() == true)
            {
                if (m_Status_e == LED_SINGLE_BLINK)
                {
                    // Kill the blinking!
                    m_Status_e = LED_OFF;
                }
                // Keep timer expired until next blink
                m_OnTimer_t.Set(0);
                m_LedOutput_b = false;
            }
            break;
        }   // endof: case LED_FAST_BLINK / LED_SLOW_BLINK
    }
    
    // Set new status handled!
    m_LedUpdate_b = false;
    
    return m_LedOutput_b;
}

//=================================================================================================
// Arguments:   The new status for the Led
// Return:      -
// Description: Set the LED to a status. 
//=================================================================================================
void WLED::Set (const LED_STATUS_E NewStatus_ce, const WU32 Interval_cu32)
{
    if (m_Status_e != NewStatus_ce) 
    {
        // Reset status! A new operation is needed
        m_LedUpdate_b = true;
        
        m_Status_e = NewStatus_ce;
        if ( (NewStatus_ce == LED_FAST_BLINK) ||
             (NewStatus_ce == LED_SINGLE_BLINK) )
        {
            m_BlinkInterval_u32 = BLINK_INTERVAL_FAST;
        }
        else if (NewStatus_ce == LED_SLOW_BLINK)
        {
            m_BlinkInterval_u32 = BLINK_INTERVAL_SLOW;
        }
        else if (NewStatus_ce == LED_BLINK)
        {
            m_BlinkInterval_u32 = Interval_cu32;
        }
    }
    else if ( (NewStatus_ce == LED_BLINK) &&
              (m_BlinkInterval_u32 != Interval_cu32) )
    {
        m_LedUpdate_b = true;
        m_BlinkInterval_u32 = Interval_cu32;
    }
}
    
//=================================================================================================
// Local functions definitions
//=================================================================================================


