#line 1 "c:\\Projects\\rdfv\\Arduino\\WUR_Led.hpp"
//=================================================================================================
// File:            LED_LedControl.h
// Author:          Jeroen de Pagter
// Created date:    June 17, 2020, 3:46 PM
// Description:     Create LED operation on an output pin
//=================================================================================================

#ifndef LED_LEDCONTROL_H
#define	LED_LEDCONTROL_H

//=================================================================================================
// Includes
//=================================================================================================
#include "WUR_Global.h"
#include "WUR_Time.hpp" 

//=================================================================================================
// Global defines
//=================================================================================================
#ifdef	__cplusplus
extern "C"
{
#endif

//=================================================================================================
// Global typedefs
//=================================================================================================

//=================================================================================================
// Forward class definition / namespace
//=================================================================================================

//=================================================================================================
// Class definition
//=================================================================================================
class WLED
{
public:	
    enum LED_STATUS_E
	{
		LED_OFF,
		LED_ON,
		LED_FAST_BLINK,
		LED_SLOW_BLINK,
		LED_BLINK,	// User must set m_BlinkInterval_u16 on it's own!!
		LED_SINGLE_BLINK
    } ;

    // enum LED_COLOUR_E
	// {
	// 	LED_WHITE = 0,
	// 	LED_BLUE = 1,
	// 	LED_GREEN,
	// 	LED_RED
    // } ;

	WLED(const WU32 Interval_cu32 = 0);
	WU1 Run(void);
	void Set(const LED_STATUS_E NewStatus_ce, const WU32 Interval_cu16 = 0);
    WU1 Output(void) { return m_LedOutput_b; }

private:
	LED_STATUS_E    m_Status_e;       // What is the status of the led
//	LED_COLOUR_E    m_Colour_e;
	WU32            m_BlinkInterval_u32;  // Time between LED on pulses
	WTIME           m_BlinkTimer_t;     // Timer to be used for blinking interval
	WTIME           m_OnTimer_t;      // Time the LED needs to remain ON for Blinking
	WU1             m_LedUpdate_b;    // Set to true if a status change is issued (restart timers etc)
	WU1             m_LedOutput_b;    // Value for the output! (true should yield a active LED
};

#ifdef	__cplusplus
}
#endif

#endif	// LED_LEDCONTROL_H

