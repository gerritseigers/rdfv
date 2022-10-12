//=================================================================================================
// File:            WUR_ArdTime.hpp
// Author:          J. de Pagter
// Author:          Jeroen de Pagter
// Created date:    May 28, 2020, 4:21 PM
// Description:     Module used to support basic 1 us timer operations. The Arduino board have 
// different usec interval timers, s [16Mhz e.g. Nano = 4 usec resolution, 8 Mhz e.g. LilyPad = 8 usec resolution
//=================================================================================================

#ifndef WUR_TIME_HPP
#define	WUR_TIME_HPP

//=================================================================================================
// Includes
//=================================================================================================
#include "WUR_Global.h"

//=================================================================================================
// Global defines
//=================================================================================================

//=================================================================================================
// Global typedefs
//=================================================================================================

//=================================================================================================
// Forward class definition / namespace
//=================================================================================================

//=================================================================================================
// Class definition
//=================================================================================================
class WTIME
{
	static const WU32 TIMEREXPIRED_CNT =   0x03938700ul;    // Maximum overflow / expired value is 1 min
	static const WU32 MAX_TIME_VALUE =     0xD693A400ul;	// maximum timer capabilities = 60 min = 60 * 60 * 1000 * 1000
	static const WU32 BASETIME_INCREMENT = 	1000000;	// Set base interval to 1 sec

// Functions:
public:
	WTIME(WU32 TimerInterval_u32 = BASETIME_INCREMENT);
	
	WU1 	HasExpired(WU1 AutoAdd_b = true);				// Check if the timer has expired
	WU1 	Set(const WU32 uSec_cu32 = BASETIME_INCREMENT);	// Set a new timestamp from the current micros() value
    WU1     Reset(void);
	WU1 	Add(const WU32 uSec_cu32 = BASETIME_INCREMENT);	// Increment the current timestamp with the interval chosen

  // Static helper function
  static WU32 MSEC2WTIME(const WU32 msec_cu32) { return (msec_cu32 * 1000); };  
  static WU32 SEC2WTIME(const WU32 usec_cu32) { return (usec_cu32 * 1000 * 1000); };
  
private:
	WU32 	m_TimeStamp_u32 = 0;
	WU32	m_TimeInterval_u32 = 0;
};

#endif	// WUR_TIME_H
