#line 1 "c:\\Projects\\rdfv\\Arduino\\WUR_Global.h"
//=================================================================================================
// File:            WUR_types.h
// Author:          Jeroen de Pagter
// Created date:    March 24, 2020, 3:32 PM
// Description:     Setup a compiler independent variable type for fixed size 
// variables
//=================================================================================================

#ifndef WUR_GLOBAL_H
#define	WUR_GLOBAL_H

//=================================================================================================
// Includes
//=================================================================================================

//=================================================================================================
// Global defines
//=================================================================================================

//=================================================================================================
// Global typedefs
//=================================================================================================

#ifndef false
#define false 0
#endif

#ifndef true
#define true !false
#endif

#ifdef ARDUINO
#include <arduino.h>

// For WUR typing:
typedef uint_least8_t WU8;
typedef int_least8_t WS8;

typedef uint_least16_t WU16;
typedef int_least16_t WS16;

//typedef uint_least24_t WU24;
//typedef int_least24_t WS24;

typedef uint_least32_t WU32;
typedef int_least32_t WS32;

typedef uint_least64_t WU64;
typedef int_least64_t WS64;

typedef float WFloat32;
typedef double WDouble64;
typedef bool WU1;   

// Qt GNU compiler
#elif QT_IS_AVAILABLE
#include <QtGlobal>

typedef quint8 WU8;
typedef qint8 WS8;

typedef quint16 WU16;
typedef qint16 WS16;

typedef quint32 WU32;
typedef qint32 WS32;

typedef quint64 WU64;
typedef qint64 WS64;

// typedef bool WU1;
typedef unsigned char WU1;  // C++ may not have variables smaller then 8 bits

typedef float WFloat32;
typedef double WDouble64;

// MICROCHIP XC8 compiler
#elif __XC8
typedef bit WU1;
typedef signed char WS8;
typedef unsigned char WU8;
//typedef signed short WS16;
//typedef unsigned short WU16;
typedef signed int WS16;
typedef unsigned int WU16;
typedef signed short long WS24;
typedef unsigned short long WU24;
typedef signed long WS32;
typedef unsigned long WU32;
// typedef signed long long WS32;
// typedef unsigned long long WU32;
typedef float WFloat32;
typedef double WDouble32;

// MICROCHIP XC16 compiler
#elif __XC16
typedef signed char WS8;
typedef unsigned char WU8;
// typedef short WS16;
// typedef unsigned short WS16;
typedef signed int WS16;
typedef unsigned int WU16;
typedef signed long WS32;
typedef unsigned long WU32;
typedef signed long long WS64;
typedef unsigned long long WU64;

typedef float WFloat32;
typedef double WDouble32;
typedef long double WDouble64;
#endif

//=================================================================================================
// Global functions declarations
//=================================================================================================
#ifdef	__cplusplus
extern "C" {
#endif

#ifdef	__cplusplus
}
#endif

#endif	/* WUR_ARDGLOBAL_H */
