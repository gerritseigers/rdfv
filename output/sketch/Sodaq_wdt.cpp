#line 1 "c:\\Projects\\rdfv\\Arduino\\Sodaq_wdt.cpp"
/*
Copyright (c) 2016-19, SODAQ
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#include <Arduino.h>
#include "Sodaq_wdt.h"

#ifdef ARDUINO_ARCH_SAMD

// The generic clock provider for the WDT
// Arduino Zero default GCLK configs
// see cores/startup.c for details
// GCLK0   DFLL48M DIV1 = 48mhz
// GCLK1   XOSC32K DIV1 = 32khz
// GCLK2
// GCLK3   OSC8M   DIV1 = 8mhz
#define WDT_GCLK    4

#endif

volatile bool sodaq_wdt_flag = false;

void sodaq_wdt_enable(wdt_period period)
{
#ifdef ARDUINO_ARCH_AVR

  // From TPH_Demo/MyWatchdog.cpp
  // Both WDE and WDIE
  __asm__ __volatile__ (  \
      "in __tmp_reg__,__SREG__" "\n\t"    \
      "cli" "\n\t"    \
      "wdr" "\n\t"    \
      "sts %0,%1" "\n\t"  \
      "out __SREG__,__tmp_reg__" "\n\t"   \
      "sts %0,%2" "\n\t" \
      : /* no outputs */  \
      : "M" (_SFR_MEM_ADDR(_WD_CONTROL_REG)), \
        "r" (_BV(_WD_CHANGE_BIT) | _BV(WDE)), \
        "r" ((uint8_t) (((period & 0x08) ? _WD_PS3_MASK : 0x00) | \
            _BV(WDE) | _BV(WDIE) | (period & 0x07)) ) \
      : "r0"  \
  );

#elif defined(ARDUINO_ARCH_SAMD)

  // Here we use normal mode with  the early warning interrupt
  // enabled. The early warning period is defined by the parameter
  // 'period' and the reset is set to twice that value.

  // Turn the power to the WDT module on
  PM->APBAMASK.reg |= PM_APBAMASK_WDT;

  // We cannot configure the WDT if it is already in always on mode
  if (!(WDT->CTRL.reg & WDT_CTRL_ALWAYSON)) {

    // Setup clock provider WDT_GCLK with a 32 source divider
    // GCLK_GENDIV_ID(X) specifies which GCLK we are configuring
    // GCLK_GENDIV_DIV(Y) specifies the clock prescalar / divider
    // If GENCTRL.DIVSEL is set (see further below) the divider
    // is 2^(Y+1). If GENCTRL.DIVSEL is 0, the divider is simply Y
    // This register has to be written in a single operation
    GCLK->GENDIV.reg = GCLK_GENDIV_ID(WDT_GCLK) |
                       GCLK_GENDIV_DIV(4);

    // Configure the GCLK module
    // GCLK_GENCTRL_GENEN, enable the specific GCLK module
    // GCLK_GENCTRL_SRC_OSCULP32K, set the source to the OSCULP32K
    // GCLK_GENCTRL_ID(X), specifies which GCLK we are configuring
    // GCLK_GENCTRL_DIVSEL, specify which prescalar mode we are using
    // Output from this module is 1khz (32khz / 32)
    // This register has to be written in a single operation.
    GCLK->GENCTRL.reg = GCLK_GENCTRL_GENEN |
                        GCLK_GENCTRL_SRC_OSCULP32K |
                        GCLK_GENCTRL_ID(WDT_GCLK) |
                        GCLK_GENCTRL_DIVSEL;
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

    // Configure the WDT clock
    // GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_WDT), specify the WDT clock
    // GCLK_CLKCTRL_GEN(WDT_GCLK), specify the source from the WDT_GCLK GCLK
    // This register has to be written in a single operation
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_WDT) |
                        GCLK_CLKCTRL_GEN(WDT_GCLK) |
                        GCLK_CLKCTRL_CLKEN;
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

    // Disable the module before configuring
    WDT->CTRL.reg &= ~WDT_CTRL_ENABLE;
    while (WDT->STATUS.reg & WDT_STATUS_SYNCBUSY);

    // Disable windowed mode
    WDT->CTRL.reg &= ~WDT_CTRL_WEN;
    while (WDT->STATUS.reg & WDT_STATUS_SYNCBUSY);

    // Set the reset period to twice that of the
    // specified interrupt period
    WDT->CONFIG.reg = WDT_CONFIG_PER(period + 1);

    // Set the early warning as specified by the period
    WDT->EWCTRL.reg = WDT_EWCTRL_EWOFFSET(period);

    // Enable the WDT module
    WDT->CTRL.reg |= WDT_CTRL_ENABLE;
    while (WDT->STATUS.reg & WDT_STATUS_SYNCBUSY);

    // Enable early warning interrupt
    WDT->INTENSET.reg = WDT_INTENSET_EW;

    // Enable interrupt vector for WDT
    // Priority is set to 0x03, the lowest
    NVIC_EnableIRQ(WDT_IRQn);
    NVIC_SetPriority(WDT_IRQn, 0x03);
  }

#endif
}

void sodaq_wdt_disable()
{
#ifdef ARDUINO_ARCH_AVR

  // Using avr/wdt.h
  wdt_disable();

#elif defined(ARDUINO_ARCH_SAMD)

  // Disable the WDT module
  WDT->CTRL.reg &= ~WDT_CTRL_ENABLE;
  while (WDT->STATUS.reg & WDT_STATUS_SYNCBUSY);

  // Turn off the power to the WDT module
  PM->APBAMASK.reg &= ~PM_APBAMASK_WDT;

#endif
}

// Resets the WDT counter
void sodaq_wdt_reset()
{
#ifdef ARDUINO_ARCH_AVR

  // Using avr/wdt.h
  wdt_reset();

  // Should this be called once per interrupt,
  // or are we ok calling it with every reset?
  WDTCSR |= _BV(WDIE);

#elif defined(ARDUINO_ARCH_SAMD)

  // Wait if currently syncing, reset counter and wait for synchronisation
  while (WDT->STATUS.reg & WDT_STATUS_SYNCBUSY);
  WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
  while (WDT->STATUS.reg & WDT_STATUS_SYNCBUSY);

#endif
}

void sodaq_wdt_safe_delay(uint32_t ms)
{
  // Delay step size
  uint32_t delay_step = 10;
  uint32_t endMS = millis() + ms;

  // Loop through and reset between steps
  while (millis() < endMS) {
    sodaq_wdt_reset();

    int32_t remaining = (endMS - millis());
    delay((remaining  > delay_step) ? delay_step : remaining);
  }
}

#ifdef ARDUINO_ARCH_AVR

// AVR WDT ISR
ISR(WDT_vect)
{
  sodaq_wdt_flag = true;
}

#elif defined(ARDUINO_ARCH_SAMD)

// SAMD WDT ISR
// This handler is triggered by the early
// warning interrupt
void WDT_Handler(void)
{
  sodaq_wdt_flag = true;

  // Clear the early warning interrupt flag
  WDT->INTFLAG.reg = WDT_INTFLAG_EW;
}

#endif
