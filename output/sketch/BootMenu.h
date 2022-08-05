#line 1 "c:\\Marien\\Sources\\Arduino\\BootMenu.h"
#ifndef BOOTMENU_H_
#define BOOTMENU_H_

#include <Arduino.h>

typedef void(*VoidCallbackMethodPtr)(void);

void showBootMenu(Stream& stream);
void setResetDevAddrOrEUItoHWEUICallback(VoidCallbackMethodPtr callback);

#endif