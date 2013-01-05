// Copyright (c) 2011-2013 by IMC.  All rights reserved.

#ifndef SYSTICK_H_INCLUDED
#define SYSTICK_H_INCLUDED 1

// Types start with first letter capitalized:

#include "types.h"
// SysTick routines:
volatile unsigned long SysTickCnt;
void SysTick__delay(unsigned long tick);
void SysTick_Handler(void);

#endif // SYSTICK_H_INCLUDED
