#ifndef SYSTICK_H_INCLUDED
#define SYSTICK_H_INCLUDED 1

// Copyright (c) 2011-2013 by IMC.  All rights reserved.

#include "types.h"

// SysTick routines:
volatile UInteger SysTickCnt;
void SysTick_delay(UInteger tick);
void SysTick_Handler(void);

#endif // SYSTICK_H_INCLUDED
