// Copyright (c) 2011-2012 by IMC.  All rights reserved.

#ifndef SYSTICK_H_INCLUDED
#define SYSTICK_H_INCLUDED 1

// SysTick routines:
volatile unsigned long SysTickCnt;
void SysTick__delay(unsigned long tick);
void SysTick_Handler(void);

#endif // SYSTICK_H_INCLUDED
