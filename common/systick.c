// Copyright (c) 2011-2013 by IMC.  All rights reserved.

#include "systick.h"

// SysTick routines:

// This interrupt routine will increment {SysTickCnt} every millisecond.
void SysTick_Handler (void) {
    SysTickCnt++;
}

// This routine will delay for {ms} milliseconds.
void SysTick_delay (
  unsigned long ms)
{
    unsigned long sys_tick_count;

    // Get current value:
    sys_tick_count = SysTickCnt;

    // Wait until we get the value we want:
    while ((SysTickCnt - sys_tick_count) < ms) {
	// do nothing:
    }
}

