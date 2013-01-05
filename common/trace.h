// Copyright (c) 2011-2013 by IMC.  All rights reserved.

#ifndef TRACE_H_INCLUDED
#define TRACE_H_INCLUDED 1

#include "lpc17xx_uart.h"
#include "lpc17xx_libcfg.h"
#include "lpc17xx_pinsel.h"

// Types start with first letter capitalized:

#include "types.h"

// {Trace} data structures and routines:

#define TRACE_BITS 7
#define TRACE_SIZE (1 << TRACE_BITS)
#define TRACE_MASK (TRACE_SIZE - 1)
extern UShort trace_buffer[];
extern UShort trace_index;
#define TRACE(byte) trace_buffer[trace_index++ & TRACE_MASK] = byte

void trace_dump(Serial serial);

#endif // TRACE_H_INCLUDED
