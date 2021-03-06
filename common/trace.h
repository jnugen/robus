#ifndef TRACE_H_INCLUDED
#define TRACE_H_INCLUDED 1

// Copyright (c) 2011-2013 by IMC.  All rights reserved.

#include "serial.h"
#include "types.h"
#include "uart.h"

#define TRACE_BITS 7
#define TRACE_SIZE (1 << TRACE_BITS)
#define TRACE_MASK (TRACE_SIZE - 1)
extern UInt16 trace_buffer[];
extern UInt16 trace_index;
#define TRACE(byte) trace_buffer[trace_index++ & TRACE_MASK] = byte
//#define TRACE(byte) // do nothing

// trace routines:
void trace_dump(Serial serial);

#endif // TRACE_H_INCLUDED
