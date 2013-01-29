#ifndef RING_BUFFER_H_INCLUDED
#define RING_BUFFER_H_INCLUDED 1

// Copyright (c) 2011-2013 by IMC.  All rights reserved.

typedef struct Ring_Buffer__Struct *Ring_Buffer;

#include "types.h"
#include "uart.h"

// A Ring Buffer:
#define RING_BUFFER__SIZE 32
#define RING_BUFFER__MASK (RING_BUFFER__SIZE - 1)

struct Ring_Buffer__Struct {
    volatile UInt8 head;			// Ring buffer head index
    volatile UInt8 tail;			// Ring buffer tail index
    volatile Frame frames[RING_BUFFER__SIZE];	// Ring buffer frame data
};

// {Ring_Buffer} routines:
Ring_Buffer Ring_Buffer__initialize(Ring_Buffer ring_buffer);
UInt8 Ring_Buffer__is_full(Ring_Buffer ring_buffer);
UInt8 Ring_Buffer__is_empty(Ring_Buffer ring_buffer);
void Ring_Buffer__head_increment(Ring_Buffer ring_buffer);
void Ring_Buffer__tail_increment(Ring_Buffer ring_buffer);

#endif // RING_BUFFER_H_INCLUDED
