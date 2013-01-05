// Copyright (c) 2011-2013 by IMC.  All rights reserved.

#ifndef RING_BUFFER_H_INCLUDED
#define RING_BUFFER_H_INCLUDED 1

// Types start with first letter capitalized:

#include "types.h"

typedef struct Ring_Buffer__Struct *Ring_Buffer;

// A Ring Buffer:
#define RING_BUFFER__SIZE 32
#define RING_BUFFER__MASK (RING_BUFFER__SIZE - 1)

struct Ring_Buffer__Struct {
    __IO UByte head;			// Ring buffer head index
    __IO UByte tail;			// Ring buffer tail index
    __IO Frame frames[RING_BUFFER__SIZE]; // Ring buffer frame data
};

// {Ring_Buffer} routines:
Ring_Buffer Ring_Buffer__initialize(Ring_Buffer ring_buffer);
UByte Ring_Buffer__is_full(Ring_Buffer ring_buffer);
UByte Ring_Buffer__is_empty(Ring_Buffer ring_buffer);
void Ring_Buffer__head_increment(Ring_Buffer ring_buffer);
void Ring_Buffer__tail_increment(Ring_Buffer ring_buffer);

#endif // RING_BUFFER_H_INCLUDED
