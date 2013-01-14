// Copyright (c) 2011-2013 by IMC.  All rights reserved.

#include "ring_buffer.h"

// {Ring_Buffer} routines:

Ring_Buffer Ring_Buffer__initialize(
  Ring_Buffer ring_buffer)
{
    // This routine will initialize {ring_buffer} to be empty.
    // The value passed in as {ring_buffer} is returned.

    ring_buffer->head = 0;
    ring_buffer->tail = 0;
    return ring_buffer;
}

Logical Ring_Buffer__is_empty(Ring_Buffer ring_buffer)
{
    UByte result;

    result = 0;
    if ((ring_buffer->head & RING_BUFFER__MASK) ==
      (ring_buffer->tail & RING_BUFFER__MASK)) {
	result = 1;
    }
    return result;
}

void Ring_Buffer__head_increment(
  Ring_Buffer ring_buffer)
{
    ring_buffer->head = (ring_buffer->head + 1) & RING_BUFFER__MASK;
}

// This routine will return 1 if {ring_buffer} is full, and 0 otherise.
UByte Ring_Buffer__is_full(
  Ring_Buffer ring_buffer)
{
    UByte result;

    result = 0;
    if ((ring_buffer->tail & RING_BUFFER__MASK) ==
      ((ring_buffer->head + 1) & RING_BUFFER__MASK)) {
	result = 1;
    }
    return result;
}

void Ring_Buffer__tail_increment(
  Ring_Buffer ring_buffer)
{
    ring_buffer->tail = (ring_buffer->tail + 1) & RING_BUFFER__MASK;
}

