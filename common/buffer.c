// Copyright (c) 2011-2013 by IMC.  All rights reserved.

#include "buffer.h"
#include "serial.h"
#include "types.h"

// {Buffer} data structures and routines:
struct Buffer__Struct Buffer__get_buffer_struct;
struct Buffer__Struct Buffer__put_buffer_struct;
Buffer Buffer__get_buffer = &Buffer__get_buffer_struct;
Buffer Buffer__put_buffer = &Buffer__put_buffer_struct;

UInt8 Buffer__checksum(
  Buffer buffer,
  UInt8 count)
{
    // This routine will return the 4-bit checksum of the first {count}
    // bytes in {buffer}.

    UInt8 checksum = 0;
    UInt8 index = buffer->get_index;
    while (count != 0) {
	checksum += buffer->ubytes[index++ & BUFFER_MASK];
	count--;
    }
    checksum = (checksum + (checksum >> 4)) & 0xf;
    return checksum;
}


void Buffer__reset(
  Buffer buffer)
{
    // This routine will reset {buffer} to be empty:

    buffer->count = 0;
    buffer->get_index = 0;
    buffer->put_index = 0;
    buffer->save_count = 0;
    buffer->save_index = 0;
}

UInt8 Buffer__remaining(
  Buffer buffer)
{
    // This routine will return the number of remaining bytes in {buffer}.

    return buffer->count;
}

void Buffer__save_restore(
  Buffer buffer)
{
    // This routine will restore {buffer} to the saved "get location".

    buffer->count = buffer->save_count;
    buffer->get_index = buffer->save_index;
}

void Buffer__save_end_set(
  Buffer buffer)
{
    // This routine will save the current "get location" for {buffer}.

    buffer->save_count = buffer->count;
}

void Buffer__save_start_set(
  Buffer buffer)
{
    // This routine will save the current "get location" for {buffer}.

    buffer->save_index = buffer->get_index;
}

UInt8 Buffer__ubyte_get(
  Buffer buffer)
{
    // This routine will return the next byte from {buffer}:

    buffer->count--;
    return buffer->ubytes[buffer->get_index++ & BUFFER_MASK];
}

void Buffer__ubyte_put(
  Buffer buffer,
  UInt8 ubyte)
{
    // This routine will enter {byte} into buffer:

    buffer->count++;
    buffer->ubytes[buffer->put_index++ & BUFFER_MASK] = ubyte;
}

Int32 Buffer__int32_get(
  Buffer buffer)
{
    // This routine will return the next byte from {buffer}:

    return 0x01234567;
}

void Buffer__int32_put(
  Buffer buffer,
  Int32 int32)
{
    // This routine will enter {byte} into buffer:

    //buffer->count++;
    //buffer->ubytes[buffer->put_index++ & BUFFER_MASK] = ubyte;
}

