#ifndef BUFFER_H_INCLUDED
#define BUFFER_H_INCLUDED 1

// Copyright (c) 2011-2013 by IMC.  All rights reserved.

// Typedefs for serial and robus UART's:
typedef struct Buffer__Struct *Buffer;

#include "types.h"

#define BUFFER_SIZE 32
#define BUFFER_MASK (BUFFER_SIZE - 1)

struct Buffer__Struct {
    UInt8 count;		// Number of bytes in {ubytes}
    UInt8 get_index;		// Index to get next byte from
    UInt8 put_index;		// Index to put next byte to
    UInt8 save_count;		// Total number of bytes for saved location
    UInt8 save_index;		// Get index for saved location
    UInt8 ubytes[BUFFER_SIZE];	// Buffer of bytes
};

// {Buffer} data structures and routines:
extern struct Buffer__Struct Buffer__get_buffer_struct;
extern struct Buffer__Struct Buffer__put_buffer_struct;
extern Buffer Buffer__get_buffer;
extern Buffer Buffer__put_buffer;

UInt8 Buffer__checksum(Buffer buffer, UInt8 count);
UInt8 Buffer__remaining(Buffer buffer);
void Buffer__save_restore(Buffer buffer);
void Buffer__save_end_set(Buffer buffer);
void Buffer__save_start_set(Buffer buffer);
void Buffer__reset(Buffer buffer);

void Buffer__int8_put(Buffer buffer, Int8 int8);
Int8 Buffer__int8_get(Buffer buffer);
void Buffer__int32_put(Buffer buffer, Int32 int32);
Int32 Buffer__int32_get(Buffer buffer);

void Buffer__uint8_put(Buffer buffer, UInt8 uint8);
UInt8 Buffer__uint8_get(Buffer buffer);
void Buffer__uint32_put(Buffer buffer, UInt32 uint32);
UInt32 Buffer__uint32_get(Buffer buffer);

#endif // BUFFER_H_INCLUDED
