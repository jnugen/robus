#ifndef BUFFER_H_INCLUDED
#define BUFFER_H_INCLUDED 1

// Copyright (c) 2011-2013 by IMC.  All rights reserved.

// Typedefs for serial and robus UART's:
typedef struct Buffer__Struct *Buffer;

#include "types.h"

#define BUFFER_SIZE 32
#define BUFFER_MASK (BUFFER_SIZE - 1)

struct Buffer__Struct {
    UByte count;		// Number of bytes in {ubytes}
    UByte get_index;		// Index to get next byte from
    UByte put_index;		// Index to put next byte to
    UByte save_count;		// Total number of bytes for saved location
    UByte save_index;		// Get index for saved location
    UByte ubytes[BUFFER_SIZE];	// Buffer of bytes
};

// {Buffer} data structures and routines:
extern struct Buffer__Struct Buffer__get_buffer_struct;
extern struct Buffer__Struct Buffer__put_buffer_struct;
extern Buffer Buffer__get_buffer;
extern Buffer Buffer__put_buffer;

UByte Buffer__checksum(Buffer buffer, UByte count);
UByte Buffer__remaining(Buffer buffer);
void Buffer__save_restore(Buffer buffer);
void Buffer__save_end_set(Buffer buffer);
void Buffer__save_start_set(Buffer buffer);
void Buffer__reset(Buffer buffer);
void Buffer__ubyte_put(Buffer buffer, UByte ubyte);
UByte Buffer__ubyte_get(Buffer buffer);

#endif // BUFFER_H_INCLUDED
