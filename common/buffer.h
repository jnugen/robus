// Copyright (c) 2011-2012 by IMC.  All rights reserved.

#ifndef BUFFER_H_INCLUDED
#define BUFFER_H_INCLUDED 1

#include "types.h"

typedef struct Buffer__Struct *Buffer;

#define BUFFER_SIZE 32
#define BUFFER_MASK (BUFFER_SIZE - 1)

struct Buffer__Struct {
    UByte ubytes[BUFFER_SIZE];
    UByte get_index;
    UByte put_index;
    UByte count;
} Buffer__get_buffer_struct, Buffer__put_buffer_struct;
Buffer Buffer__get_buffer = &Buffer__get_buffer_struct;
Buffer Buffer__put_buffer = &Buffer__put_buffer_struct;

// {Buffer} routines:
void Buffer__ubyte_put(Buffer buffer, UByte ubyte);
UByte Buffer__ubyte_get(Buffer buffer);
UByte Buffer__checksum(Buffer buffer, UByte count);
void Buffer__reset(Buffer buffer);

#endif // BUFFER_H_INCLUDED
