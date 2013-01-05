// Copyright (c) 2011-2013 by IMC.  All rights reserved.

#ifndef BUFFER_H_INCLUDED
#define BUFFER_H_INCLUDED 1

// Types start with first letter capitalized:

#include "types.h"

// Typedefs for serial and robus UART's:
typedef struct Buffer__Struct *Buffer;

#define BUFFER_SIZE 32
#define BUFFER_MASK (BUFFER_SIZE - 1)

struct Buffer__Struct {
    UByte ubytes[BUFFER_SIZE];
    UByte get_index;
    UByte put_index;
    UByte count;
};

// {Buffer} routines:
extern struct Buffer__Struct Buffer__get_buffer_struct;
extern struct Buffer__Struct Buffer__put_buffer_struct;
extern Buffer Buffer__get_buffer;
extern Buffer Buffer__put_buffer;

void Buffer__ubyte_put(Buffer buffer, UByte ubyte);
UByte Buffer__ubyte_get(Buffer buffer);
UByte Buffer__checksum(Buffer buffer, UByte count);
void Buffer__reset(Buffer buffer);


#endif // BUFFER_H_INCLUDED
