#ifndef ROBUS_H_INCLUDED
#define ROBUS_H_INCLUDED 1

// Copyright (c) 2011-2013 by IMC.  All rights reserved.

// Structure pointer typedef's come before #include's:
typedef struct Robus__Struct *Robus;

// #inlcudes come next:
#include "types.h"
#include "buffer.h"
#include "serial.h"
#include "uart.h"

// Now it is save to define the actual structure:
struct Robus__Struct {
    Byte address;		// Currently selected address (start at 0xFF)
    Serial bus_serial;		// bus serial port
    Byte commands_length;	// Number of bytes of valid commands
    Serial debug_serial;	// Debugging serial
    Buffer get_buffer;		// Buffer of received bytes
    Buffer put_buffer;		// Buffer of bytes to send
    Uart1 uart1;		// UART for data sending
};

// {Robus} data structures:
extern struct Robus__Struct Robus___null_struct;
extern Robus Robus__null;

// {Robus} routines:
void Robus__request_begin(Robus robus, UByte address, UByte command);
void Robus__request_ubyte_put(Robus robus, UByte ubyte);
void Robus__request_flush(Robus robus);
void Robus__request_end(Robus robus);
void Robus__slave_process(Robus robus,
  UByte (*process_routine)(void *, Robus, UByte, Logical), void *object);

UByte Robus__byte_get(Robus robus);
void Robus__byte_put(Robus robus, UByte byte);

UByte Robus__ubyte_get(Robus robus);
void Robus__ubyte_put(Robus robus, UByte ubyte);
void Robus__initialize(Robus robus,
  Serial debug, Buffer get_buffer, Buffer put_buffer, UByte slave_address);

#endif // ROBUS_H_INCLUDED
