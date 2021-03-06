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
    Int8 address;		// Currently selected address (start at 0xFF)
    Serial bus_serial;		// bus serial port
    Int8 commands_length;	// Number of bytes of valid commands
    Serial debug_serial;	// Debugging serial
    Buffer get_buffer;		// Buffer of received bytes
    Buffer put_buffer;		// Buffer of bytes to send
    struct LPC_UART1__Struct *uart1; // UART for data sending
};

// {Robus} data structures:
extern struct Robus__Struct Robus___null_struct;
extern Robus Robus__null;

// {Robus} routines:
void Robus__request_begin(Robus robus, UInt8 address, UInt8 command);
void Robus__request_ubyte_put(Robus robus, UInt8 ubyte);
void Robus__request_flush(Robus robus);
void Robus__request_end(Robus robus);
void Robus__slave_process(Robus robus,
  UInt8 (*process_routine)(void *, Robus, UInt8, Bool8), void *object);

UInt8 Robus__byte_get(Robus robus);
void Robus__byte_put(Robus robus, UInt8 byte);

UInt8 Robus__ubyte_get(Robus robus);
void Robus__ubyte_put(Robus robus, UInt8 ubyte);
void Robus__initialize(Robus robus,
  Serial debug, Buffer get_buffer, Buffer put_buffer, UInt8 slave_address);

#endif // ROBUS_H_INCLUDED
