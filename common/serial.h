#ifndef SERIAL_H_INCLUDED
#define SERIAL_H_INCLUDED 1

// Copyright (c) 2011-2013 by IMC.  All rights reserved.

// Structure pointer typedefs come before #include's:
typedef struct Serial__Struct *Serial;

// Next, come #inlcudes"
#include "types.h"
#include "ring_buffer.h"
#include "serial.h"
#include "uart.h"

// Now it is save to define {Serial__Struct}:
struct Serial__Struct {		// {Serial} structure
    Uart uart;			// {Uart} to use
    Bool8 preview_available;	// 1=> we have a preview character
    UInt8 character;		// Preview character
    struct Ring_Buffer__Struct transmit_buffer;	// Actual transmit buffer
    struct Ring_Buffer__Struct receive_buffer;	// Actual receive buffer
    Ring_Buffer transmit;	// Pointer to {transmit_buffer}
    Ring_Buffer receive;	// Pointer to {receive_buffer}
};

// {Serial} data structures:
extern struct Serial__Struct Serial__uart0, Serial__uart1, Serial__uart3;

// {Serial} routines:
UInt8 Serial__character_get(Serial serial);
UInt8 Serial__character_peek(Serial serial);
void Serial__character_preview(Serial serial,  UInt8 character);
void Serial__character_put(Serial serial, UInt8 character);
UInt8 Serial__character_store(Serial serial, UInt8 character);
Bool8 Serial__end_of_line(Serial serial);
Bool8 Serial__error;
void Serial__float_put(Serial serial, Float32 value);
void Serial__float_label(Serial serial, char *label, Float32 value);
void Serial__hex_byte_put(Serial serial, UInt8 byte);
Int32 Serial__hex_get(Serial serial);
void Serial__hex_put(Serial serial, Int32 value);
Serial Serial__initialize(Serial serial, Uart uart, UInt32 baud_rate,
  UInt8 function, UInt8 port, UInt8 tx_bit, UInt8 rx_bit,
  Int8 interrupt_number, UInt8 interrupt_priority, UInt8 slave_address);
Bool8 Serial__input_pending(Serial serial);
void Serial__interrupt(Serial serial);
void Serial__interrupt_receive(Serial serial);
void Serial__interrupt_transmit(Serial serial);
void Serial__label_hex(Serial serial, char *label, Int32 value);
UInt8 Serial__letter_get(Serial serial);
void Serial__nibble_put(Serial serial, UInt8 nibble);
UInt32 Serial__receive(Serial serial, Frame *buffer, UInt32 amount);
void Serial__receive_blocking(Serial serial, Frame *buffer, UInt32 amount);
UInt32 Serial__send(Serial serial, Frame *buffer, UInt32 amount);
void Serial__send_blocking(Serial serial, Frame *buffer, UInt32 amount);
void Serial__string_put(Serial serial, char *string);
void Serial__white_space_skip(Serial serial);

#endif // SERIAL_H_INCLUDED
