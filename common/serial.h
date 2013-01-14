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
    Logical preview_available;	// 1=> we have a preview character
    UByte character;		// Preview character
    struct Ring_Buffer__Struct transmit_buffer;	// Actual transmit buffer
    struct Ring_Buffer__Struct receive_buffer;	// Actual receive buffer
    Ring_Buffer transmit;	// Pointer to {transmit_buffer}
    Ring_Buffer receive;	// Pointer to {receive_buffer}
};

// {Serial} data structures:
extern struct Serial__Struct Serial__uart0, Serial__uart1, Serial__uart3;

// {Serial} routines:
UByte Serial__character_get(Serial serial);
UByte Serial__character_peek(Serial serial);
void Serial__character_preview(Serial serial,  UByte character);
void Serial__character_put(Serial serial, UByte character);
UByte Serial__character_store(Serial serial, UByte character);
Logical Serial__end_of_line(Serial serial);
Logical Serial__error;
void Serial__float_put(Serial serial, Float value);
void Serial__float_label(Serial serial, char *label, Float value);
void Serial__hex_byte_put(Serial serial, UByte byte);
Integer Serial__hex_get(Serial serial);
void Serial__hex_put(Serial serial, Integer value);
Serial Serial__initialize(Serial serial, Uart uart, UInteger baud_rate,
  UByte function, UByte port, UByte tx_bit, UByte rx_bit,
  Byte interrupt_number, UByte interrupt_priority, UByte slave_address);
Logical Serial__input_pending(Serial serial);
void Serial__interrupt(Serial serial);
void Serial__interrupt_receive(Serial serial);
void Serial__interrupt_transmit(Serial serial);
void Serial__label_hex(Serial serial, char *label, Integer value);
UByte Serial__letter_get(Serial serial);
void Serial__nibble_put(Serial serial, UByte nibble);
UInteger Serial__receive(Serial serial, Frame *buffer, UInteger amount);
void Serial__receive_blocking(Serial serial, Frame *buffer, UInteger amount);
UInteger Serial__send(Serial serial, Frame *buffer, UInteger amount);
void Serial__send_blocking(Serial serial, Frame *buffer, UInteger amount);
void Serial__string_put(Serial serial, char *string);
void Serial__white_space_skip(Serial serial);

#endif // SERIAL_H_INCLUDED
