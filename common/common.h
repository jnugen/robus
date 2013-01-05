// Copyright (c) 2011-2012 by IMC.  All rights reserved.

#ifndef COMMON_H_INCLUDED
#define COMMON_H_INCLUDED 1

#include "lpc17xx_uart.h"
#include "lpc17xx_libcfg.h"
#include "lpc17xx_pinsel.h"

// Note: Including <math.h> triggers an include file error on line 490 of:
//
//  .../codesourcery/arm-2010q1/arm-none-eabi/include/sys/reent.h
//
// #if DEBUG
//
// should be
// 
//   #if 0
//
// This problem was fixed by editing <math.h>.

#include <math.h>

// Types start with first letter capitalized:

// Scalar typedef's with first letter capitalized:
typedef int8_t Byte;
typedef uint8_t Character;
typedef float Float;
typedef int32_t Integer;
typedef uint8_t Logical;
typedef int16_t Short;
typedef uint8_t UByte;
typedef uint32_t UInteger;
typedef uint16_t UShort;

// A frame contains 9-bits of data and potentially some other flags
// from the UART receive FIFE.  We use a signed value so that a negative
// value can indicate "no frame received".
typedef Short Frame;

// Typedefs for serial and robus UART's:
typedef struct Buffer__Struct *Buffer;
typedef struct Serial__Struct *Serial;
typedef enum Serial__Mode Serial_Mode;
typedef struct Ring_Buffer__Struct *Ring_Buffer;
typedef struct Robus__Struct *Robus;
typedef LPC_UART_TypeDef *Uart;
typedef struct Discovery__Struct *Discovery;

#define BUFFER_SIZE 32
#define BUFFER_MASK (BUFFER_SIZE - 1)

struct Buffer__Struct {
    UByte ubytes[BUFFER_SIZE];
    UByte get_index;
    UByte put_index;
    UByte count;
};

#define DISCOVERY_READ_ONLY_BITS (6 * 8 + 1)
#define DISCOVERY_READ_WRITE_BITS (8 - 1)
#define DISCOVERY_TOTAL_BITS \
  (DISCOVERY_READ_ONLY_BITS + DISCOVERY_READ_WRITE_BITS)
#define DISCOVERY_LOW_ADDRESS_START 0
#define DISCOVERY_HIGH_ADDRESS_START 0x80
struct Discovery__Struct {
    UByte low_address;		// Next low address to allocate
    UByte high_address;		// Next high address to allocate
    Logical stack[DISCOVERY_TOTAL_BITS]; // Stack to work with
    Integer top;		// Current valid top bit in stack
    Uart uart_8bit;		// 8-bit UART (up stream)
    Uart uart_9bit;		// 9-bit UART (for bus)
};

// A Ring Buffer:
#define RING_BUFFER__SIZE 32
#define RING_BUFFER__MASK (RING_BUFFER__SIZE - 1)

struct Ring_Buffer__Struct {
    __IO UByte head;			// Ring buffer head index
    __IO UByte tail;			// Ring buffer tail index
    __IO Frame frames[RING_BUFFER__SIZE]; // Ring buffer frame data
};

// The Robot Bus controller:
struct Robus__Struct {
    Byte address;		// Currently selected address (start at 0xFF)
    Serial bus_serial;		// bus serial port
    Byte commands_length;	// Number of bytes of valid commands
    Serial debug_serial;	// Debugging serial
    Buffer get_buffer;		// Buffer of received bytes
    Buffer put_buffer;		// Buffer of bytes to send
    Uart uart;			// UART for data sending
};

// The main serial for character I/O:
struct Serial__Struct {		// {Serial} structure
    Uart uart;			// {Uart} to use
    Logical preview_available;	// 1=> we have a preview character
    UByte character;		// Preview character
    struct Ring_Buffer__Struct transmit_buffer;	// Actual transmit buffer
    struct Ring_Buffer__Struct receive_buffer;	// Actual receive buffer
    Ring_Buffer transmit;	// Pointer to {transmit_buffer}
    Ring_Buffer receive;	// Pointer to {receive_buffer}
};

// Main routines:
Integer main(void);
Integer c_entry(void);
Integer abs(Integer value);

// Command parse routines:
void command_id_show(Serial serial, Robus robus);
void command_id_string_show(Serial serial, Robus robus);
UByte command_id_byte_show(Serial serial, Robus robus);
//void command_parse_navigate(Serial serial, Motor2 motor2, Shaft2 shaft2);

// {Buffer} routines:
extern struct Buffer__Struct Buffer__get_buffer_struct;
extern struct Buffer__Struct Buffer__put_buffer_struct;
extern Buffer Buffer__get_buffer;
extern Buffer Buffer__put_buffer;

void Buffer__ubyte_put(Buffer buffer, UByte ubyte);
UByte Buffer__ubyte_get(Buffer buffer);
UByte Buffer__checksum(Buffer buffer, UByte count);
void Buffer__reset(Buffer buffer);

// {Discovery} routines:
extern struct Discovery__Struct Discovery__one_and_only__struct;
extern Discovery Discovery__one_and_only;

void Discovery__byte_send(Discovery discovery, UByte ubyte);
void Discovery__hex_send(Discovery discovery, UByte uart_8bit);
void Discovery__initialize(Discovery discovery, Uart uart8_bit, Uart uart9_bit);
void Discovery__scan(Discovery discovery);
void Discovery__stack_send(Discovery discovery, UByte *stack);

// {Logical} stuff:
extern const Logical Logical__true;
extern const Logical Logical__false;

// {Serial} data structures and routines:
extern struct Serial__Struct Serial__uart0, Serial__uart1, Serial__uart3;

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
  Byte interrupt_number, UByte interrupt_priority);
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

// {Ring_Buffer} routines:
Ring_Buffer Ring_Buffer__initialize(Ring_Buffer ring_buffer);
UByte Ring_Buffer__is_full(Ring_Buffer ring_buffer);
UByte Ring_Buffer__is_empty(Ring_Buffer ring_buffer);
void Ring_Buffer__head_increment(Ring_Buffer ring_buffer);
void Ring_Buffer__tail_increment(Ring_Buffer ring_buffer);

// {Robus} routines:
extern struct Robus__Struct Robus___null_struct;
extern Robus Robus__null;

void Robus__request_begin(Robus robus, UByte address, UByte command);
void Robus__request_ubyte_put(Robus robus, UByte ubyte);
void Robus__request_flush(Robus robus);
void Robus__request_end(Robus robus);

UByte Robus__byte_get(Robus robus);
void Robus__byte_put(Robus robus, UByte byte);

UByte Robus__ubyte_get(Robus robus);
void Robus__ubyte_put(Robus robus, UByte ubyte);
void Robus__initialize(Robus robus,
  Serial debug, Buffer get_buffer, Buffer put_buffer);

// {Trace} data structures and routines:

#define TRACE_BITS 7
#define TRACE_SIZE (1 << TRACE_BITS)
#define TRACE_MASK (TRACE_SIZE - 1)
extern UShort trace_buffer[];
extern UShort trace_index;
#define TRACE(byte) trace_buffer[trace_index++ & TRACE_MASK] = byte

void trace_dump(Serial serial);

// SysTick routines:
volatile unsigned long SysTickCnt;
void SysTick__delay(unsigned long tick);
void SysTick_Handler(void);

// {Uart} routines:
void Uart__frame_put(Uart uart, UShort frame);
UShort Uart__frame_get(Uart uart);

// Devfine all four UART interrupt handlers regardless of whether
// or not they actually get implemented:
void UART0_IRQHandler(void);
void UART1_IRQHandler(void);
void UART2_IRQHandler(void);
void UART3_IRQHandler(void);

void uart_interrupt_error(Uart uart, UInteger line_flags);
__IO FlagStatus uart_transmit_interrupt_status;

#endif // COMMON_H_INCLUDED
