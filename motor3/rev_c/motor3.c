// Copyright (c) 2011-2012 by IMC.  All rights reserved.

#include "lpc17xx_uart.h"
#include "lpc17xx_libcfg.h"
#include "lpc17xx_pinsel.h"
#include "motor.h"
#include "pwm.h"

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
typedef struct Motor3__Struct *Motor3;
typedef struct Ring_Buffer__Struct *Ring_Buffer;
typedef struct Robus__Struct *Robus;
typedef enum Serial__Mode Serial_Mode;
typedef struct Serial__Struct *Serial;
typedef LPC_UART_TypeDef *Uart;
typedef LPC_UART1_TypeDef *Uart1;

#define BUFFER_SIZE 32
#define BUFFER_MASK (BUFFER_SIZE - 1)

struct Buffer__Struct {
    UByte count;		// Number of bytes in {ubytes}
    UByte get_index;		// Index to get next byte from
    UByte put_index;		// Index to put next byte to
    UByte save_count;		// Total number of bytes for saved location
    UByte save_index;		// Get index for saved location
    UByte ubytes[BUFFER_SIZE];	// Buffer of bytes
} Buffer__get_buffer_struct, Buffer__put_buffer_struct;
Buffer Buffer__get_buffer = &Buffer__get_buffer_struct;
Buffer Buffer__put_buffer = &Buffer__put_buffer_struct;

#define TRACE_BITS 7
#define TRACE_SIZE (1 << TRACE_BITS)
#define TRACE_MASK (TRACE_SIZE - 1)
UShort trace_buffer[TRACE_SIZE];
UShort trace_index = 0;
#define TRACE(byte) trace_buffer[trace_index++ & TRACE_MASK] = byte

struct Motor3__Struct {
    Byte speed;			// Signed motor speed
    Logical direction_invert;	// Invert motor speed sign
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
    Uart1 uart1;		// UART for data sending
} Robus___null_struct;
Robus Robus__null = &Robus___null_struct;

// The main serial for character I/O:
struct Serial__Struct {		// {Serial} structure
    Uart uart;			// {Uart} to use
    Logical preview_available;	// 1=> we have a preview character
    UByte character;		// Preview character
    struct Ring_Buffer__Struct transmit_buffer;	// Actual transmit buffer
    struct Ring_Buffer__Struct receive_buffer;	// Actual receive buffer
    Ring_Buffer transmit;	// Pointer to {transmit_buffer}
    Ring_Buffer receive;	// Pointer to {receive_buffer}
} Serial__uart0, Serial__uart1, Serial__uart3;

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
UByte Buffer__checksum(Buffer buffer, UByte count);
UByte Buffer__remaining(Buffer buffer);
void Buffer__save_restore(Buffer buffer);
void Buffer__save_end_set(Buffer buffer);
void Buffer__save_start_set(Buffer buffer);
void Buffer__reset(Buffer buffer);
void Buffer__ubyte_put(Buffer buffer, UByte ubyte);
UByte Buffer__ubyte_get(Buffer buffer);

// {Logical} stuff:
const Logical Logical__true = (Logical)1;
const Logical Logical__false = (Logical)0;

// {Motor3} routines:
void Motor3__speed_update(Motor3 motor3, Serial debug_serial);
UByte Motor3__process(void *motor3__pointer,
   Robus robus, UByte command, Logical execute);

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

// {Ring_Buffer} routines:
Ring_Buffer Ring_Buffer__initialize(Ring_Buffer ring_buffer);
UByte Ring_Buffer__is_full(Ring_Buffer ring_buffer);
UByte Ring_Buffer__is_empty(Ring_Buffer ring_buffer);
void Ring_Buffer__head_increment(Ring_Buffer ring_buffer);
void Ring_Buffer__tail_increment(Ring_Buffer ring_buffer);

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

// trace routines:
void trace_dump(Serial serial);

// SysTick routines:
volatile unsigned long SysTickCnt;
void SysTick_delay(unsigned long tick);
void SysTick_Handler(void);

// {Uart} routines:
void Uart__frame_put(Uart1 uart1, UShort frame);
UShort Uart__frame_get(Uart1 uart1);

// Devfine all four UART interrupt handlers regardless of whether
// or not they actually get implemented:
void UART0_IRQHandler(void);
void UART1_IRQHandler(void);
void UART2_IRQHandler(void);
void UART3_IRQHandler(void);

void uart_interrupt_error(Uart uart, UInteger line_flags);
__IO FlagStatus uart_transmit_interrupt_status;

// Routine definitions from here on:

// Most compilers programs start from main...:

Integer main(void)
{
    return c_entry();
}

// ... but for some reason, we use c_entry instead:
Integer c_entry(void)
{
    // Generate interrupt each 1 ms:
    SysTick_Config(SystemCoreClock/1000 - 1);

    // Initialize the "console" UART to 115200 bps, using alternate function 1
    // on P0.2(TXD0) or P0.3(RXD0):
    Byte interrupt_number = UART0_IRQn;
    interrupt_number = (Byte)-1;	// Disable interrupts
    Serial console = Serial__initialize(&Serial__uart0,
      (Uart)LPC_UART0, 115200, 1, 0, 2, 3, interrupt_number, 0x01, 0);

    // Make sure that _UART3 is defined in libcfg.h!:
    //Serial debug = Serial__initialize(&Serial__uart3,
    //  (Uart)LPC_UART3, 115200, 2, 0, 0, 1, UART3_IRQn, 0x04);
    //Serial__string_put(debug, "debug:\n");

    UByte slave_address = 133;
    Robus robus = Robus__null;
    Robus__initialize(robus,
      console, Buffer__get_buffer, Buffer__put_buffer, slave_address);
    Uart1 uart1 = robus->uart1;

    uart1->LCR |= UART_LCR_PARITY_EN;
    Serial__string_put(console, "Motor3:");
    Serial__hex_put(console, uart1->LCR);
    Serial__character_put(console, ';');
    Serial__hex_put(console, uart1->RS485CTRL);
    Serial__character_put(console, '\n');

    struct Motor3__Struct motor3__struct;
    Motor3 motor3 = &motor3__struct;
    motor3->speed = 0;
    motor3->direction_invert = (Logical)0;

    motor_init();
    pwm_init();

    Robus__slave_process(robus, Motor3__process, (void *)motor3);

    // We never get here:
    return 1;
}

// {Buffer} routines:

UByte Buffer__checksum(
  Buffer buffer,
  UByte count)
{
    // This routine will return the 4-bit checksum of the first {count}
    // bytes in {buffer}.

    UByte checksum = 0;
    UByte index = buffer->get_index;
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

UByte Buffer__remaining(
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

UByte Buffer__ubyte_get(
  Buffer buffer)
{
    // This routine will return the next byte from {buffer}:

    buffer->count--;
    return buffer->ubytes[buffer->get_index++ & BUFFER_MASK];
}

void Buffer__ubyte_put(
  Buffer buffer,
  UByte ubyte)
{
    // This routine will enter {byte} into buffer:

    buffer->count++;
    buffer->ubytes[buffer->put_index++ & BUFFER_MASK] = ubyte;
}

// {Motor3} routines:

void Motor3__speed_update(
  Motor3 motor3,
  Serial debug_serial)
{
    Byte speed = motor3->speed;
    Logical direction_invert = motor3->direction_invert;

    Integer pwm = 0;
    if (debug_serial != (Serial)0) {
	Serial__character_put(debug_serial, ' ');
	Serial__character_put(debug_serial, 'S');
	Serial__hex_byte_put(debug_serial, speed);
	Serial__character_put(debug_serial, 'D');
	Serial__hex_put(debug_serial, direction_invert);
    }

    if (direction_invert) {
	speed = -speed;
    }

    if (speed > 0) {
	motor_forward();
	pwm = (Integer)(speed << 1);
    } else if (speed < 0) {
	motor_reverse();
        pwm = (Integer)((-speed) << 1);
    } else {
	motor_stop();
    }

    if (debug_serial != (Serial)0) {
	Serial__character_put(debug_serial, 'P');
	Serial__hex_put(debug_serial, pwm);
    }

    pwm_update(pwm);
}

UByte Motor3__process(
  void *motor3_pointer,
  Robus robus,
  UByte command,
  Logical execute)
{
    UByte errors = 0;
    Motor3 motor3 = (Motor3)motor3_pointer;
    Buffer get_buffer = robus->get_buffer;
    Buffer put_buffer = robus->put_buffer;
    UByte remaining = Buffer__remaining(get_buffer);
    Serial debug_serial = robus->debug_serial;
    debug_serial = (Serial)0;

    switch (command) {
      case 0:
	// Speed get:
	if (execute) {
	    Buffer__ubyte_put(put_buffer, (UByte)motor3->speed);
	}
	break;
      case 1:
	// Speed set:
	if (remaining == 0) {
	    errors = 1;
	} else {
	    Byte speed = (Byte)Buffer__ubyte_get(get_buffer);
	    if (execute) {
		motor3->speed = speed;
		Motor3__speed_update(motor3, debug_serial);
	    }
	}
	break;
      case 2:
	// Direction Invert get:
	if (execute) {
	    Buffer__ubyte_put(put_buffer, (UByte)motor3->direction_invert);
	}
	break;
      case 3:
	// Direction Invert set:
	if (remaining == 0) {
	    errors = 1;
	} else {
	    Logical direction_invert =
	      (Logical)(Buffer__ubyte_get(get_buffer) != 0);
	    if (execute) {
		motor3->direction_invert = direction_invert;
		Motor3__speed_update(motor3, debug_serial);
	    }
	}
	break;
      default:
	errors = 1;
	break;
    }

    //Serial__character_put(debug_serial, 'T');
    //Serial__hex_put(debug_serial, motor3->speed);
    //Serial__character_put(debug_serial, 'V');
    //Serial__hex_put(debug_serial, motor3->direction_invert);

    return errors;
}

// {Ring_Buffer} routines:

Ring_Buffer Ring_Buffer__initialize(
  Ring_Buffer ring_buffer)
{
    // This routine will initialize {ring_buffer} to be empty.
    // The value passed in as {ring_buffer} is returned.

    ring_buffer->head = 0;
    ring_buffer->tail = 0;
    return ring_buffer;
}

Logical Ring_Buffer__is_empty(Ring_Buffer ring_buffer)
{
    UByte result;

    result = 0;
    if ((ring_buffer->head & RING_BUFFER__MASK) ==
      (ring_buffer->tail & RING_BUFFER__MASK)) {
	result = 1;
    }
    return result;
}

void Ring_Buffer__head_increment(
  Ring_Buffer ring_buffer)
{
    ring_buffer->head = (ring_buffer->head + 1) & RING_BUFFER__MASK;
}

// This routine will return 1 if {ring_buffer} is full, and 0 otherise.
UByte Ring_Buffer__is_full(
  Ring_Buffer ring_buffer)
{
    UByte result;

    result = 0;
    if ((ring_buffer->tail & RING_BUFFER__MASK) ==
      ((ring_buffer->head + 1) & RING_BUFFER__MASK)) {
	result = 1;
    }
    return result;
}

void Ring_Buffer__tail_increment(
  Ring_Buffer ring_buffer)
{
    ring_buffer->tail = (ring_buffer->tail + 1) & RING_BUFFER__MASK;
}

// {Robus} routines:


void Robus__byte_put(
  Robus robus,
  UByte byte)
{
    // This routine will send {byte} to the Robus bus attached to {robus}.
    // The echo byte from sending {byte} is read back.
    UByte buffer[1];

    // Send {byte} to {robus}:
    buffer[0] = byte;
    (void)UART_RS485SendData(robus->uart1, buffer, 1);

    TRACE(byte | 0x200);

    // Read the echo back from {robus}:
    if (Robus__byte_get(robus) != byte) {
	//robus->errors++;
    }
}

UByte Robus__byte_get(
  Robus robus)
{
    // This routine will get one byte of response Robus bus attached to {robus}.
    // If no byte is found after a resonable amount of time, 0x5a is returned
    // instead.

    UByte byte;
    UByte buffer[1];
    UByte count;
    UInteger tries;

    // Try for a while to get the byte:
    byte = 0x5a;
    for (tries = 0; tries < 20; tries++) {
	// See if anything has come in yet:
	count = UART_Receive((Uart)robus->uart1, buffer, 1, NONE_BLOCKING);
	//count = UARTReceive(robus->uart, buffer, 1);
	if (count == 1) {
	    // We got a byte; return it:
	    byte = buffer[0];
	    TRACE(byte | 0x1000);
	    return byte;
	}

	// FIXME!!! Is this necessary???
	// Wait for a millisecond:
	SysTick_delay(1);
    }

    // We have timed out; return 0x5a:
    //robus->errors++;
    TRACE(0x800 | 0x5a);

    return 0x5a;
}

void Robus__request_begin(
  Robus robus,
  UByte address,
  UByte command)
{
    // This routine will start a the output of a command to the {robus}
    // module at {address}.  The first byte of the command is {command}.

    // If we are changing addresses.  Flush any previous commands:
    if (address != robus->address) {
        Robus__request_flush(robus);
	robus->address = address;
    }

    // Send out the new {address}:
    Uart1 uart1 = robus->uart1;
    Uart__frame_put(uart1, (Frame)address | (Frame)0x100);

    // If 8th bit of address is 0, an acknowledge byte is requred:
    if ((address & 0x80) == 0) {
	// Deal with an acknowledge byte:
	(void)Uart__frame_put(uart1, (Frame)0xa5);
    }

    // Remember how many bytes are ready to be sent:
    Buffer put_buffer = robus->put_buffer;
    robus->commands_length = put_buffer->count;

    // Stuff {command} into {put_buffer}:
    Robus__request_ubyte_put(robus, command);
}

void Robus__request_end(
  Robus robus)
{
    Robus__request_flush(robus);
}

void Robus__request_flush(
  Robus robus)
{
    // This routine will flush out the commands stored {robus} and get
    // response data that is needed.

    // Grab some values from {robus}:
    UByte commands_length = robus->commands_length;
    Buffer put_buffer = robus->put_buffer;
    Buffer get_buffer = robus->get_buffer;
    Uart1 uart1 = robus->uart1;
    //Serial debug = robus->serial;

    //Serial__string_put(debug, "Flush[");

    // Send as many request/response pairs as needed to clear {put_buffer}:
    UByte put_buffer_count = put_buffer->count;
    while (put_buffer_count != 0) {
	// Can we can clear out the entire buffer this pass?:
	if (put_buffer_count <= 15) {
	    // Yes, we can:
	    commands_length = put_buffer_count;
	}

	// Output the request packet header:
	UByte checksum = Buffer__checksum(put_buffer, commands_length);
	UByte request_header = (commands_length << 4) | checksum;
	Uart__frame_put(uart1, (UShort)request_header);

	// Output the request packet data:
	UByte index;
	for (index = 0; index < commands_length; index++) {
	    UByte ubyte = Buffer__ubyte_get(put_buffer);
	    Uart__frame_put(uart1, (UShort)ubyte);
	}
	put_buffer_count -= commands_length;

	// Wait for the response packet header:
	UByte response_header = (Byte)Uart__frame_get(uart1);
	UByte response_length = response_header >> 4;

	// Read in the response packet data:
	checksum = 0;
	for (index = 0; index < response_length; index++) {
	    UByte ubyte = (Byte)Uart__frame_get(uart1);
	    checksum += ubyte;
	    Buffer__ubyte_put(get_buffer, ubyte);
	}
    }
    robus->commands_length = 0;

    //Serial__string_put(debug, "]\n");
}

void Robus__request_ubyte_put(
  Robus robus,
  UByte ubyte)
{
    // This routine will enter {ubyte} into the command put buffer for {robus}.
    // The byte should be the continuation of a multi-byte command.

    // Stuff {byte} into {put_buffer}:
    Buffer put_buffer = robus->put_buffer;
    Buffer__ubyte_put(put_buffer, ubyte);

    // If the buffer is already over 15 bytes, flush any pending commands:
    if (put_buffer->count > 15) {
	Robus__request_flush(robus);
    }
}

//UByte Robus__command_byte_get(
//  Robus robus,
//  UByte command)
//{
//    // This routine will send {command} to the current selected Robus module
//    // attached to {robus} and return the single byte response.
//
//    UByte tries;
//    Byte result;
//
//    //UByte address = robus->address;
//    for (tries = 0; tries < 5; tries++) {
//	// Send {command}:
//	Robus__byte_put(robus, command);
//
//	// Get {byte}:
//	result = Robus__byte_get(robus);
//
//	// If there are no errors, we are done:
//	//if (robus->errors == 0) {
//	//    break;
//	//}
//
//	// We had an error, forget the module address to force reselect:
//	robus->address = 0xff;
//    }
//    return result;
//}
//
//void Robus__command_byte_put(
//  Robus robus,
//  UByte command,
//  Byte byte)
//{
//    // This routine will send {command} to the currently selected Robus module
//    // attached to {robus} followed by {byte}.
//
//    // Send {command}:
//    Robus__byte_put(robus, command);
//
//    // Send {byte}:
//    Robus__byte_put(robus, byte);
//}

void Robus__initialize(
  Robus robus,
  Serial debug_serial,
  Buffer get_buffer,
  Buffer put_buffer,
  UByte slave_address)
{
    // This routine will initialize {robus} data structure and stuff
    // {serial} into it.  The UART connect to the {robus}.

    // Robus is connected to UART1:
    Uart1 uart1 = LPC_UART1;
    Uart uart = (Uart)uart1;
    robus->address = 0xff;
    robus->debug_serial = debug_serial;
    robus->uart1 = uart1;
    robus->get_buffer = get_buffer;
    robus->put_buffer = put_buffer;
    robus->address = slave_address;

    Buffer__reset(get_buffer);
    Buffer__reset(put_buffer);

    if (1) {
	robus->bus_serial = Serial__initialize(&Serial__uart1,
	  uart, 500000, 2, 2, 0, 1, UART1_IRQn, 0x1, slave_address);
    } else {
	//// Enable UART Rx interrupt
	//UART_IntConfig((LPC_UART_TypeDef *)LPC_UART1, UART_INTCFG_RBR, ENABLE);
	//// Enable UART line status interrupt
	//UART_IntConfig((LPC_UART_TypeDef *)LPC_UART1, UART_INTCFG_RLS, ENABLE);
	//
	//// preemption = 1, sub-priority = 1
	//NVIC_SetPriority(UART1_IRQn, ((0x01<<3)|0x01));
	//// Enable Interrupt for UART0 channel
	//NVIC_EnableIRQ(UART1_IRQn);
	//
	//__BUF_RESET(rb.rx_head);
	//__BUF_RESET(rb.tx_head);
	//__BUF_RESET(rb.rx_tail);
	//__BUF_RESET(rb.tx_tail);
    }

    // Enable UART1 Transmit:
    UART_TxCmd(uart, ENABLE);

    // Enble UART1 to Receive:
    UART_RS485ReceiverCmd(uart1, ENABLE);

    // Flush any garbage that came in:
    //Robus__quiesce(robus);
}

void Robus__slave_process(
  Robus robus,
  UByte (*process_routine)(void *, Robus, UByte, Logical),
  void *object)
{
    //UByte transmit_buffer[16];

    UByte address = robus->address;
    Logical address_match = (Logical)0;
    Serial debug_serial = robus->debug_serial;
    UByte echo_suppress = 0;
    Buffer get_buffer = robus->get_buffer;
    Buffer put_buffer = robus->put_buffer;
    const Logical trace = (Logical)0;
    Uart1 uart1 = robus->uart1;
    Integer receive_length = -1;
    UByte desired_checksum = 0;

    while (1) {
	// Check that there is a data available in {uart1}.  Stash the LSR
	// (Line Status Register) into a local variable because reading
	// the LSR clears all of the error bits:
	UInteger lsr = uart1->LSR;
	if ((lsr & UART_LSR_RDR) != 0) {
	    Frame frame = (Frame)0;

	    // A parity error indicates that we have the 9th address bit set:
	    if ((lsr & UART_LSR_PE) != 0) {
	        // We have an address frame:
		frame |= (Frame)0x100;

		// Any residual bytes in the receive buffer get flused out
		// when we get an address frame:
		Buffer__reset(get_buffer);
		receive_length = -1;
	    }

	    // Grab the remaining 8 bits of data:
	    frame |= (Frame)(uart1->RBR & UART_RBR_MASKBIT);

	    // For debugging, output the received character:
	    if (trace) {
		// With a new frame; start on a new line:
		Character prefix = ' ';
	        if ((frame & 0x100) != 0) {
		    prefix = '\n';
	        }

		// Output {frame} in hex:
	        Serial__character_put(debug_serial, prefix);
		Serial__string_put(debug_serial, " G");
		Serial__hex_put(debug_serial, frame);
	    }

	    // Process the received frame:
	    if ((frame & 0x100) != 0) {
	        // Deal with address.  In address match mode, the address
	        // will always match.
	        address_match =
		  (Logical)((UByte)(frame & 0xff) == address);
		Buffer__reset(get_buffer);
		echo_suppress = 0;

		if (trace) {
		    Serial__character_put(debug_serial,
		      address_match ? 'M' : 'U');
		    Serial__hex_put(debug_serial, address);
		}
	    } else if (address_match) {
		// Deal with frame:
		if (echo_suppress != 0) {
		    // Suppress echo of transmitted byte:
		    echo_suppress -= 1;
		    if (trace) {
			Serial__character_put(debug_serial, '@');
		    }
		} else {
		    // We have a byte to put into the receive buffer:
		    UByte receive_byte = (UByte)(frame & 0xff);
		    if (receive_length < 0) {
			// Have a byte that contains length and checksum:
			receive_length = receive_byte >> 4;
			desired_checksum = receive_byte & 0xf;
			Buffer__save_start_set(get_buffer);

			if (trace) {
			    Serial__character_put(debug_serial, 'R');
			    Serial__hex_put(debug_serial, receive_byte);
			}
		    } else  {
		        // Have a byte that that needs to be saved:
			Buffer__ubyte_put(get_buffer, receive_byte);

			if (trace) {
			    Serial__character_put(debug_serial, 'X');
			    Serial__hex_put(debug_serial,
			      Buffer__remaining(get_buffer));
			}

			// Do we have a complete message?:
			if (Buffer__remaining(get_buffer) >= receive_length) {
			    // We have a complete message:
			    Buffer__save_end_set(get_buffer);

			    UByte actual_checksum =
			      Buffer__checksum(get_buffer, receive_length);
			    if (desired_checksum != actual_checksum) {
				// Checksum error; respond with error byte:
				Uart__frame_put(uart1, 0x1);
				if (trace) {
				    Serial__character_put(debug_serial, '!');
				}
			    } else {
				// The checksums match, process the commands:
				UByte errors = 0;
				UByte pass;
				Buffer__reset(put_buffer);

				// Pass through the buffer twice.  The first
				// pass verifies that all the commands are
				// correct; The second pass actuall executes
				// the commands:
				for (pass = 0; pass < 2; pass++) {
				    if (trace) {
					Serial__character_put(debug_serial,
					  '^');
					Serial__hex_put(debug_serial,
					  Buffer__remaining(get_buffer));
				    }

				    Buffer__save_restore(get_buffer);
				    Logical execute = (Logical)(pass == 1);

				    // Keep processing commands until done:
				    while (Buffer__remaining(get_buffer) != 0) {
					UByte command =
					   Buffer__ubyte_get(get_buffer);
					errors += process_routine(object,
					  robus, command, execute);
					if (trace) {
					    Serial__character_put(debug_serial,
					     'C');
					    Serial__hex_put(debug_serial,
					      command);
					}
				    }

				    // Clear out {get_buffer}:
				    if (errors != 0) {
				         break;
				    }
				}

				// Send back a response:
				if (errors == 0) {
				    // No errors, send the result back:

				    // Compute and send header byte:
				    UByte send_size =
				      Buffer__remaining(put_buffer);
				    UByte send_checksum =
				      Buffer__checksum(put_buffer, send_size);
				    UByte header_byte =
				      (send_size << 4) | send_checksum;
				    Uart__frame_put(uart1, header_byte);
				    echo_suppress = 1;

				    if (trace) {
					Serial__character_put(debug_serial,
					  'H');
					Serial__hex_put(debug_serial,
					  header_byte);
				    }

				    // Send rest of results:
				    echo_suppress += send_size;
				    UByte index;
				    for (index = 0;
				      index < send_size; index++) {
					UByte ubyte = 
					  Buffer__ubyte_get(put_buffer);
				        Uart__frame_put(uart1, ubyte);

					if (trace) {
					    Serial__character_put(debug_serial,
					     'I');
					    Serial__hex_put(debug_serial,
					      index);
					    Serial__character_put(debug_serial,
					      'P');
					    Serial__hex_put(debug_serial,
					      ubyte);
					}
				    }
				} else {
				    // There is an error; send an error result:
				    echo_suppress = 1;
				    Uart__frame_put(uart1, 0x3);

				    if (trace) {
					Serial__character_put(debug_serial,
					  '$');
				    }
				}

				// No matter what, reset both buffers:
				Buffer__reset(get_buffer);
				Buffer__reset(put_buffer);

				// Mark that for next message block:
				receive_length = -1;
			    }  // checksums match
			} // We have a complete message:
		    }
		} // Deal with frame
	    } // else data byte must be for a different module:
	}
    }
}

UByte Robus__ubyte_get(
  Robus robus)
{
    // This routine will get the next response byte from {robus}.

    Robus__request_flush(robus);
    return Buffer__ubyte_get(robus->get_buffer);
}

// {Serial} routines:

// This routine will return the next charcter from {serial}.
UByte Serial__character_get(
  Serial serial)
{
    UByte character;

    character = Serial__character_peek(serial);
    serial->preview_available = 0;
    return character;
}

// The routine will return the next character from {serial}
// without actually "reading" it.
UByte Serial__character_peek(
  Serial serial)
{
    Frame rx_buffer[1];
    UByte character;
    UInteger amount;

    // Read the next character if we have not already done so:
    character = serial->character;
    if (!serial->preview_available) {
        // Get one character:
	while (1) {
	    amount = Serial__receive(serial, rx_buffer, 1);
	    if (amount >= 1) {
	        break;
	    }
	}
	character = (UByte)rx_buffer[0];
	character = Serial__character_store(serial, character);
    }

    // Return the peeked character:
    return character;
}

// This routine will return 1 if there is any character input pending
// for {serial}.
UByte Serial__input_pending(
  Serial serial)
{
    UByte rx_buffer[1];
    UByte rx_count;
    UByte character;
    
    if (!serial->preview_available) {
	// No preview yet:
	rx_count = UART_Receive(serial->uart, rx_buffer, 1, NONE_BLOCKING);
	if (rx_count != 0) {
	    // We got a character, shift it:
	    character = rx_buffer[0];

	    // Does the character count?:
	    if (character != 0xff) {
		// Yes it does:
		character = Serial__character_store(serial, character);
	    }
	}
    }
    return serial->preview_available;
}

// Serial__character_put(uart, character) will output {character} to {uart}.
//  A new-line character is output as CRLF.  Only printing charcters are
//  actually output.
void Serial__character_put(
  Serial serial,
  UByte character)
{
    Frame tx_buffer[2];

    // Make sure that 8th bit is clear:
    character &= 0x7f;

    // Only print out "visible" characters:
    if ((' ' <= character) && (character <= '~')) {
	// {character} is a printing ASCII character:
	tx_buffer[0] = character;
 	Serial__send_blocking(serial, tx_buffer, 1);
    } else if (character == '\n') {
	// {character} is a new-line; echo as CRLF:
	tx_buffer[0] = '\r';
 	Serial__send_blocking(serial, tx_buffer, 1);
	tx_buffer[0] = '\n';
 	Serial__send_blocking(serial, tx_buffer, 1);
    } else if (character == '\t') {
	// {character} is a tab:
	tx_buffer[0] = character;
 	Serial__send_blocking(serial, tx_buffer, 1);
    } else {
	// {character} should not be printed:
    }

}

// This routine is a helper routine that stores {character} into {serial}
// performing any needed transformations, character echoing, etc.  The
// stored character is also returned.
UByte Serial__character_store(
  Serial serial,
  UByte character)
{
    // Convert carriage-return character into a new-line character:
    if (character == '\r') {
	character = '\n';
    }
    serial->character = character;
    
    // Echo the character:
    Serial__character_put(serial, character);

    // Mark that we have a character available:
    serial->preview_available = 1;

    return character;
}

// This routine will skip over any white space and read the ending
// new-line character from {serial}.  If there are any characters other
// than white space, {Serial__error} is set to 1.  If {Serial__error} is set 1
// either by this routine or a previous call, an additional "?\n" is output
// to indicate that an error occured and 0 is returned.
UByte Serial__end_of_line(
  Serial serial)
{
    UByte character;
    UByte result;

    // Read characters until we get a new-line:
    result = 1;
    while (1) {
	character = Serial__character_get(serial);
	if (character == '\n') {
	    // We have the new-line:
	    break;
	} else if ((character == ' ') || (character == '\t')) {
	    // White space is ignored:
	} else {
	    // Anything else causes {uart_error} to be set to 1:
	    Serial__error = 1;
	}
    }

    // Once {uart_error} is set, print an error response and return 0:
    if (Serial__error) {
	Serial__string_put(serial, "?\n");
	result = 0;
    }
    return result;
}

// This routine will output {byte} uart as a two-digit
// hexadecimal number.
void Serial__hex_byte_put(
  Serial serial,
  UByte byte)
{
    Serial__nibble_put(serial, byte >> 4);
    Serial__nibble_put(serial, byte);
}

// This routine will read in a hexadecimal number from {serial} by skipping
// over any whitespace and read in a string of hexadecimal digits and return
// the resulting value.  If no hexadecimal digit is encountered,
// {Serial__error} is set to one and 0 is returned.
Integer Serial__hex_get(
  Serial serial)
{
    UByte character;
    UInteger value;
    UByte digit;
    UByte digits_count;
    UByte negative;

    // Skip over any preceeding white space:
    Serial__white_space_skip(serial);

    // Check for negative sign:
    negative = 0;
    if (Serial__character_peek(serial) == '-') {
	// Read in the negative sign:
	Serial__character_get(serial);

	// Set negative flag:
	negative = 1;
    }

    // Read as many hexadecimal digits as are present:
    digits_count = 0;
    value = 0;
    while (1) {
	character = Serial__character_peek(serial);
	digit = 0xff;
	if (('0' <= character) && (character <= '9')) {
	    // Numeric hexadecimal digit:
	    digit = character - '0';
	} else if (('A' <= character) && (character <= 'F')) {
	     // Upper case hexadecimal digit:
	    digit = character - 'A' + 10;
	} else if (('a' <= character) && (character <= 'f')) {
	    // Lower case hexadecimal digit:
	    digit = character - 'a' + 10;
	} else {
	    // No hexadecimal digit:
	    break;
	}

	// If we get here, we have peeked a valid digit;
	// so now we read it:
	(void)Serial__character_get(serial);
	
	// Add {digit} to {value}:
	if (digit < 16) {
	    value = (value << 4) | digit;
	    digits_count++;
	}
    }

    // Figure out if we have an error:
    if (digits_count == 0) {
	Serial__error = 1;
    }

    // See if we have to invert the sign:
    if (negative) {
	value = -value;
    }

    // Return {value}:
    return value;
}

// This routine will output {value} as a hexadecimal number.
// Negative numbers are prefaced with a minus sign.
void Serial__hex_put(
  Serial serial,
  Integer value)
{
    UByte index;
    UByte nibble;
    UByte print;
    UByte shift;

    if (value < 0) {
 	value = -value;
	Serial__character_put(serial, '-');
    }

    shift = 32;
    print = 0;
    for (index = 0; index < 8; index++) {
	shift -= 4;
	nibble = (UByte)((value >> shift) & 0xf);
	if (nibble != 0 || index == 7) {
	    print = 1;
	}
	if (print) {
	    Serial__nibble_put(serial, nibble);
	}
    }
}

void Serial__float_label(
  Serial serial,
  char *label,
  Float value)
{
    // This routine will output {label} followed by {value} to serial.

    Serial__string_put(serial, label);
    Serial__character_put(serial, ':');
    Serial__float_put(serial, value);
    Serial__character_put(serial, ' ');
}

void Serial__float_put(
  Serial serial,
  Float value)
{
    // This routine will output {value} to {serial}.

    Integer int_value = (Integer)(value * (Float)0x100);
    if (int_value < 0) {
	Serial__character_put(serial, '-');
	int_value = -int_value;
    }    
    Serial__hex_put(serial, int_value >> 8);
    Serial__character_put(serial, '.');
    Serial__hex_put(serial, int_value & 0xff);
}

// This routine will initialize the serial UART and return a point
// to the associated UART data structure.
Serial Serial__initialize(
  Serial serial,
  Uart uart,
  UInteger baud_rate,
  UByte function,
  UByte port,
  UByte tx_bit,
  UByte rx_bit,
  Byte interrupt_number,
  UByte interrupt_priority,
  UByte match_address)
{
    PINSEL_CFG_Type pin_config;
    UART_CFG_Type uart_config;
    UART_FIFO_CFG_Type uart_fifo_config;

    // Initialize {serial}:
    serial->uart = uart;
    serial->receive = Ring_Buffer__initialize(&serial->receive_buffer);
    serial->transmit = Ring_Buffer__initialize(&serial->transmit_buffer);
    serial->preview_available = 0;

    //FIXME: Pins need to be set depending upon the UART:
    // Initialize SERIAL pins:
    pin_config.Funcnum = function;
    pin_config.OpenDrain = 0;
    pin_config.Pinmode = 0;
    pin_config.Pinnum = tx_bit;
    pin_config.Portnum = port;
    PINSEL_ConfigPin(&pin_config);
    pin_config.Pinnum = rx_bit;
    PINSEL_ConfigPin(&pin_config);

    // Initialize UART Configuration parameter structure to default state:
    // Baudrate = {baud_rate}
    // 8 data bit
    // 1 Stop bit
    // No parity
    UART_ConfigStructInit(&uart_config);
    uart_config.Baud_rate = baud_rate;
    UART_Init(uart, &uart_config);

    // Initialize FIFOConfigStruct to default state:
    // 				- FIFO_DMAMode = DISABLE
    // 				- FIFO_Level = UART_FIFO_TRGLEV0
    // 				- FIFO_ResetRxBuf = ENABLE
    // 				- FIFO_ResetTxBuf = ENABLE
    // 				- FIFO_State = ENABLE
    UART_FIFOConfigStructInit(&uart_fifo_config);
    UART_FIFOConfig(uart, &uart_fifo_config);

    // Deal with special initialization needs of UART1:
    if (uart == (Uart)LPC_UART1) {
	// Configure RS485:
	//
	// - Auto Direction in Tx/Rx driving is enabled
	// - Direction control pin is set to DTR1
	// - Direction control pole is set to "1" that means direction pin
	//   will drive to high state before transmit data.
	// - Multidrop mode is disable
	// - Auto detect address is disabled
	// - Receive state is enable
	UART1_RS485_CTRLCFG_Type rs485_config;

	rs485_config.AutoDirCtrl_State = ENABLE;
	rs485_config.DirCtrlPin = UART1_RS485_DIRCTRL_DTR;
	rs485_config.DirCtrlPol_Level = SET;
	rs485_config.DelayValue = 0;
	if (0) {
	    // Recieve everything on bus:
	    rs485_config.NormalMultiDropMode_State = DISABLE;
	    rs485_config.AutoAddrDetect_State = DISABLE;
	    rs485_config.MatchAddrValue = 0;
	    rs485_config.Rx_State = ENABLE;
	} else {
	    // Receive just stuff addressed to {match_address}:
	    rs485_config.NormalMultiDropMode_State = ENABLE;
	    rs485_config.AutoAddrDetect_State = ENABLE;
	    rs485_config.MatchAddrValue = match_address;
	    rs485_config.Rx_State = DISABLE;
	}
	UART_RS485Config((LPC_UART1_TypeDef *)uart, &rs485_config);
    }

    if (uart == (Uart)LPC_UART0) {
	// Enable UART Transmit:
	UART_TxCmd(uart, ENABLE);

	// Do not enable transmit interrupt here, since it is handled by
	// uart_send() function, just to reset Tx Interrupt state for the
	// first time
	uart_transmit_interrupt_status = RESET;

	if (interrupt_number >= 0) {
	    // preemption = 1, sub-priority = {interrupt_priority}:
	    NVIC_SetPriority(interrupt_number, ((interrupt_priority<<3)|0x01));

	    // Enable Interrupt for UART channel:
	    NVIC_EnableIRQ(interrupt_number);
	}

	// Enable UART Rx interrupt:
	UART_IntConfig(uart, UART_INTCFG_RBR, ENABLE);

	// Enable UART line status interrupt:
	UART_IntConfig(uart, UART_INTCFG_RLS, ENABLE);
    }

    // We are done:
    return serial;
}

void Serial__interrupt(
  Serial serial)
{
    // This routine processes an interrupt for {serial}.

    UInteger interrupt_source;
    UInteger line_flags;
    UInteger flags;

    Uart uart = serial->uart;

    // Determine the interrupt source:
    interrupt_source = UART_GetIntId(uart);
    flags = interrupt_source & UART_IIR_INTID_MASK;

    // Receive Line Status:
    if (flags == UART_IIR_INTID_RLS) {
	// Check line status:
	line_flags = UART_GetLineStatus(uart);

	// Mask out the Receive Ready and Transmit Holding empty status:
	line_flags &= (UART_LSR_OE |
	  UART_LSR_PE | UART_LSR_FE | UART_LSR_BI | UART_LSR_RXFE);

	// If any error exists:
	if (line_flags != 0) {
	    uart_interrupt_error(uart, line_flags);
	}
    }

    // Receive Data Available or Character time-out:
    if ((flags == UART_IIR_INTID_RDA) || (flags == UART_IIR_INTID_CTI)) {
	Serial__interrupt_receive(serial);
    }

    // Transmit Holding Empty:
    if (flags == UART_IIR_INTID_THRE) {
	Serial__interrupt_transmit(serial);
    }
}

void Serial__interrupt_receive(
  Serial serial)
{
    // This routine will process a empty frames from the UART FIFO associated
    // with {serial} into the ring buffer associated with {serial}.

    UByte character;
    UInteger amount_received;

    Ring_Buffer receive = serial->receive;
    Uart uart = serial->uart;

    while (1) {
	// Call UART read function in UART driver:
	amount_received = UART_Receive(uart, &character, 1, NONE_BLOCKING);
	Frame frame = (Frame)character;

	// If data received:
	if (amount_received != 0) {
	    // Check if buffer is more space
	    // If no more space, remaining character will be trimmed out:
	    if (!Ring_Buffer__is_full(receive)) {
		receive->frames[receive->head] = frame;
		Ring_Buffer__head_increment(receive);
	    }
	} else {
	    // no more data:
	    break;
	}
    }
}

void Serial__interrupt_transmit(
  Serial serial)
{
    // This routine empties the transmit ring buffer associated with
    // {serial} into the transmit fifo associated with {serial}.

    // Grab some data structures from {serial}:
    Ring_Buffer transmit = serial->transmit;
    Uart uart = serial->uart;

    // Disable THRE interrupt:
    UART_IntConfig(uart, UART_INTCFG_THRE, DISABLE);

    // Wait for FIFO buffer empty, transfer UART_TX_FIFO_SIZE bytes
    // of data or break whenever ring buffers are empty:

    // Wait until THR empty:
    while (UART_CheckBusy(uart) == SET) {
	// Do nothing:
    }

    while (!Ring_Buffer__is_empty(transmit)) {
	// Move a piece of data into the transmit FIFO
	Frame frame = transmit->frames[transmit->tail];
	UByte ubyte = (UByte)frame;

        if (UART_Send(uart, &ubyte, 1, NONE_BLOCKING)) {
	    // Update transmit ring FIFO tail pointer
	    Ring_Buffer__tail_increment(transmit);
    	} else {
	    break;
    	}
    }

    // If there is no more data to send, disable the transmit
    // interrupt - else enable it or keep it enabled:
    if (Ring_Buffer__is_empty(transmit)) {
    	UART_IntConfig(uart, UART_INTCFG_THRE, DISABLE);

	// Reset Tx Interrupt state:
    	uart_transmit_interrupt_status = RESET;
    } else {
	// Set Tx Interrupt state:
	uart_transmit_interrupt_status = SET;
    	UART_IntConfig(uart, UART_INTCFG_THRE, ENABLE);
    }
}

// This routine will {label} followed by {value} printed in hex to {serial}.
void Serial__label_hex(
  Serial serial,
  char *label,
  Integer value)
{
    // Output {label} followed by a colon:
    Serial__string_put(serial, label);
    Serial__character_put(serial, ':');

    // Output {value} in hexadecimal followed by a space:
    Serial__hex_put(serial, value);
    Serial__character_put(serial, ' ');
}

// This routine the next letter from {serial} and returns it.
// If the character is upper case, it is converted to lower case.
// If a non-letter is encountered, it is not read, {Serial__error} is
// set to one, and '!' is returned.
UByte Serial__letter_get(
  Serial serial)
{
    UByte character;

    // First peek a character:
    character = Serial__character_peek(serial);

    // If upper case, convert to lower case:
    if (('A' <= character) && (character <= 'Z')) {
	character += 'a' - 'A';
    }
    // If it is a lower case character, now "get" it:
    if (('a' <= character) && (character <= 'z')) {
	Serial__character_get(serial);
    } else {
	// Return a non-letter as an error:
	character = '!';
	Serial__error = 1;
    }
    return character;
}

// This routine will output the low order 4 bits of {nibble}
// as a single hexadecimal digit.
void Serial__nibble_put(
  Serial serial,
  UByte nibble)
{
    Serial__character_put(serial, "0123456789abcdef"[nibble & 15]);
}

UInteger Serial__receive(
  Serial serial,
  Frame *buffer,
  UInteger amount)
{
    // This routine will receive up to {amount} bytes of data from {serial}
    // into {buffer}.  The total number of bytes received (possibly 0)
    // is returned.

    Frame *data = buffer;
    UInteger bytes = 0;
    Ring_Buffer receive = serial->receive;
    Uart uart = serial->uart;

    // Temporarily lock out UART receive interrupts during this
    // read so the UART receive interrupt won't cause problems
    // with the index values
    UART_IntConfig(uart, UART_INTCFG_RBR, DISABLE);

    // Loop until receive buffer ring is empty or until max_bytes expires:
    while ((amount > 0) && (!Ring_Buffer__is_empty(receive))) {
	// Read data from ring buffer into user buffer
	*data = receive->frames[receive->tail];
	data++;

	// Update tail pointer
	Ring_Buffer__tail_increment(receive);

	// Increment data count and decrement buffer size count
	bytes++;
	amount--;
    }

    // Re-enable UART interrupts:
    UART_IntConfig(uart, UART_INTCFG_RBR, ENABLE);

    return bytes;
}

// This routine will receive {amount} byte of data from {uart} into {buffer}.
// This routine blocks until all {amount} bytes have been received.
void Serial__receive_blocking(
  Serial serial,
  Frame *buffer,
  UInteger amount)
{
    UInteger received;

    while (amount != 0) {
	received = Serial__receive(serial, buffer, amount);
	buffer += received;
	amount -= received;
    }
}

// This routine will attempt to send up to {amount} bytes from {buffer}
// to {uart}.  The number of bytes send (possibly 0) is returned.
UInteger Serial__send(
  Serial serial,
  Frame *buffer,
  UInteger amount)
{
    Frame *data = buffer;
    UInteger bytes = 0;
    Uart uart = serial->uart;
    Ring_Buffer transmit = serial->transmit;

    // Temporarily lock out UART transmit interrupts during this
    // read so the UART transmit interrupt won't cause problems
    // with the index values:
    UART_IntConfig(uart, UART_INTCFG_THRE, DISABLE);

    // Loop until transmit run buffer is full or until n_bytes expires:
    while ((amount > 0) && (!Ring_Buffer__is_full(transmit))) {
	// Write data from buffer into ring buffer:
	transmit->frames[transmit->head] = *data;
	data++;

	// Increment head pointer:
	Ring_Buffer__head_increment(transmit);

	// Increment data count and decrement buffer size count:
	bytes++;
	amount--;
    }

    // If the current {uart_transmit_interrupt_status} is reset, it means
    // the Tx interrupt must be re-enabled via a call to
    // {Serial__interrupt_transmit}():
    if (uart_transmit_interrupt_status == RESET) {
	Serial__interrupt_transmit(serial);
    } else {
	// Otherwise, re-enable Tx Interrupt: */
	UART_IntConfig(uart, UART_INTCFG_THRE, ENABLE);
    }

    return bytes;
}

void Serial__send_blocking(
  Serial serial,
  Frame *buffer,
  UInteger amount)
{
    // This routine will send the {amount} bytes in {buffer} to {serial}.
    // This routine will block until all {amount} bytes are in {buffer}.

    UInteger tmp, tmp2;
    Frame *data;

    tmp = amount;
    tmp2 = 0;
    data = buffer;
    while (tmp != 0) {
	tmp2 = Serial__send(serial, data, tmp);
	data += tmp2;
	tmp -= tmp2;
    }
}

// Serial__string_put(uart, string) will output {string} to {uart}.
void Serial__string_put(
  Serial serial,
  char *string)
{
    UByte character;

    // Output characters until '\0' (NULL) is encountered:
    character = '!';
    while (character != '\0') {
	// Get the character:
	character = (UByte)*string++;

	// Output the character:
	Serial__character_put(serial, character);
    }
}

// This routine will read in an skip over any spaces and tabs
// read from {serial}.

void Serial__white_space_skip(
  Serial serial)
{
    UByte character;

    while (1) {
	// peek the next character:
	character = Serial__character_peek(serial);

	// If it is a white space, "get" it and continue:
	if (character == ' ' || character == '\t') {
	    Serial__character_get(serial);
	} else {
	    // Otherwise, we are done:
	    break;
	}
    }
}

// SysTick routines:

// This interrupt routine will increment {SysTickCnt} every millisecond.
void SysTick_Handler (void) {
    SysTickCnt++;
}

// This routine will delay for {ms} milliseconds.
void SysTick_delay (
  unsigned long ms)
{
    unsigned long sys_tick_count;

    // Get current value:
    sys_tick_count = SysTickCnt;

    // Wait until we get the value we want:
    while ((SysTickCnt - sys_tick_count) < ms) {
	// do nothing:
    }
}

// {Uart} routines:

// This routine is invoked everytime UART0 issues an intterupt.  This routine
// hands both transmit and receive interrupts.
void UART0_IRQHandler(void)
{
    Serial__interrupt(&Serial__uart0);
}

// This routine is invoked everytime UART0 issues an intterupt.  This routine
// hands both transmit and receive interrupts.
void UART1_IRQHandler(void)
{
    Serial__interrupt(&Serial__uart1);
}

// This routine is invoked everytime UART3 issues an intterupt.  This routine
// hands both transmit and receive interrupts.
void UART3_IRQHandler(void)
{
    Serial__interrupt(&Serial__uart3);
}

void Uart__frame_put(
  Uart1 uart1,
  UShort frame)
{
    // This routine will send {frame} to the to {uart}.
    // The echo byte from sending {byte} is read back.
    UByte buffer[1];

    // Send {byte} to {robus}:
    if ((frame & 0x100) != 0) {
	// Set 9th bit here:
	(void)UART_RS485SendSlvAddr(uart1, frame);
    } else {
	buffer[0] = (UByte)frame;
	(void)UART_RS485SendData(uart1, buffer, 1);
    }

    TRACE(frame | 0x200);

    // Read the echo back from {robus}:
    //(void)Uart__frame_get(uart1);
}

UShort Uart__frame_get(
  Uart1 uart1)
{
    // This routine will get one byte of response Robus bus attached to {robus}.
    // If no byte is found after a resonable amount of time, 0x5a is returned
    // instead.

    UShort frame;
    UByte buffer[1];
    UByte count;
    UByte tries;

    // Try for a while to get the byte:
    frame = 0x5a;
    for (tries = 0; tries < 20; tries++) {
	// See if anything has come in yet:
	count = UART_Receive((Uart)uart1, buffer, 1, NONE_BLOCKING);
	//count = UARTReceive(robus->uart, buffer, 1);
	if (count == 1) {
	    // We got a byte; return it:
	    frame = buffer[0];
	    TRACE(frame | 0x1000);
	    return frame;
	}

	// Wait for a millisecond, to give the bus some time:
	SysTick_delay(1);
    }

    // We have timed out; return 0x5a:
    TRACE(0x800 | 0x5a);

    return 0x5a;
}

// This routine will process {line_flags} for {uart}.
void uart_interrupt_error(
  Uart uart,
  UInteger line_flags)
{
    // Loop forever:
    while (1) {
	// do nothing:
    }
}

#ifdef  DEBUG
// This routine is called whenever an check macro fails.
void check_failed(
  UByte *file,
  UInteger line)
{
    // User can add his own implementation to report the file name and
    // line number, example:
    //    printf("Wrong parameters value: file %s on line %d\r\n", file, line);

    // Infinite loop:
    while (1) {
	// do nothing
    }
}
#endif

void trace_dump(
  Serial serial)
{
    Integer index = 0;
    for (index = 0; index < TRACE_SIZE; index++) {
        // Fetch the {trace} value and clear it out of the trace buffer:
	UInteger masked_index = (index + trace_index) & TRACE_MASK;
	UInteger trace = trace_buffer[masked_index];
	trace_buffer[masked_index] = 0;

	// Show any tracing flags:
	if ((trace & 0x1000) != 0) {
	    // Received (Get) byte:
	    Serial__character_put(serial, 'G');
	    trace &= ~0x1000;
	}
	if ((trace & 0x200) != 0) {
	    // Transmitted (Put) byte:
	    Serial__character_put(serial, 'P');
	    trace &= ~0x200;
	}
	if ((trace & 0x800) != 0) {
	    // Timeout byte:
	    Serial__character_put(serial, 'T');
	    trace &= ~0x800;
	}

	// Output the remaining trace value:
	Serial__hex_put(serial, trace);
	char character = ' ';
	if ((index & 0xf) == 0xf) {
	    character = '\n';
	}
	Serial__character_put(serial, character);
    }
}
