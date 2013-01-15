// Copyright (c) 2011-2013 by IMC.  All rights reserved.

#include "lpc17xx_uart.h"
#include "lpc17xx_pinsel.h"
#include "uart.h"
#include "serial.h"

// {Serial} data structures and routines:
struct Serial__Struct Serial__uart0, Serial__uart1, Serial__uart3;

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
	if (match_address = 0xff) {
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

