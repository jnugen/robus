// Copyright (c) 2011-2013 by IMC.  All rights reserved.

#include "lpc17xx_uart.h"
#include "robus.h"
#include "trace.h"

// {Robus} data structures and routines:
struct Robus__Struct Robus___null_struct;
Robus Robus__null = &Robus___null_struct;

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
	SysTick__delay(1);
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

