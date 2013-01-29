// Copyright (c) 2011-2013 by IMC.  All rights reserved.

#include "lpc17xx_uart.h"
#include "serial.h"
#include "systick.h"
#include "trace.h"
#include "uart.h"

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
  Frame frame)
{
    // This routine will send {frame} to the to {uart}.
    // The echo byte from sending {byte} is read back.
    UInt8 buffer[1];

    // Send {byte} to {robus}:
    if ((frame & 0x100) != 0) {
	// Set 9th bit here:
	(void)UART_RS485SendSlvAddr(uart1, frame);
    } else {
	buffer[0] = (UInt8)frame;
	(void)UART_RS485SendData(uart1, buffer, 1);
    }

    TRACE(frame | 0x200);

    // Read the echo back from {robus}:
    //(void)Uart__frame_get(uart1);
}

UInt16 Uart__frame_get(
  Uart1 uart1)
{
    // This routine will get one byte of response Robus bus attached to {robus}.
    // If no byte is found after a resonable amount of time, 0x5a is returned
    // instead.

    UInt16 frame;
    UInt8 buffer[1];
    UInt8 count;
    UInt8 tries;

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
	SysTick__delay(1);
    }

    // We have timed out; return 0x5a:
    TRACE(0x800 | 0x5a);

    return 0x5a;
}

// This routine will process {line_flags} for {uart}.
void uart_interrupt_error(
  Uart uart,
  UInt32 line_flags)
{
    // Loop forever:
    while (1) {
	// do nothing:
    }
}

