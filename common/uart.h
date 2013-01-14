#ifndef UART_H_INCLUDED
#define UART_H_INCLUDED 1

// Copyright (c) 2011-2013 by IMC.  All rights reserved.

// Pointer structure typedef's occur before #includes':
typedef struct LPC_UART__Struct *Uart;
typedef struct LPC_UART1__Struct *Uart1;

// #incluede's come next:
#include "types.h"

// Typedefs for serial and robus UART's:
// A frame contains 9-bits of data and potentially some other flags
// from the UART receive FIFE.  We use a signed value so that a negative
// value can indicate "no frame received".
typedef Short Frame;

// {Uart} routines:
void Uart__frame_put(Uart1 uart1, Frame frame);
UShort Uart__frame_get(Uart1 uart1);

// Define all four UART interrupt handlers regardless of whether
// or not they actually get implemented:
void UART0_IRQHandler(void);
void UART1_IRQHandler(void);
void UART2_IRQHandler(void);
void UART3_IRQHandler(void);

void uart_interrupt_error(Uart uart, UInteger line_flags);
volatile Logical uart_transmit_interrupt_status;

#endif // UART_H_INCLUDED
