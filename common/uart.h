// Copyright (c) 2011-2012 by IMC.  All rights reserved.

#ifndef UART_H_INCLUDED
#define UART_H_INCLUDED 1

#include "types.h"

typedef LPC_UART_TypeDef *Uart;

// {Uart} routines:
void Uart__frame_put(Uart uart, UShort frame);
UShort Uart__frame_get(Uart uart);

// Define all four UART interrupt handlers regardless of whether
// or not they actually get implemented:
void UART0_IRQHandler(void);
void UART1_IRQHandler(void);
void UART2_IRQHandler(void);
void UART3_IRQHandler(void);

void uart_interrupt_error(Uart uart, UInteger line_flags);
__IO FlagStatus uart_transmit_interrupt_status;

#endif // UART_H_INCLUDED
