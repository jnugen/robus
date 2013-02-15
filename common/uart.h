#ifndef UART_H_INCLUDED
#define UART_H_INCLUDED 1

// Copyright (c) 2011-2013 by IMC.  All rights reserved.

// Pointer structure typedef's occur before #includes':
typedef struct LPC_UART__Struct *Uart;
typedef struct LPC_UART1__Struct *Uart1;

// #incluede's come next:
#include "types.h"
#include "lpc17xx_uart.h"

// Typedefs for serial and robus UART's:
// A frame contains 9-bits of data and potentially some other flags
// from the UART receive FIFE.  We use a signed value so that a negative
// value can indicate "no frame received".
typedef Int16 Frame;

// {Uart} routines:
void Uart__transmit_enable(Uart uart, Bool8 enable);
Bool8 Uart__transmit_possible(Uart uart);
Bool8 Uart__is_transmitting(Uart uart);
void Uart__fifo_configure(Uart uart, Bool8 dma_mode, Bool8 reset_receive_buffer,
  Bool8 reset_transmit_buffer, UInt8 trigger_level);
// {Uart1} routines:
void Uart1__frame_put(Uart1 uart1, Frame frame);
Frame Uart1__frame_get(Uart1 uart1);
void Uart1__receive_enable(Uart1 uart1, Bool8 enable);
void Uart1__transmit_enable(Uart1 uart1, Bool8 enable);
void uart1__configure(Uart1 uart1, Bool8 auto_direction_control,
  UInt8 direction_control_pin, Bool8 direction_control_polarity,
  UInt8 delay_value, Bool8 normal_multi_drop_mode, Bool8 auto_address_detect,
  UInt8 match_address_value, Bool8 receive_state);

// Define all four UART interrupt handlers regardless of whether
// or not they actually get implemented:
void UART0_IRQHandler(void);
void UART1_IRQHandler(void);
void UART2_IRQHandler(void);
void UART3_IRQHandler(void);

void uart_interrupt_error(Uart uart, UInt32 line_flags);
volatile Bool8 uart_transmit_interrupt_status;

#endif // UART_H_INCLUDED
