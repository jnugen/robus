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

Bool8 Uart__transmit_possible(
  Uart uart)
{
    Bool8 result = (Bool8)0;
    if ((uart->LSR & UART_LSR_THRE) != 0) {
	result = (Bool8)1;
    }
    return result;
}

void Uart__byte_put(
  Uart uart,
  UInt8 byte)
{
    // This routine will send {byte} to the to {uart}.

    // Wait until there is room in the buffer to put the frame:
    while ((uart->LSR & UART_LSR_THRE) == 0) {
	// Do nothing:
    }

    // Stuff the frame into the transmit buffer:
    uart->THR = byte & UART_THR_MASKBIT;

    // Trace what was sent:
    TRACE(byte | 0x200);
}

void Uart1__frame_put(
  Uart1 uart1,
  Frame frame)
{
    // This routine will send {frame} to the to {uart1}.

    // The Line Control Register (LCR) controls the bits per frame,
    // number of stop bits, and parity.  Depending upon whether the
    // 9th bit of {frame} is 1 or 0, we need to the force parity bit
    // to be set correctly.

    // Read out the current LCR into a local value, masking off
    // any bits that we are not supposed to look at:
    UInt32 current_lcr = uart1->LCR & UART_LCR_BITMASK;

    // {UART_LCR_PARITY_F_0} = 3<<4 which will mask out the two
    // bits used to represent the parity mode:
    UInt32 desired_lcr = current_lcr & ~UART_LCR_PARITY_F_0;

    // Now compute the desired LCR value based on the 9th bit
    // of {frame9_out}:
    if ((frame & 0x100) == 0) {
	// 9th bit of {frame9_out} is 0; "force 0 parity" mode;
	desired_lcr |= UART_LCR_PARITY_F_0;
    } else {
	// 9th bit of {frame9_out} is 1; "force 1 parity" mode:
	desired_lcr |= UART_LCR_PARITY_F_1;
    }

    // If the LCR needs to change, we must wait until the transmit
    // FIFO is completely empty (LSR_TEMT bit is set.)  Otherwise,
    // we just have to wait until there is room in the FIFO (LSR_THRE
    // is set.):
    if (current_lcr != desired_lcr) {
	// We have to change the LSR parity bit:

	// Wait for TEMT (Transmitter EMpTy) flag to become set:
	while ((uart1->LSR & UART_LSR_TEMT) == 0) {
	    // Do nothing:
	}
	uart1->LCR = desired_lcr;
    }

    // Wait until there is room in the buffer to put the frame:
    while ((uart1->LSR & UART_LSR_THRE) == 0) {
	// Do nothing:
    }

    // Stuff the frame into the transmit buffer:
    uart1->THR = frame & UART_THR_MASKBIT;

    // Trace what was sent:
    TRACE(frame | 0x200);
}

Frame Uart1__frame_get(
  Uart1 uart1)
{
    // This routine will get a 9-bit frame from {uart1} and return it.
    // If no frame found after a resonable amount of time, -1 is returned
    // instead.

    // Initialize {frame} to the timeout value:
    Frame frame = (Frame)-1;

    // Try for a while to get the byte:
    UInt32 tries = 0;
    for (tries = 0; tries < 20; tries++) {
	// Check whether there is a frame available on {uart1}.  Stash the
	// LSR into a local variable because reading the LSR register
	// clears all the error bits:
	UInt32 lsr = uart1->LSR;	// LSR = Line Status Register
	
	// Now check to see if whe have a frame:
	if ((lsr & UART_LSR_RDR) != 0) {	// RDR = Read Data Ready
	    // A parity error indicates that we have the 9th address bit set:
	    if ((lsr & UART_LSR_PE) != 0) {	// PE = Parity erro
		// We have an address frame:
		frame |= 0x100;
	    }

	    // Now grab the 8-bits of data (RBR = Read Buffer Register):
	    frame |= (Frame)(uart1->RBR & UART_RBR_MASKBIT); 

	    // Trace the received frame:
	    TRACE(frame | 0x1000);

	    // Done; break out of loop:
	    break;
	} else {
	    // Wait for a millisecond, to give the bus some time:
	    SysTick__delay(1);
	}
    }

    // Check for timeout:
    if (frame < 0) {
	// We have timed out; mark timeout in trace buffer:
	TRACE(0x800 | 0xff);
    }

    return frame;
}

// This routine will enable receiving for {uart1} if {enable} is non-zero.
void Uart1__receive_enable(
  Uart1 uart1,
  Bool8 enable)
{
    if (enable) {
	uart1->RS485CTRL &= ~UART1_RS485CTRL_RX_DIS;
    } else {
	uart1->RS485CTRL |= UART1_RS485CTRL_RX_DIS;
    }
}


// This routine will enable transmission for {uart1} if {enable} is non-zero;
// Otherwise, transmission is disabled.
void Uart1__transmit_enable(
  Uart1 uart1,
  Bool8 enable)
{
    if (enable) {
	uart1->TER |= UART_TER_TXEN;
    } else {
	uart1->TER &= (~UART_TER_TXEN) & UART_TER_BITMASK;
    }
}

// This routine will enable transmission for {uart} if {enable} is non-zero;
// Otherwise, transmission is disabled.
void Uart__transmit_enable(
  Uart uart,
  Bool8 enable)
{
    if (enable) {
	uart->TER |= UART_TER_TXEN;
    } else {
	uart->TER &= (~UART_TER_TXEN) & UART_TER_BITMASK;
    }
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

Bool8 Uart__is_transmitting(
  Uart uart)
{
    Bool8 result = (Bool8)1;
    if ((uart->LSR & UART_LSR_TEMT) != 0) {
	result = (Bool8)0;
    }
    return result;
}

void Uart__interrupt_configure(
  Uart uart,
  UART_INT_Type UARTIntCfg,
  FunctionalState NewState)
{
    UInt32 tmp;

    switch (UARTIntCfg) {
      case UART_INTCFG_RBR:
	tmp = UART_IER_RBRINT_EN;
	break;
      case UART_INTCFG_THRE:
	tmp = UART_IER_THREINT_EN;
	break;
      case UART_INTCFG_RLS:
	tmp = UART_IER_RLSINT_EN;
	break;
      case UART1_INTCFG_MS:
	tmp = UART1_IER_MSINT_EN;
	break;
      case UART1_INTCFG_CTS:
	tmp = UART1_IER_CTSINT_EN;
	break;
      case UART_INTCFG_ABEO:
	tmp = UART_IER_ABEOINT_EN;
	break;
      case UART_INTCFG_ABTO:
	tmp = UART_IER_ABTOINT_EN;
	break;
    }

    if (NewState == ENABLE) {
	if ((LPC_UART1_TypeDef *)uart == LPC_UART1) {
	    ((LPC_UART1_TypeDef *)uart)->/*DLIER.*/IER |= tmp;
	} else {
	    uart->/*DLIER.*/IER |= tmp;
	}
     } else {
	if ((LPC_UART1_TypeDef *)uart == LPC_UART1) {
	    ((LPC_UART1_TypeDef *)uart)->/*DLIER.*/IER &=
	      (~tmp) & UART1_IER_BITMASK;
	} else {
	    uart->/*DLIER.*/IER &= (~tmp) & UART_IER_BITMASK;
	}
    }
}

void Uart__fifo_configure(
  Uart uart,
  Bool8 dma_mode,
  Bool8 reset_receive_buffer,
  Bool8 reset_transmit_buffer,
  UInt8 trigger_level)
{
    UInt8 fcr = UART_FCR_FIFO_EN;    
    switch (trigger_level) {
      case UART_FIFO_TRGLEV0:
	fcr |= UART_FCR_TRG_LEV0;
	break;
      case UART_FIFO_TRGLEV1:
	fcr |= UART_FCR_TRG_LEV1;
	break;
      case UART_FIFO_TRGLEV2:
	fcr |= UART_FCR_TRG_LEV2;
	break;
      case UART_FIFO_TRGLEV3:
      default:
	fcr |= UART_FCR_TRG_LEV3;
	break;
    }

    if (reset_transmit_buffer) {
	fcr |= UART_FCR_TX_RS;
    }
    if (reset_receive_buffer) {
	fcr |= UART_FCR_RX_RS;
    }
    if (dma_mode) {
	fcr |= UART_FCR_DMAMODE_SEL;
    }

    //write to FIFO control register
    if (((LPC_UART1_TypeDef *)uart) == LPC_UART1) {
	((LPC_UART1_TypeDef *)uart)->/*IIFCR.*/FCR = fcr & UART_FCR_BITMASK;
    } else {
	uart->/*IIFCR.*/FCR = fcr & UART_FCR_BITMASK;
    }

}

void uart1__configure(Uart1 uart1, 
  Bool8 auto_direction_control,
  UInt8 direction_control_pin,
  Bool8 direction_control_polarity,
  UInt8 delay_value,
  Bool8 normal_multi_drop_mode,
  Bool8 auto_address_detect,
  UInt8 match_address_value,
  Bool8 receive_state)
{
    UInt32 tmp = 0;

    // If Auto Direction Control is used in Master mode:
    if (auto_direction_control)	{
	tmp |= UART1_RS485CTRL_DCTRL_EN;

	// Set polarity:
	if (direction_control_polarity) {
	    tmp |= UART1_RS485CTRL_OINV_1;
	}

	// Set pin according to:
	if (direction_control_pin) {
	    tmp |= UART1_RS485CTRL_SEL_DTR;
	}

	// Fill delay time
	uart1->RS485DLY = delay_value & UART1_RS485DLY_BITMASK;
    }

    // MultiDrop mode is enable
    if (normal_multi_drop_mode) {
	tmp |= UART1_RS485CTRL_NMM_EN;
    }

    // Auto Address Detect function
    if (auto_address_detect) {
	tmp |= UART1_RS485CTRL_AADEN;
	// Fill Match Address
	uart1->ADRMATCH = match_address_value & UART1_RS485ADRMATCH_BITMASK;
    }

    // Receiver is disabled:
    if (!receive_state) {
	tmp |= UART1_RS485CTRL_RX_DIS;
    }

    // Write RS485 control register:
    uart1->RS485CTRL = tmp & UART1_RS485CTRL_BITMASK;

    // Enable Parity function and leave parity in stick '0' parity as default
    uart1->LCR |= (UART_LCR_PARITY_F_0 | UART_LCR_PARITY_EN);
}
