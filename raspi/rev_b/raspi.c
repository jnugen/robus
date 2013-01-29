// Copyright (c) 2011-2012 by IMC.  All rights reserved.

#include "LPC17xx.h"
#include "lpc_types.h"
#include "lpc17xx_uart.h"

#include "common.h"

// Typedefs for serial and robus UART's:
typedef struct Discovery__Struct *Discovery;

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
} Discovery__one_and_only__struct;
Discovery Discovery__one_and_only = &Discovery__one_and_only__struct;

// {Discovery} routines:
void Discovery__byte_send(Discovery discovery, UByte ubyte);
void Discovery__hex_send(Discovery discovery, UByte uart_8bit);
void Discovery__initialize(Discovery discovery, Uart uart8_bit, Uart uart9_bit);
void Discovery__scan(Discovery discovery);
void Discovery__stack_send(Discovery discovery, UByte *stack);

//#define BLINKY 1

Integer main(void)
{
#ifdef BLINKY
    // The code below blinks the LED:
    if (1) {
	#include "lpc17xx_pinsel.h"
	// Configure P1.26 and P1.29 as digital outputs:
	PINSEL_CFG_Type pin_config;
	pin_config.Pinnum = 26;
	pin_config.Portnum = 1;
	pin_config.Funcnum = 0;
	pin_config.Pinmode = 0;
	pin_config.OpenDrain = 0;
	PINSEL_ConfigPin(&pin_config);
	pin_config.Pinnum = 29;
	PINSEL_ConfigPin(&pin_config);
	GPIO_SetDir(1, 1<<26, 1);
	GPIO_SetDir(1, 1<<29, 1);

	// The LED spans P1.26 and P1.29:
	if (1) {
	    GPIO_SetValue(1, 1<<26);
	    GPIO_ClearValue(1, 1<<29);
	} else {
	    GPIO_ClearValue(1, 1<<26);
	    GPIO_SetValue(1, 1<<29);
	}

	// Generate interrupt each 1 ms:
	SysTick_Config(SystemCoreClock/1000 - 1);

	// Blink the LED:
	while (1) {
	    GPIO_ClearValue(1, 1<<26);
	    GPIO_SetValue(1, 1<<29);
	    SysTick__delay(200);
	    GPIO_SetValue(1, 1<<26);
	    GPIO_ClearValue(1, 1<<29);
	    SysTick__delay(200);
	}
    }
#endif // BLINKY

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

    Robus robus = Robus__null;
    Robus__initialize(robus,
      console, Buffer__get_buffer, Buffer__put_buffer, 0xff);

    //Serial__string_put(console, "Hello\n");

    Uart uart_8bit = console->uart;
    Uart uart_9bit = (Uart)robus->uart1;
    Frame high_bits = (Frame)-1;
    Frame frame8_out = (Frame)-1;
    Frame frame9_out = (Frame)-1;
    Logical echo_surpress = (Logical)0;

    Discovery discovery = Discovery__one_and_only;
    Discovery__initialize(discovery, uart_8bit, uart_9bit);

    while (1) {
	// Check for a byte from the 8-bit UART and that we have a place
	// to put resulting frame for the 9-bit UART:
	if ((uart_8bit->LSR & UART_LSR_RDR) != 0 && frame9_out < (Frame)0) {
	    // Yes, we have an 8-bit byte and {frame9_out} is empty:
	    Frame frame8_in = (Frame)(uart_8bit->RBR & UART_RBR_MASKBIT);
	    if (0xc1 <= frame8_in && frame8_in <= 0xc3) {
			high_bits = (frame8_in & 3) << 7;
	    } else if (high_bits >= 0) {
		frame9_out = (frame8_in & 0x7f) | high_bits;
		high_bits = (Frame)-1;
	    } else if (frame8_in == 0xc4) {
		Discovery__scan(discovery);
	    } else if (frame8_in == 0xc5) {
		// Reset the bus by sending break for 1ms:

		// Start the break:
		UInteger lcr_save = uart_9bit->LCR & UART_LCR_BITMASK;
		uart_9bit->LCR = lcr_save | UART_LCR_BREAK_EN;

		// Wait for 10 milliseconds, to trigger reset:
		SysTick__delay(10);

		// Clear the break:
		uart_9bit->LCR = lcr_save;

		SysTick__delay(2);

		// Respond with 0xA5 to let host know that bus reset done:
		frame8_out = (Frame)0xa5;
	    } else {
		// Pass *frame8_in* to the bus "as is":
		frame9_out = frame8_in;

		// Temporary for debugging only:
		//frame8_out = frame8_in;
	    }
	}

	// Check to see if there is an 8-bit value to send out and the
	// 8-bit uart is ready to take it:
	if (frame8_out >= (Frame)0 && (uart_8bit->LSR & UART_LSR_THRE) != 0) {
	    // Send the byte on its merry way:
	    uart_8bit->THR = frame8_out & UART_THR_MASKBIT;
	    frame8_out = (Frame)-1;
	}

	// Check that there is a a frame available on the 9-bit UART:
	if ((uart_9bit->LSR & UART_LSR_RDR) != 0 &&
	  (echo_surpress || frame8_out < (Frame)0)) {
	    // Grab the 9-bit byte:
	    Frame frame9_in = (Frame)(uart_9bit->RBR & UART_RBR_MASKBIT);

	    // Deal with echo surpression:
	    if (echo_surpress) {
		echo_surpress = (Logical)0;
	    } else {
		// See about forwarding it along:
		frame8_out = frame9_in;
	    }
	}

	if (frame9_out >= (Frame)0) {
	    // The Line Control Register (LCR) controls the bits per frame,
	    // number of stop bits, and parity.  Depending upon whether the
	    // 9th bit of {frame9} is 1 or 0, we need to ensure tha the
	    // force parity bit is set correctly.

	    // Read out the current LCR into a local value, masking off
	    // any bits that we are not supposed to look at:
	    UInteger current_lcr = uart_9bit->LCR & UART_LCR_BITMASK;

	    // {UART_LCR_PARITY_F_0} = 3<<4 which will mask out the two
	    // bits used to represent the parity mode:
	    UInteger desired_lcr = current_lcr & ~UART_LCR_PARITY_F_0;

	    // Now compute the desired LCR value based on the 9th bit
	    // of {frame9_out}:
	    if ((frame9_out & 0x100) == 0) {
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
	    if ((current_lcr == desired_lcr &&
	      (uart_9bit->LSR & UART_LSR_THRE) != 0) ||
	      (current_lcr != desired_lcr &&
	      (uart_9bit->LSR & UART_LSR_TEMT) != 0)) {
		// It is safe to overwrite LCR with {desired_lcr}:
		uart_9bit->LCR = desired_lcr;

		// Send the byte on its merry way:
		uart_9bit->THR = frame9_out & UART_THR_MASKBIT;

		// Suppress the half duplex bus echo:
		echo_surpress = (Logical)1;

		// Remember {frame9_out} has been sent:
		frame9_out = (Frame)-1;
	    }
	}
    }

    while (1) {
	// Do nothing:
    }

    // We never get here:
    return 1;
}

void Discovery__initialize(
  Discovery discovery,
  Uart uart_8bit,
  Uart uart_9bit)
{
    discovery->low_address = 0;
    discovery->high_address = 0;
    discovery->top = -1;
    discovery->uart_8bit = uart_8bit;
    discovery->uart_9bit = uart_9bit;
}

// Perform MakerBus discovery:
void Discovery__scan(
  Discovery discovery)
{
    static UByte stack1[13] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
    static UByte stack2[13] = {13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1};

    Discovery__stack_send(discovery, stack1);
    Discovery__stack_send(discovery, stack2);
    Discovery__stack_send(discovery, stack1);

    Discovery__byte_send(discovery, (UByte)'!');
    Discovery__byte_send(discovery, (UByte)'\n');
}

void Discovery__stack_send(
  Discovery discovery,
  UByte *stack)
{
    UByte index;

    Discovery__byte_send(discovery, (UByte)'@');
    for (index = 0; index < 13; index++) {
	Discovery__hex_send(discovery, stack[index]);
    }
    Discovery__byte_send(discovery, (UByte)'\n');
}

void Discovery__hex_send(
  Discovery discovery,
  UByte ubyte)
{
    static Character nibble[] = "0123456789abcdef";

    ubyte &= 0xff;
    Discovery__byte_send(discovery, nibble[ubyte >> 4]);
    Discovery__byte_send(discovery, nibble[ubyte & 0xf]);
}

void Discovery__byte_send(
  Discovery discovery,
  UByte ubyte)
{
    Uart uart_8bit = discovery->uart_8bit;

    // Check to see if there is an 8-bit value to send out and the
    // 8-bit uart is ready to take it:
    while ((uart_8bit->LSR & UART_LSR_THRE) == 0) {
	// Wait until there is room:
    }
    // Send the byte on its merry way:
    uart_8bit->THR = ubyte & UART_THR_MASKBIT;
}

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
