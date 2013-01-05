// Copyright (c) 2011-2012 by IMC.  All rights reserved.

#include "common.h"
#include "ssp.h"
#include "pwm.h"
#include "motor.h"

// Main routines:
Integer main(void);
Integer c_entry(void);
Integer abs(Integer value);

// Command parse routines:
void command_id_show(Serial serial, Robus robus);
void command_id_string_show(Serial serial, Robus robus);
UByte command_id_byte_show(Serial serial, Robus robus);
//void command_parse_navigate(Serial serial, Motor2 motor2, Shaft2 shaft2);

// Routine definitions from here on:

// Most compilers programs start from main...:

void check_encoder(void)
{
    int val;
    //char buffer[10];

    val = (int)ssp_read();
    //itoa(val & 0x3f, &buffer, 2);
    //uart_send(&buffer);

    //uart_send(itoa(val & 0x3f, 2));
    //uart_send(itoa((val & 0x4000) >> 14, 2));
    //uart_send("  ");

    //itoa(val >> 6, &buffer, 10);
    //uart_send(&buffer);

    //uart_send(itoa(val >> 6, 10));
    //uart_send(itoa(val & 0x3FFF, 10));
    //uart_send("\r\n");
}

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
      (Uart)LPC_UART0, 115200, 1, 0, 2, 3, interrupt_number, 0x01);

    // Make sure that _UART3 is defined in libcfg.h!:
    //Serial debug = Serial__initialize(&Serial__uart3,
    //  (Uart)LPC_UART3, 115200, 2, 0, 0, 1, UART3_IRQn, 0x04);
    //Serial__string_put(debug, "debug:\n");

    Robus robus = Robus__null;
    Robus__initialize(robus, console, Buffer__get_buffer, Buffer__put_buffer);

    Serial__string_put(console, "Hello\n");

    Uart uart_8bit = console->uart;
    Uart uart_9bit = robus->uart;
    Frame high_bits = (Frame)-1;
    Frame frame8_out = (Frame)-1;
    Frame frame9_out = (Frame)-1;
    Logical echo_surpress = (Logical)0;

    //led_init();
    motor_init();
    pwm_init();
    ssp_init();
    //uart_init();

    //uart_send("SystemCoreClock = ");
    //uart_send(itoa(SystemCoreClock, 10));
    //uart_send("\r\n");

    //uart_send("ssp_clk = ");
    //uart_send(itoa(CLKPWR_GetPCLK(CLKPWR_PCLKSEL_SSP0), 10));
    //uart_send("\r\n");

    //test = 3.141592;
    //printf("test = %f\r\n", test);
 
    while (1) {
	int i;
	//led_on();
	motor_forward();
	for (i=0; i<=256; i+=16) {
	    pwm_update(i);
	    check_encoder();
	    SysTick__delay(250);
	}

	//led_off();
	motor_stop();
	pwm_update(0);
	SysTick__delay(1000);

	//led_on();
	motor_reverse();
	for (i=0; i<=256; i+=16) {
	    pwm_update(i);
	    check_encoder();
	    SysTick__delay(250);
	}

	//led_off();
	motor_stop();
	pwm_update(0);
	SysTick__delay(1000);
    }

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
	    //} else if (frame8_in == 0xc4) {
	    //	Discovery__scan(discovery);
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

    Serial__string_put(console, "Robus:\n");
    // Basic command loop:
    while (1) {
	UByte command_letter;

	// Generate a prompt:
	Serial__character_put(console, '>');
	Serial__character_put(console, '>');

	// Reset errors:
	Serial__error = 0;

	// Parse the command letter:
	Serial__white_space_skip(console);
	command_letter = Serial__letter_get(console);

	switch (command_letter) {
	  case 'd':
	    // D ; dump trace buffer
	    if (Serial__end_of_line(console)) {
		trace_dump(console);
	    }
	    break;
	  case 'l':
	    {
		Integer character_code = Serial__hex_get(console);
		if (Serial__end_of_line(console)) {
		    Robus__request_begin(robus, 0x92, 0x10);
		    Robus__request_ubyte_put(robus, character_code);
		    Robus__request_end(robus);
		}
	    }
	    break;
	  case 't':
	    // T {ubyte} ; Send {ubyte} to LCD:
	    {
		Integer character_code = Serial__hex_get(console);
		if (Serial__end_of_line(console)) {
		    Serial__label_hex(console,
		      "char", (Character)character_code);
		    Serial__character_put(console, '\n');
		}
	    }
	    break;
	  default:
	    // Unknown command:
	    if (Serial__end_of_line(console)) {
		Serial__string_put(console, "Huh?\n");
	    }
	    break;
	}
    }

    // We never get here:
    return 1;
}

