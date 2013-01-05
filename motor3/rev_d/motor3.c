// Copyright (c) 2011-2012 by IMC.  All rights reserved.

#include "common.h"
#include "ssp.h"
#include "pwm.h"
#include "motor.h"

// Main routines:
Integer main(void);
Integer c_entry(void);
Integer abs(Integer value);
void check_encoder(void);

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
      (Uart)LPC_UART0, 115200, 1, 0, 2, 3, interrupt_number, 0x01);

    Serial__string_put(console, "Hello\n");

    Robus robus = Robus__null;
    Robus__initialize(robus, console, Buffer__get_buffer, Buffer__put_buffer);

    motor_init();
    pwm_init();
    ssp_init();

    if (1) {
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
    }

    if (0) {
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
    }

    while (1) {
	// Do nothing:
    }

    // We never get here:
    return 1;
}

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

