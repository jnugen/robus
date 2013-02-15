// Copyright (c) 2011-2013 by IMC.  All rights reserved.

#include "common.h"

#include "LPC17xx.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_libcfg.h"
#include "lpc17xx_pinsel.h"
#include "motor.h"
#include "pwm.h"

typedef struct Motor3__Struct *Motor3;

struct Motor3__Struct {
    Int8 speed;			// Signed motor speed
    Bool8 direction_invert;	// Invert motor speed sign
};

// {Motor3} routines:
void Motor3__speed_update(Motor3 motor3, Serial debug_serial);
UInt8 Motor3__process(void *motor3__pointer,
   Robus robus, UInt8 command, Bool8 execute);

// Main routines:
Int32 main(void);
Int32 abs(Int32 value);

// Command parse routines:
void command_id_show(Serial serial, Robus robus);
void command_id_string_show(Serial serial, Robus robus);
UInt8 command_id_byte_show(Serial serial, Robus robus);
//void command_parse_navigate(Serial serial, Motor2 motor2, Shaft2 shaft2);

// {Bool8} stuff:
const Bool8 Bool8__true = (Bool8)1;
const Bool8 Bool8__false = (Bool8)0;


// Routine definitions from here on:

// Most compilers programs start from main...:

Int32 main(void)
{
#ifdef BLINKY
    if (1) {
	#include "lpc17xx_gpio.h"
	// Configure P0.0 and P0.1 as digital outputs:
	PINSEL_CFG_Type pin_config;
	pin_config.Pinnum = 0;
	pin_config.Portnum = 0;
	pin_config.Funcnum = 0;
	pin_config.Pinmode = 0;
	pin_config.OpenDrain = 0;
	PINSEL_ConfigPin(&pin_config);
	pin_config.Pinnum = 1;
	PINSEL_ConfigPin(&pin_config);
	GPIO_SetDir(0, 1<<0, 1);
	GPIO_SetDir(0, 1<<1, 1);

	// The LED spans P1.26 and P1.29:
	if (1) {
	    GPIO_SetValue(0, 1<<0);
	    GPIO_ClearValue(0, 1<<1);
	} else {
	    GPIO_ClearValue(0, 1<<0);
	    GPIO_SetValue(0, 1<<1);
	}

	// Generate interrupt each 1 ms:
	SysTick_Config(SystemCoreClock/1000 - 1);

	// Blink the LED:
	while (1) {
	    GPIO_ClearValue(0, 1<<0);
	    GPIO_SetValue(0, 1<<1);
	    SysTick__delay(200);
	    GPIO_SetValue(0, 1<<0);
	    GPIO_ClearValue(0, 1<<1);
	    SysTick__delay(200);
	}
    }
#endif // BLINKY

    // Make sure that global variable SystemCoreClock is correct:
    SystemCoreClockUpdate();

    // Generate interrupt each 1 ms:
    SysTick_Config(SystemCoreClock/1000 - 1);

    // Initialize the "console" UART to 115200 bps, using alternate function 1
    // on P0.2(TXD0) or P0.3(RXD0):
    Int8 interrupt_number = UART0_IRQn;
    interrupt_number = (Int8)-1;	// Disable interrupts
    Serial console = Serial__initialize(&Serial__uart0,
      (Uart)LPC_UART0, 115200, 1, 0, 2, 3, interrupt_number, 0x01, 0);

    // Make sure that _UART3 is defined in libcfg.h!:
    //Serial debug = Serial__initialize(&Serial__uart3,
    //  (Uart)LPC_UART3, 115200, 2, 0, 0, 1, UART3_IRQn, 0x04);
    //Serial__string_put(debug, "debug:\n");

    UInt8 slave_address = 133;
    Robus robus = Robus__null;
    Robus__initialize(robus,
      console, Buffer__get_buffer, Buffer__put_buffer, slave_address);
    Uart1 uart1 = robus->uart1;

    uart1->LCR |= UART_LCR_PARITY_EN;
    Serial__string_put(console, "Motor3:");
    Serial__hex_put(console, uart1->LCR);
    Serial__character_put(console, ';');
    Serial__hex_put(console, uart1->RS485CTRL);
    Serial__character_put(console, '\n');

    struct Motor3__Struct motor3__struct;
    Motor3 motor3 = &motor3__struct;
    motor3->speed = 0;
    motor3->direction_invert = (Bool8)0;

    motor_init();
    pwm_init();
    qei_init();

    Robus__slave_process(robus, Motor3__process, (void *)motor3);

    // We never get here:
    return 1;
}

// {Motor3} routines:

void Motor3__speed_update(
  Motor3 motor3,
  Serial debug_serial)
{
    Int8 speed = motor3->speed;
    Bool8 direction_invert = motor3->direction_invert;

    Int32 pwm = 0;
    if (debug_serial != (Serial)0) {
	Serial__character_put(debug_serial, ' ');
	Serial__character_put(debug_serial, 'S');
	Serial__hex_byte_put(debug_serial, speed);
	Serial__character_put(debug_serial, 'D');
	Serial__hex_put(debug_serial, direction_invert);
    }

    if (direction_invert) {
	speed = -speed;
    }

    if (speed > 0) {
	motor_forward();
	pwm = (Int32)(speed << 1);
    } else if (speed < 0) {
	motor_reverse();
        pwm = (Int32)((-speed) << 1);
    } else {
	motor_stop();
    }

    if (debug_serial != (Serial)0) {
	Serial__character_put(debug_serial, 'P');
	Serial__hex_put(debug_serial, pwm);
    }

    pwm_update(pwm);
}

UInt8 Motor3__process(
  void *motor3_pointer,
  Robus robus,
  UInt8 command,
  Bool8 execute)
{
    UInt8 errors = 0;
    Motor3 motor3 = (Motor3)motor3_pointer;
    Buffer get_buffer = robus->get_buffer;
    Buffer put_buffer = robus->put_buffer;
    UInt8 remaining = Buffer__remaining(get_buffer);
    Serial debug_serial = robus->debug_serial;
    debug_serial = (Serial)0;

    switch (command) {
      case 0:
	// Speed get:
	if (execute) {
	    Buffer__int8_put(put_buffer, (Int8)motor3->speed);
	}
	break;
      case 1:
	// Speed set:
	if (remaining == 0) {
	    errors = 1;
	} else {
	    Int8 speed = (Int8)Buffer__int8_get(get_buffer);
	    if (execute) {
		motor3->speed = speed;
		Motor3__speed_update(motor3, debug_serial);
	    }
	}
	break;
      case 2:
	// Direction Invert get:
	if (execute) {
	    Buffer__uint8_put(put_buffer, (UInt8)motor3->direction_invert);
	}
	break;
      case 3:
	// Direction Invert set:
	if (remaining == 0) {
	    errors = 1;
	} else {
	    Bool8 direction_invert =
	      (Bool8)(Buffer__uint8_get(get_buffer) != 0);
	    if (execute) {
		motor3->direction_invert = direction_invert;
		Motor3__speed_update(motor3, debug_serial);
	    }
	}
	break;
      case 4:
	// Encoder8 get:
	if (execute) {
	    Buffer__int8_put(put_buffer, (Int8)qei_read());
	}
	break;
      case 5:
	// Encoder8 set:
	if (remaining == 0) {
	    errors = 1;
	} else {
	    Int32 encoder = (Int32)Buffer__int8_get(get_buffer);
	    if (execute) {
		// Do something here:
	    }
	}
	break;
      case 6:
	// Encoder get:
	if (execute) {
	    Buffer__int32_put(put_buffer, qei_read());
	}
	break;
      case 8:
	// Encoder set:
	if (remaining == 0) {
	    errors = 1;
	} else {
	    Int32 encoder = (Int32)Buffer__int32_get(get_buffer);
	    if (execute) {
		// Do something here:
	    }
	}
	break;
      default:
	errors = 1;
	break;
    }

    //Serial__character_put(debug_serial, 'T');
    //Serial__hex_put(debug_serial, motor3->speed);
    //Serial__character_put(debug_serial, 'V');
    //Serial__hex_put(debug_serial, motor3->direction_invert);

    return errors;
}

#ifdef  DEBUG
// This routine is called whenever an check macro fails.
void check_failed(
  UInt8 *file,
  UInt32 line)
{
    // User can add his own implementation to report the file name and
    // line number, example:
    //    printf("Wrong parameters value: file %s on line %d\r\n", file, line);

    // Infinite loop:
    while (1) {
	// do nothing
    }
}
#endif

