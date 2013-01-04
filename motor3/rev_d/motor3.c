#include "led.h"
#include "motor.h"
#include "pwm.h"
#include "ssp.h"
#include "uart.h"
#include "lpc17xx_libcfg.h"

#include "system_LPC17xx.h"
#include "lpc17xx_clkpwr.h"

char* itoa(int val, int base)
{
    static char buf[32] = {0};
    int i = 30;

    buf[31] = 0;
    for(; val && i ; --i, val /= base) {
	buf[i] = "0123456789abcdef"[val % base];
    }

    return &buf[i+1];
}

void check_encoder(void)
{
    int val;
    //char buffer[10];

    val = (int)ssp_read();
    //itoa(val & 0x3f, &buffer, 2);
    //uart_send(&buffer);

    //uart_send(itoa(val & 0x3f, 2));
    uart_send(itoa((val & 0x4000) >> 14, 2));
    uart_send("  ");

    //itoa(val >> 6, &buffer, 10);
    //uart_send(&buffer);

    //uart_send(itoa(val >> 6, 10));
    uart_send(itoa(val & 0x3FFF, 10));
    uart_send("\r\n");
}

int c_entry(void)
{
    int i;
    float test;

    // Adjust SystemCoreClock global according to clock registers
    SystemCoreClockUpdate();

    led_init();
    motor_init();
    pwm_init();
    ssp_init();
    uart_init();

    uart_send("SystemCoreClock = ");
    uart_send(itoa(SystemCoreClock, 10));
    uart_send("\r\n");

    uart_send("ssp_clk = ");
    uart_send(itoa(CLKPWR_GetPCLK(CLKPWR_PCLKSEL_SSP0), 10));
    uart_send("\r\n");

    while (1) {
	// Do nothing:
    }
 
    while (1) {
	// led_on();
	motor_forward();
	for (i=0; i<=256; i+=16) {
	    pwm_update(i);
	    check_encoder();
	    Delay(250);
	}

	// led_off();
	motor_stop();
	pwm_update(0);
	Delay(1000);

	// led_on();
	motor_reverse();
	for (i=0; i<=256; i+=16) {
	    pwm_update(i);
	    check_encoder();
	    Delay(250);
	}

	// led_off();
	motor_stop();
	pwm_update(0);
	Delay(1000);
    }
}

// With ARM and GHS toolsets, the entry point is main() - this will
// allow the linker to generate wrapper code to setup stacks, allocate
// heap area, and initialize and copy code and data segments. For GNU
// toolsets, the entry point is through __start() in the crt0_gnu.asm
// file, and that startup code will setup stacks and data
int main(void)
{
    return c_entry();
}


#ifdef DEBUG
void check_failed(uint8_t *file, uint32_t line)
{
    // User can add his own implementation to report the file name and
    // line number,
    // ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line)

    // Infinite loop
    while(1) {
	// Do nothing:
    }
}
#endif // DEBUG

