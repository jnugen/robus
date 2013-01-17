/**********************************************************************
* $Id$		pwm_single_edge.c 				2010-05-21
*//**
* @file		pwm_single_edge.c
* @brief	This program illustrates the PWM signal on 6 Channels
*           in single edge mode
* @version	2.0
* @date		21. May. 2010
* @author	NXP MCU SW Application Team
*
* Copyright(C) 2010, NXP Semiconductor
* All rights reserved.
*
***********************************************************************
* Software that is described herein is for illustrative purposes only
* which provides customers with programming information regarding the
* products. This software is supplied "AS IS" without any warranties.
* NXP Semiconductors assumes no responsibility or liability for the
* use of the software, conveys no license or title under any patent,
* copyright, or mask work right to the product. NXP Semiconductors
* reserves the right to make changes in the software without
* notification. NXP Semiconductors also make no representation or
* warranty that such application will be suitable for the specified
* use without further testing or modification.
**********************************************************************/
#include "common.h"
#include "led.h"
#include "uart-local.h"
#include "lpc17xx_libcfg.h"

#include "system_LPC17xx.h"
#include "lpc17xx_clkpwr.h"

/* Example group ----------------------------------------------------------- */
/** @defgroup PWM_Single_Edge	Single_Edge
 * @ingroup PWM_Examples
 * @{
 */

/**
 * Ansi C "itoa" based on Kernighan & Ritchie's "Ansi C":
 */
	
/*void strreverse(char* begin, char* end)
{
	char aux;
	
	while(end>begin)
		aux=*end, *end--=*begin, *begin++=aux;
}
	
void itoa(int value, char* str, int base)
{
	static char num[] = "0123456789abcdefghijklmnopqrstuvwxyz";
	char* wstr=str;
	int sign;

	// Validate base
	if (base<2 || base>35){ *wstr='\0'; return; }

	// Take care of sign
	if ((sign=value) < 0) value = -value;

	// Conversion. Number is reversed.
	do *wstr++ = num[value%base]; while(value/=base);

	if(sign<0) *wstr++='-';

	*wstr='\0';

	// Reverse string
	strreverse(str,wstr-1);
}*/

char* itoa(int val, int base)
{
	static char buf[32] = {0};
	int i = 30;

	buf[31] = 0;
	for(; val && i ; --i, val /= base)
		buf[i] = "0123456789abcdef"[val % base];

	return &buf[i+1];
}

/*-------------------------MAIN FUNCTION------------------------------*/
/*********************************************************************//**
 * @brief		c_entry: Main PWM program body
 * @param[in]	None
 * @return 		int
 **********************************************************************/
int c_entry(void)
{
	int i;
	float test;

	// Adjust SystemCoreClock global according to clock registers
	SystemCoreClockUpdate();

	led_init();
	uart0_init();
	uart3_init();

	uart_send("SystemCoreClock = ");
	uart_send(itoa(SystemCoreClock, 10));
	uart_send("\r\n");

	while (1)
	{
		led_green();
		SysTick__delay(1000);

		led_red();
		SysTick__delay(1000);

		//led_off();
		//Delay(1000);
	}
}

/* With ARM and GHS toolsets, the entry point is main() - this will
   allow the linker to generate wrapper code to setup stacks, allocate
   heap area, and initialize and copy code and data segments. For GNU
   toolsets, the entry point is through __start() in the crt0_gnu.asm
   file, and that startup code will setup stacks and data */
int main(void)
{
    return c_entry();
}


#ifdef  DEBUG
/*******************************************************************************
* @brief		Reports the name of the source file and the source line number
* 				where the CHECK_PARAM error has occurred.
* @param[in]	file Pointer to the source file name
* @param[in]    line assert_param error line source number
* @return		None
*******************************************************************************/
void check_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while(1);
}
#endif

/*
 * @}
 */
