#include "lpc17xx_gpio.h"
#include "lpc17xx_libcfg.h"

#define LED_PORT	0
#define LED_A_PIN	(1<<22)
#define LED_C_PIN	(1<<16)

/************************** PRIVATE VARIABLES *************************/

/************************** PRIVATE FUNCTIONS *************************/

void led_green(void)
{
    GPIO_ClearValue(LED_PORT, LED_A_PIN);
    GPIO_SetValue(LED_PORT, LED_C_PIN);

  GPIO_SetValue(0, (1<<0));
//  GPIO_SetValue(0, (1<<1));
//  GPIO_SetValue(0, (1<<11));
  GPIO_SetValue(2, (1<<4));
}

void led_red(void)
{
    GPIO_ClearValue(LED_PORT, LED_C_PIN);
    GPIO_SetValue(LED_PORT, LED_A_PIN);

  GPIO_ClearValue(0, (1<<0));
//  GPIO_ClearValue(0, (1<<1));
//  GPIO_ClearValue(0, (1<<11));
  GPIO_ClearValue(2, (1<<4));
}

void led_off(void)
{
    GPIO_ClearValue(LED_PORT, LED_A_PIN);
    GPIO_ClearValue(LED_PORT, LED_C_PIN);

  GPIO_ClearValue(0, (1<<0));
//  GPIO_ClearValue(0, (1<<1));
//  GPIO_ClearValue(0, (1<<11));
  GPIO_ClearValue(2, (1<<4));
}

int led_init (void)
{
  GPIO_SetDir(LED_PORT, LED_A_PIN, 1);
  GPIO_SetDir(LED_PORT, LED_C_PIN, 1);

  GPIO_SetDir(0, (1<<0), 1);
  GPIO_SetDir(0, (1<<1), 0);
//  GPIO_SetDir(0, (1<<11), 1);
  GPIO_SetDir(2, (1<<4), 1);

  led_off();
}
