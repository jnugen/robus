#include "lpc17xx_gpio.h"
#include "lpc17xx_libcfg.h"

#define LED_PIN	(1<<10)

/************************** PRIVATE VARIABLES *************************/
/* SysTick Counter */
volatile unsigned long SysTickCnt;

/************************** PRIVATE FUNCTIONS *************************/
void SysTick_Handler (void);

void Delay (unsigned long tick);

/*----------------- INTERRUPT SERVICE ROUTINES --------------------------*/
/*********************************************************************//**
 * @brief		SysTick handler sub-routine (1ms)
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void SysTick_Handler (void) {
  SysTickCnt++;
}

/*-------------------------PRIVATE FUNCTIONS------------------------------*/
/*********************************************************************//**
 * @brief		Delay function
 * @param[in]	tick - number milisecond of delay time
 * @return 		None
 **********************************************************************/
void Delay (unsigned long tick) {
  unsigned long systickcnt;

  systickcnt = SysTickCnt;
  while ((SysTickCnt - systickcnt) < tick);
}

void led_on(void)
{
    GPIO_ClearValue(2, LED_PIN);
}

void led_off(void)
{
    GPIO_SetValue(2, LED_PIN);
}

int led_init (void)
{
  /* Adjust SystemCoreClock global according to clock registers */
  //SystemCoreClockUpdate();

  /* Generate interrupt each 1 ms   */
  SysTick_Config(SystemCoreClock/1000 - 1);

  GPIO_SetDir(2, LED_PIN, 1);

  led_off();
}
