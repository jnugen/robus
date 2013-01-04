#include "lpc17xx_pwm.h"
#include "lpc17xx_libcfg.h"
#include "lpc17xx_pinsel.h"

#define EN_PWM_CHN	4

void pwm_init(void)
{
	PWM_TIMERCFG_Type PWMCfgDat;
	PWM_MATCHCFG_Type PWMMatchCfgDat;
	PINSEL_CFG_Type PinCfg;

	/* Initialize PWM peripheral, timer mode
	 * PWM prescale value = 1 (absolute value - tick value) */
	PWMCfgDat.PrescaleOption = PWM_TIMER_PRESCALE_TICKVAL;
	PWMCfgDat.PrescaleValue = 500;
	PWM_Init(LPC_PWM1, PWM_MODE_TIMER, (void *) &PWMCfgDat);

	/* Initialize PWM pin connect for PWM1[4] on P2[3] */
	PinCfg.Funcnum = 1;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = (EN_PWM_CHN - 1);	// hack
	PINSEL_ConfigPin(&PinCfg);

	/* Set match value for PWM match channel 0 = 256, update immediately */
	PWM_MatchUpdate(LPC_PWM1, 0, 256, PWM_MATCH_UPDATE_NOW);

	/* PWM Timer/Counter will be reset when channel 0 matching
	 * no interrupt when match
	 * no stop when match */
	PWMMatchCfgDat.IntOnMatch = DISABLE;
	PWMMatchCfgDat.MatchChannel = 0;
	PWMMatchCfgDat.ResetOnMatch = ENABLE;
	PWMMatchCfgDat.StopOnMatch = DISABLE;
	PWM_ConfigMatch(LPC_PWM1, &PWMMatchCfgDat);

	/* Configure PWM channel edge option for EN_PWM_CHN */
	PWM_ChannelConfig(LPC_PWM1, EN_PWM_CHN, PWM_CHANNEL_SINGLE_EDGE);

	/* Set initial match value for EN_PWM_CHN to disable output change */
	PWM_MatchUpdate(LPC_PWM1, EN_PWM_CHN, 0, PWM_MATCH_UPDATE_NOW);

	/* Configure match option for EN_PWM_CHN*/
	PWMMatchCfgDat.IntOnMatch = DISABLE;
	PWMMatchCfgDat.MatchChannel = EN_PWM_CHN;
	PWMMatchCfgDat.ResetOnMatch = DISABLE;
	PWMMatchCfgDat.StopOnMatch = DISABLE;
	PWM_ConfigMatch(LPC_PWM1, &PWMMatchCfgDat);

	/* Enable PWM Channel Output for EN_PWM_CHN */
	PWM_ChannelCmd(LPC_PWM1, EN_PWM_CHN, ENABLE);

	/* Reset and Start counter */
	PWM_ResetCounter(LPC_PWM1);
	PWM_CounterCmd(LPC_PWM1, ENABLE);

	/* Start PWM now */
	PWM_Cmd(LPC_PWM1, ENABLE);
}

void pwm_update(int value)
{
	PWM_MatchUpdate(LPC_PWM1, EN_PWM_CHN, value, PWM_MATCH_UPDATE_NOW);
}
