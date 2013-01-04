#include "lpc17xx_ssp.h"
#include "lpc17xx_libcfg.h"
#include "lpc17xx_pinsel.h"

/************************** PRIVATE DEFINTIONS ************************/
#define SSEL0_PORT 0
#define SSEL0_PIN 16
#define SSP0_PORT 1
#define SCK0_PIN 20
#define MISO0_PIN 23
#define MOSI0_PIN 24

#define AS5055_CMD_NOP 0x0000
#define AS5055_CMD_READ_ANGLE 0xFFFF

/************************** PRIVATE VARIABLES *************************/

/************************** PRIVATE FUNCTIONS *************************/

/*-------------------------PRIVATE FUNCTIONS------------------------------*/

/*-------------------------MAIN FUNCTION------------------------------*/
/*********************************************************************//**
 * @brief		c_entry: Main SSP program body
 * @param[in]	None
 * @return 		int
 **********************************************************************/
int ssp_init(void)
{
	PINSEL_CFG_Type PinCfg;
	SSP_CFG_Type SSP_ConfigStruct;

	/*
	 * Initialize SPI pin connect
	 * P0.16 - SSEL0
	 * P1.20 - SCK0
	 * P1.23 - MISO0
	 * P1.24 - MOSI0
	 * P1.30 - (connected to P1.20)
	 */
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
	PinCfg.Funcnum = PINSEL_FUNC_2;
	PinCfg.Portnum = SSEL0_PORT;
	PinCfg.Pinnum = SSEL0_PIN;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Funcnum = PINSEL_FUNC_3;
	PinCfg.Portnum = SSP0_PORT;
	PinCfg.Pinnum = SCK0_PIN;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = MISO0_PIN;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = MOSI0_PIN;
	PINSEL_ConfigPin(&PinCfg);

	// initialize SSP configuration structure to default
	SSP_ConfigStructInit(&SSP_ConfigStruct);
	SSP_ConfigStruct.CPHA = SSP_CPHA_SECOND;
	//SSP_ConfigStruct.CPOL = SSP_CPOL_HI;
	SSP_ConfigStruct.ClockRate = 400000;
	SSP_ConfigStruct.Databit = SSP_DATABIT_16;
	//SSP_ConfigStruct.Mode = SSP_MASTER_MODE;
	//SSP_ConfigStruct.FrameFormat = SSP_FRAME_SPI;

	// Initialize SSP peripheral with parameter given in structure above
	SSP_Init(LPC_SSP0, &SSP_ConfigStruct);

	// Enable SSP peripheral
	SSP_Cmd(LPC_SSP0, ENABLE);
}

int ssp_read(void)
{
	uint16_t val;

	SSP_SendData(LPC_SSP0, AS5055_CMD_READ_ANGLE);
	while (SSP_GetStatus(LPC_SSP0, SSP_STAT_BUSY));
	val = SSP_ReceiveData(LPC_SSP0);
	// first read is junk

	SSP_SendData(LPC_SSP0, AS5055_CMD_NOP);
	while (SSP_GetStatus(LPC_SSP0, SSP_STAT_BUSY));
	val = SSP_ReceiveData(LPC_SSP0);

	return (int)val;
}
