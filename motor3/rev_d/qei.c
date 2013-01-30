#include "lpc17xx_qei.h"
#include "lpc17xx_libcfg.h"
//#include "lpc17xx_timer.h"
#include "lpc17xx_clkpwr.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"


/* Example group ----------------------------------------------------------- */
/** @defgroup QEI_Velo  QEI_Velo
 * @ingroup QEI_Examples
 * @{
 */

/************************** PRIVATE DEFINITIONS *************************/
/** Velocity capture period definition (in microsecond) */
#define CAP_PERIOD          250000UL

/** Delay time to Read Velocity Accumulator and display (in microsecond)*/
#define DISP_TIME           3000000UL
/** Max velocity capture times calculated */
#define MAX_CAP_TIMES       (DISP_TIME/CAP_PERIOD)

#define ENC_RES             2048UL  /**< Encoder resolution (PPR) */


/************************** PRIVATE VARIABLES *************************/
/** Velocity Accumulator */
__IO uint64_t VeloAcc;
/** Times of Velocity capture */
__IO uint32_t VeloCapCnt;
/** Flag indicates Times of Velocity capture is enough to read out */
__IO FlagStatus VeloAccFlag;


/* Pin Configuration selection must be defined in structure following:
 * - Port Number,
 * - Pin Number,
 * - Function Number,
 * - Pin Mode,
 * - Open Drain
 */

/** QEI Phase-A Pin */
const PINSEL_CFG_Type qei_phaA_pin[1] = {{1, 20, 1, 0, 0}};
/** QEI Phase-B Pin */
const PINSEL_CFG_Type qei_phaB_pin[1] = {{1, 23, 1, 0, 0}};
/** QEI Index Pin */
const PINSEL_CFG_Type qei_idx_pin[1] = {{1, 24, 1, 0, 0}};


/************************** PRIVATE FUNCTIONS *************************/
void QEI_IRQHandler(void);


/*----------------- INTERRUPT SERVICE ROUTINES --------------------------*/
/*********************************************************************//**
 * @brief       QEI interrupt handler. This sub-routine will update current
 *              value of captured velocity in to velocity accumulate.
 * @param[in]   None
 * @return      None
 **********************************************************************/
void QEI_IRQHandler(void)
{
    // Check whether if velocity timer overflow
    if (QEI_GetIntStatus(LPC_QEI, QEI_INTFLAG_TIM_Int) == SET) {
        if (VeloAccFlag == RESET) {

            // Get current velocity captured and update to accumulate
            VeloAcc += QEI_GetVelocityCap(LPC_QEI);

            // Update Velocity capture times
            VeloAccFlag = ((VeloCapCnt++) >= MAX_CAP_TIMES) ? SET : RESET;
        }
        // Reset Interrupt flag pending
        QEI_IntClear(LPC_QEI, QEI_INTFLAG_TIM_Int);
    }

    // Check whether if direction change occurred
    if (QEI_GetIntStatus(LPC_QEI, QEI_INTFLAG_DIR_Int) == SET) {
        // Print direction status
//        _DBG("Direction has changed: ");
//        _DBG_((QEI_GetStatus(LPC_QEI, QEI_STATUS_DIR) == SET) ? "1" : "0");
        // Reset Interrupt flag pending
        QEI_IntClear(LPC_QEI, QEI_INTFLAG_DIR_Int);
    }
}

/*-------------------------PRIVATE FUNCTIONS------------------------------*/

void qei_init(void)
{
    PINSEL_CFG_Type PinCfg;
    QEI_CFG_Type QEIConfig;
    QEI_RELOADCFG_Type ReloadConfig;

    /* Set QEI function pin
     * P1.20: MCI0 phase A
     * P1.23: MCI1 phase B
     * P1.24: MCI2 index
     */
    PinCfg.Funcnum = PINSEL_FUNC_1;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PinCfg.Portnum = 1;
    PinCfg.Pinnum = 20;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 23;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 24;
    PINSEL_ConfigPin(&PinCfg);

    // Initialize QEI configuration structure to default value
    QEIConfig.CaptureMode = QEI_CAPMODE_4X;
    QEIConfig.DirectionInvert = QEI_DIRINV_NONE;
    QEIConfig.InvertIndex = QEI_INVINX_NONE;
    QEIConfig.SignalMode = QEI_SIGNALMODE_QUAD;

    // Initialize QEI peripheral with given configuration structure
    QEI_Init(LPC_QEI, &QEIConfig);

    // Set timer reload value for  QEI that used to set velocity capture period
    ReloadConfig.ReloadOption = QEI_TIMERRELOAD_USVAL;
    ReloadConfig.ReloadValue = CAP_PERIOD;
    QEI_SetTimerReload(LPC_QEI, &ReloadConfig);

    /* preemption = 1, sub-priority = 1 */
    NVIC_SetPriority(QEI_IRQn, ((0x01<<3)|0x01));
    /* Enable interrupt for QEI  */
    NVIC_EnableIRQ(QEI_IRQn);

    // Reset VeloAccFlag
    VeloAccFlag = RESET;
    // Reset value of Acc and Acc count to default
    VeloAcc = 0;
    VeloCapCnt = 0;

    // Enable interrupt for velocity Timer overflow for capture velocity into Acc
//    QEI_IntCmd(LPC_QEI, QEI_INTFLAG_TIM_Int, ENABLE);
    // Enable interrupt for direction change */
//    QEI_IntCmd(LPC_QEI, QEI_INTFLAG_DIR_Int, ENABLE);
}

uint32_t qei_read(void)
{
	return QEI_GetPosition(LPC_QEI);
}

uint32_t qei_getrpm(void)
{
    uint32_t rpm, averageVelo;

    // Check VeloAccFlag continuously
    if (VeloAccFlag == SET) {
        // Get Acc
        averageVelo = (uint32_t)(VeloAcc / VeloCapCnt);
        rpm = QEI_CalculateRPM(LPC_QEI, averageVelo, ENC_RES);
        // Disp the result
//        Serial__string_put("Sampling Speed: ");
//        Serial__hex_put(rpm);
//        Serial__string_put("RPM\n");
        // Reset VeloAccFlag
        VeloAccFlag = RESET;
        // Reset value of Acc and Acc count to default
        VeloAcc = 0;
        VeloCapCnt = 0;
    }

    return rpm;
}
