The following edits have performed:

In .../Core/CM3/DeviceSupport/NXP/LPC17xx/LPC17xxh:

1) Add LPC_UART__Struct, LPC_UART0__Struct, and LPC_UART1__Struct
   to UART type definitions.  These structure definitions are used
   byte $(PROJECT_ROOT)/common/uart.h .

2) In .../Core/CM3/DeviceSupport/NXP/LPC17xx, change the following:

    Change 1:
	#define SCS_Val               0x00000000
	#define CLKSRCSEL_Val         0x00000000
	#define PLL0_SETUP            0
	#define PLL0CFG_Val           0x00050063
		=>
	#define SCS_Val               0x00000030
	#define CLKSRCSEL_Val         0x00000001
	#define PLL0_SETUP            1
	#define PLL0CFG_Val           0x00000009

    Change 2:
	#define PCLKSEL1_Val          0x00000000
		=>
	#define PCLKSEL1_Val          (3<<10)|(1<<0)

    Change 3:
	#define FLASH_SETUP 0
		=>
	#define FLASH_SETUP 1

    Change 4:
	#define XTAL (12000000UL)
	    =>
	#define XTAL (20000000UL)

3) In cmsis/lpc17xx/Drivers/source/lpc17xx_qei.c:

   In QEI_Init():
   Change 1:
	Commented out the call to CLKPWR_SetPCLKDiv

   Change 2:
	Commented out the QEI_CON_RESI term for QEIx->QEICON

   Change 3:
	Changed QEIx->QEIMAXPOS value from 0x00 to 0xFFFFFFFF

