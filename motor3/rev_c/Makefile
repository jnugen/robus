ARM_TOOLS_BIN := $(ARM_TOOLS_ROOT)/bin
CC := $(ARM_TOOLS_BIN)/arm-none-eabi-gcc
AS := $(ARM_TOOLS_BIN)/arm-none-eabi-as
OBJCOPY := $(ARM_TOOLS_BIN)/arm-none-eabi-objcopy
SIZE := $(ARM_TOOLS_BIN)/arm-none-eabi-size
COMMON := ../../common

CMSIS := ../../cmsis
INCLUDES := \
    -I$(CMSIS)/drivers/include \
    -I$(CMSIS)/core/include \
    -I.

AS_FLAGS := \
    -mcpu=cortex-m3 \
    -gdwarf-2 \
    --defsym RAM_MODE=0

CFLAGS := \
    -mcpu=cortex-m3 \
    -mthumb \
    -Wall \
    -O0 \
    -mapcs-frame \
    -D__thumb2__=1 \
    -msoft-float \
    -gdwarf-2 \
    -mno-sched-prolog \
    -fno-hosted \
    -mtune=cortex-m3 \
    -march=armv7-m \
    -mfix-cortex-m3-ldrd \
    -ffunction-sections \
    -fdata-sections \
    -D__BUILD_WITH_EXAMPLE__=1 \
    -g \
    -D__RAM_MODE__=0

CMSIS_OBJECTS := \
    core_cm3.o \
    system_LPC17xx.o \
    startup_LPC17xx.o \
    lpc17xx_clkpwr.o \
    lpc17xx_gpio.o \
    lpc17xx_pinsel.o \
    lpc17xx_pwm.o \
    lpc17xx_uart.o

#    lpc17xx_nvic.o \

LD_SCRIPT := ../../common/ldscript_rom_gnu.ld
#LD_SCRIPT := ldscript_rom_gnu.ld

OBJECTS := \
    motor3.o \
    motor.o \
    pwm.o \
    ${CMSIS_OBJECTS}
LINK_FLAGS := \
    -static \
    -mcpu=cortex-m3 \
    -mthumb \
    -mthumb-interwork \
    -Wl,--start-group \
    -L$(ARM_TOOLS_ROOT)/lib/gcc/arm-none-eabi/4.4.1/thumb2 \
    -L$(ARM_TOOLS_ROOT)/arm-none-eabi/lib/thumb2 \
    -lc -lg -lstdc++ -lsupc++ -lgcc -lm \
    -Wl,--end-group \
    -Xlinker -Map \
    -Xlinker motor3.map \
    -Xlinker -T $(LD_SCRIPT)

all: motor3.hex

core_cm3.o: $(CMSIS)/core/core_cm3.c
	$(CC) -c ${CFLAGS} ${INCLUDES} $(CMSIS)/core/core_cm3.c -o core_cm3.o

system_LPC17xx.o: $(CMSIS)/core/system_LPC17xx.c
	$(CC) -c ${CFLAGS} ${INCLUDES} $(CMSIS)/core/system_LPC17xx.c -o system_LPC17xx.o

startup_LPC17xx.o: $(CMSIS)/core/startup_LPC17xx.s
	$(AS) ${AS_FLAGS} ${INCLUDES} $(CMSIS)/core/startup_LPC17xx.s -o startup_LPC17xx.o


lpc17xx_clkpwr.o: $(CMSIS)/drivers/src/lpc17xx_clkpwr.c
	$(CC) -c ${CFLAGS} ${INCLUDES} $(CMSIS)/drivers/src/lpc17xx_clkpwr.c -o lpc17xx_clkpwr.o

lpc17xx_gpio.o: $(CMSIS)/drivers/src/lpc17xx_gpio.c
	$(CC) -c ${CFLAGS} ${INCLUDES} $(CMSIS)/drivers/src/lpc17xx_gpio.c -o lpc17xx_gpio.o

lpc17xx_nvic.o: $(CMSIS)/drivers/src/lpc17xx_nvic.c
	$(CC) -c ${CFLAGS} ${INCLUDES} $(CMSIS)/drivers/src/lpc17xx_nvic.c -o lpc17xx_nvic.o

lpc17xx_pinsel.o: $(CMSIS)/drivers/src/lpc17xx_pinsel.c
	$(CC) -c ${CFLAGS} ${INCLUDES} $(CMSIS)/drivers/src/lpc17xx_pinsel.c -o lpc17xx_pinsel.o

lpc17xx_pwm.o: $(CMSIS)/drivers/src/lpc17xx_pwm.c
	$(CC) -c ${CFLAGS} ${INCLUDES} $(CMSIS)/drivers/src/lpc17xx_pwm.c -o lpc17xx_pwm.o

lpc17xx_uart.o: $(CMSIS)/drivers/src/lpc17xx_uart.c
	$(CC) -c ${CFLAGS} ${INCLUDES} $(CMSIS)/drivers/src/lpc17xx_uart.c -o lpc17xx_uart.o

motor3.o: motor3.c
	$(CC) -c ${CFLAGS} ${INCLUDES} motor3.c -o motor3.o

motor.o: motor.c
	$(CC) -c ${CFLAGS} ${INCLUDES} motor.c -o motor.o

pwm.o: pwm.c
	$(CC) -c ${CFLAGS} ${INCLUDES} pwm.c -o pwm.o

motor3.elf: ${OBJECTS}
	$(CC) ${OBJECTS} ${LINK_FLAGS} -o motor3.elf

motor3.hex: motor3.elf
	$(OBJCOPY) -O ihex  motor3.elf motor3.hex
	$(SIZE) motor3.elf motor3.elf motor3.hex

clean:
	rm -f ${OBJECTS} motor3.elf motor3.hex motor3.map


###### OLD stuff to be deleted:

#CMSIS := ../../cmsis_1.30
#INCLUDES := \
#    -I$(CMSIS)/Drivers/include \
#    -I$(CMSIS)/Core/CM3/CoreSupport \
#    -I$(CMSIS)/Core/CM3/DeviceSupport/NXP/LPC17xx \
#    -I.

#core_cm3.o: core_cm3_cmsis13.c
#	$(CC) -c ${CFLAGS} ${INCLUDES} core_cm3_cmsis13.c -o core_cm3.o

#system_LPC17xx.o: system_LPC17xx_cmsis13.c
#	$(CC) -c ${CFLAGS} ${INCLUDES} system_LPC17xx_cmsis13.c -o system_LPC17xx.o

#startup_LPC17xx.o: startup_LPC17xx_cmsis13.s
#	$(AS) ${AS_FLAGS} ${INCLUDES} startup_LPC17xx_cmsis13.s -o startup_LPC17xx.o


#lpc17xx_clkpwr.o: $(CMSIS)/Drivers/source/lpc17xx_clkpwr.c
#	$(CC) -c ${CFLAGS} ${INCLUDES} $(CMSIS)/Drivers/source/lpc17xx_clkpwr.c -o lpc17xx_clkpwr.o

#lpc17xx_gpio.o: $(CMSIS)/Drivers/source/lpc17xx_gpio.c
#	$(CC) -c ${CFLAGS} ${INCLUDES} $(CMSIS)/Drivers/source/lpc17xx_gpio.c -o lpc17xx_gpio.o

#lpc17xx_nvic.o: $(CMSIS)/Drivers/source/lpc17xx_nvic.c
#	$(CC) -c ${CFLAGS} ${INCLUDES} $(CMSIS)/Drivers/source/lpc17xx_nvic.c -o lpc17xx_nvic.o

#lpc17xx_pinsel.o: $(CMSIS)/Drivers/source/lpc17xx_pinsel.c
#	$(CC) -c ${CFLAGS} ${INCLUDES} $(CMSIS)/Drivers/source/lpc17xx_pinsel.c -o lpc17xx_pinsel.o

#lpc17xx_pwm.o: $(CMSIS)/Drivers/source/lpc17xx_pwm.c
#	$(CC) -c ${CFLAGS} ${INCLUDES} $(CMSIS)/Drivers/source/lpc17xx_pwm.c -o lpc17xx_pwm.o

#lpc17xx_uart.o: $(CMSIS)/Drivers/source/lpc17xx_uart.c
#	$(CC) -c ${CFLAGS} ${INCLUDES} $(CMSIS)/Drivers/source/lpc17xx_uart.c -o lpc17xx_uart.o

