#!/bin/sh

# ARM tools definitions:
#ARM_TOOLS_ROOT := /home/wayne/download/codesourcery/arm-2010q1
ARM_TOOLS_BIN := $(ARM_TOOLS_ROOT)/bin
ARM_TOOLS_BIN_PREFIX := $(ARM_TOOLS_BIN)/arm-none-eabi
CC := $(ARM_TOOLS_BIN_PREFIX)-gcc
AS := $(ARM_TOOLS_BIN_PREFIX)-as
OBJCOPY := $(ARM_TOOLS_BIN_PREFIX)-objcopy
SIZE := $(ARM_TOOLS_BIN_PREFIX)-size

CMSIS := cmsis
CMSIS_CORE := $(CMSIS)/core
CMSIS_DRIVERS := $(CMSIS)/drivers

OBJECT_FILES := \
    led.o \
    motor3.o \
    motor.o \
    pwm.o \
    ssp.o \
    uart.o \
    $(CMSIS_CORE)/core_cm3.o \
    $(CMSIS_CORE)/system_LPC17xx.o \
    $(CMSIS_CORE)/startup_LPC17xx.o \
    $(CMSIS_DRIVERS)/src/lpc17xx_clkpwr.o \
    $(CMSIS_DRIVERS)/src/lpc17xx_gpio.o \
    $(CMSIS_DRIVERS)/src/lpc17xx_pinsel.o \
    $(CMSIS_DRIVERS)/src/lpc17xx_pwm.o \
    $(CMSIS_DRIVERS)/src/lpc17xx_ssp.o \
    $(CMSIS_DRIVERS)/src/lpc17xx_uart.o

INCLUDES := \
    -I$(CMSIS_DRIVERS)/include \
    -I$(CMSIS_CORE)/include \
    -I.

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
    -D__RAM_MODE__=0 \
    ${INCLUDES}

# Linker flags:
LIBRARIES := -lc -lg -lstdc++ -lsupc++  -lgcc -lm
LIBRARIES_GROUP := \
    -Wl,--start-group \
    -L$(ARM_TOOLS_ROOT)/lib/gcc/arm-none-eabi/4.6.3/thumb2 \
    -L$(ARM_TOOLS_ROOT)/arm-none-eabi/lib/thumb2 \
    ${LIBRARIES} \
    -Wl,--end-group
LD_FLAGS := \
    -static \
    -mcpu=cortex-m3 \
    -mthumb \
    -mthumb-interwork
MAP_FLAGS := \
    -Xlinker -Map \
    -Xlinker motor3.map \
    -Xlinker -T \
    ldscript_rom_gnu.ld    

all: motor3.hex

motor3.hex: motor3.elf
	$(OBJCOPY) -O ihex  motor3.elf motor3.hex

motor3.elf: ${OBJECT_FILES}
	$(CC) ${OBJECT_FILES} ${LD_FLAGS} ${LIBRARIES_GROUP} ${MAP_FLAGS} \
	  -o motor3.elf
	$(SIZE) motor3.elf

clean:
	rm -f ${OBJECT_FILES}
	rm -f motor3.elf motor3.hex motor3.srec motor3.map

led.o: led.c
	$(CC) -c ${CFLAGS} led.c -o led.o

motor3.o: motor3.c
	$(CC) -c ${CFLAGS} motor3.c -o motor3.o

motor.o: motor.c
	$(CC) -c ${CFLAGS} motor.c -o motor.o

pwm.o: pwm.c
	$(CC) -c ${CFLAGS} pwm.c -o pwm.o

ssp.o: ssp.c
	$(CC) -c ${CFLAGS} ssp.c -o ssp.o

uart.o: uart.c
	$(CC) -c ${CFLAGS} uart.c -o uart.o

$(CMSIS_CORE)/core_cm3.o: $(CMSIS_CORE)/core_cm3.c
	$(CC) -c ${CFLAGS} $(CMSIS_CORE)/core_cm3.c -o $(CMSIS_CORE)/core_cm3.o

$(CMSIS_CORE)/system_LPC17xx.o: $(CMSIS_CORE)/system_LPC17xx.c
	$(CC) -c ${CFLAGS} $(CMSIS_CORE)/system_LPC17xx.c -o $(CMSIS_CORE)/system_LPC17xx.o

$(CMSIS_CORE)/startup_LPC17xx.o: $(CMSIS_CORE)/startup_LPC17xx.s
	$(AS) -mcpu=cortex-m3 -gdwarf-2  -gdwarf-2 --defsym RAM_MODE=0  $(CMSIS_CORE)/startup_LPC17xx.s -o $(CMSIS_CORE)/startup_LPC17xx.o

$(CMSIS_DRIVERS)/src/lpc17xx_clkpwr.o: $(CMSIS_DRIVERS)/src/lpc17xx_clkpwr.c
	$(CC) -c ${CFLAGS} $(CMSIS_DRIVERS)/src/lpc17xx_clkpwr.c -o $(CMSIS_DRIVERS)/src/lpc17xx_clkpwr.o

$(CMSIS_DRIVERS)/src/lpc17xx_gpio.o: $(CMSIS_DRIVERS)/src/lpc17xx_gpio.c
	$(CC) -c ${CFLAGS} $(CMSIS_DRIVERS)/src/lpc17xx_gpio.c -o $(CMSIS_DRIVERS)/src/lpc17xx_gpio.o

$(CMSIS_DRIVERS)/src/lpc17xx_pinsel.o: $(CMSIS_DRIVERS)/src/lpc17xx_pinsel.c
	$(CC) -c ${CFLAGS} $(CMSIS_DRIVERS)/src/lpc17xx_pinsel.c -o $(CMSIS_DRIVERS)/src/lpc17xx_pinsel.o

$(CMSIS_DRIVERS)/src/lpc17xx_pwm.o: $(CMSIS_DRIVERS)/src/lpc17xx_pwm.c
	$(CC) -c ${CFLAGS} $(CMSIS_DRIVERS)/src/lpc17xx_pwm.c -o $(CMSIS_DRIVERS)/src/lpc17xx_pwm.o

$(CMSIS_DRIVERS)/src/lpc17xx_ssp.o: $(CMSIS_DRIVERS)/src/lpc17xx_ssp.c
	$(CC) -c ${CFLAGS} $(CMSIS_DRIVERS)/src/lpc17xx_ssp.c -o $(CMSIS_DRIVERS)/src/lpc17xx_ssp.o

$(CMSIS_DRIVERS)/src/lpc17xx_uart.o: $(CMSIS_DRIVERS)/src/lpc17xx_uart.c
	$(CC) -c ${CFLAGS} $(CMSIS_DRIVERS)/src/lpc17xx_uart.c -o $(CMSIS_DRIVERS)/src/lpc17xx_uart.o

