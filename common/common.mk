# Copyright (c) 2013 by IMC.  All rights reserved.
#
# This make file fragment is included from another makfile as follows:
#
#    PROJECT_ROOT := ../..             # As appropriate
#    include $(PROJECT_ROOT)/common/common.mk
#    
# makefile fragment requires that both $(PROJECT_ROOT) and $(ARM_TOOLS_ROOT)
# be defined prior being included.  It is expected that $(ARM_TOOLS_ROOT)
# will be picked via an environment variable that is defined in .bashrc:
#
#    export ARM_TOOLS_ROOT = ...

# Various locations in the ARM tools tree:
ARM_TOOLS_BIN := $(ARM_TOOLS_ROOT)/bin
CC := $(ARM_TOOLS_BIN)/arm-none-eabi-gcc
AS := $(ARM_TOOLS_BIN)/arm-none-eabi-as
OBJCOPY := $(ARM_TOOLS_BIN)/arm-none-eabi-objcopy
SIZE := $(ARM_TOOLS_BIN)/arm-none-eabi-size

# CMSIS is a vendor supplied library of code that can be used
# to access processor peripherals.
#
# CMSIS library locations:
CMSIS := $(PROJECT_ROOT)/cmsis

COMMON := $(PROJECT_ROOT)/common

# Compiler -I flags:
INCLUDES := \
    -I$(CMSIS)/drivers/include \
    -I$(CMSIS)/core/include \
    -I$(COMMON) \
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

COMMON_OBJECTS := \
    buffer.o \
    ring_buffer.o \
    robus.o \
    serial.o \
    systick.o \
    trace.o \
    uart.o

#    lpc17xx_nvic.o \

LD_SCRIPT := $(COMMON)/ldscript_rom_gnu.ld

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

# Common core object files:
core_cm3.o: $(CMSIS)/core/core_cm3.c
	$(CC) -c ${CFLAGS} ${INCLUDES} $(CMSIS)/core/core_cm3.c -o $@

system_LPC17xx.o: $(CMSIS)/core/system_LPC17xx.c
	$(CC) -c ${CFLAGS} ${INCLUDES} $(CMSIS)/core/system_LPC17xx.c -o $@

startup_LPC17xx.o: $(CMSIS)/core/startup_LPC17xx.s
	$(AS) ${AS_FLAGS} ${INCLUDES} $(CMSIS)/core/startup_LPC17xx.s -o $@

#CMSIS 2.0 object files:
lpc17xx_clkpwr.o: $(CMSIS)/drivers/src/lpc17xx_clkpwr.c
	$(CC) -c ${CFLAGS} ${INCLUDES} $(CMSIS)/drivers/src/lpc17xx_clkpwr.c -o $@

lpc17xx_gpio.o: $(CMSIS)/drivers/src/lpc17xx_gpio.c
	$(CC) -c ${CFLAGS} ${INCLUDES} $(CMSIS)/drivers/src/lpc17xx_gpio.c -o $@

lpc17xx_nvic.o: $(CMSIS)/drivers/src/lpc17xx_nvic.c
	$(CC) -c ${CFLAGS} ${INCLUDES} $(CMSIS)/drivers/src/lpc17xx_nvic.c -o $@

lpc17xx_pinsel.o: $(CMSIS)/drivers/src/lpc17xx_pinsel.c
	$(CC) -c ${CFLAGS} ${INCLUDES} $(CMSIS)/drivers/src/lpc17xx_pinsel.c -o $@

lpc17xx_pwm.o: $(CMSIS)/drivers/src/lpc17xx_pwm.c
	$(CC) -c ${CFLAGS} ${INCLUDES} $(CMSIS)/drivers/src/lpc17xx_pwm.c -o $@

lpc17xx_uart.o: $(CMSIS)/drivers/src/lpc17xx_uart.c
	$(CC) -c ${CFLAGS} ${INCLUDES} $(CMSIS)/drivers/src/lpc17xx_uart.c -o $@

# Common object files:
buffer.o: $(COMMON)/buffer.c
	$(CC) -c ${CFLAGS} ${INCLUDES} $(COMMON)/buffer.c -o $@

ring_buffer.o: $(COMMON)/ring_buffer.c
	$(CC) -c ${CFLAGS} ${INCLUDES} $(COMMON)/ring_buffer.c -o $@

robus.o: $(COMMON)/robus.c
	$(CC) -c ${CFLAGS} ${INCLUDES} $(COMMON)/robus.c -o $@

serial.o: $(COMMON)/serial.c
	$(CC) -c ${CFLAGS} ${INCLUDES} $(COMMON)/serial.c -o $@

systick.o: $(COMMON)/systick.c
	$(CC) -c ${CFLAGS} ${INCLUDES} $(COMMON)/systick.c -o $@

trace.o: $(COMMON)/trace.c
	$(CC) -c ${CFLAGS} ${INCLUDES} $(COMMON)/trace.c -o $@

uart.o: $(COMMON)/uart.c
	$(CC) -c ${CFLAGS} ${INCLUDES} $(COMMON)/uart.c -o $@


