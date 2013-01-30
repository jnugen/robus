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
OBJCOPY := $(ARM_TOOLS_BIN)/arm-none-eabi-objcopy
SIZE := $(ARM_TOOLS_BIN)/arm-none-eabi-size

# The common directory contains some code and definitions shared across
# the entire project:
COMMON_DIR := $(PROJECT_ROOT)/common

# The cmsis directory vendor supplied library of code that can be used
# to access processor peripherals:
CMSIS_DIR := $(PROJECT_ROOT)/cmsis
CMSIS_LPC17XX_DIR := $(CMSIS_DIR)/lpc17xx
CMSIS_LPC17XX_DRIVERS := $(CMSIS_LPC17XX_DIR)/Drivers/source
CMSIS_LPC17XX_CORE := $(CMSIS_LPC17XX_DIR)/Core/CM3/DeviceSupport/NXP/LPC17xx

# Selecting Cortex Core:
CORTEX_M := 3
CORE := CM$(CORTEX_M)

# Options for specific architecture:
ARCH_FLAGS := \
    -mthumb \
    -mcpu=cortex-m$(CORTEX_M)

# Compiler #include path:
INCLUDES := \
    -I$(CMSIS_LPC17XX_DIR)/Core/CM3/CoreSupport \
    -I$(CMSIS_LPC17XX_DIR)/Core/CM3/DeviceSupport/NXP/LPC17xx \
    -I$(CMSIS_LPC17XX_DIR)/Drivers/include \
    -I$(COMMON_DIR) \
    -I.

# To optimize for size, use -Os.  To have debugging work disable -Os and
# set -g instead:
#OPTIMIZATIONS := -Os
OPTIMIZATIONS := \
    -gdwarf-2 \
    -g

# C compiler flags:
C_FLAGS := \
    ${ARCH_FLAGS} \
    ${INCLUDES} \
    -ffunction-sections \
    -fdata-sections \
    ${OPTIMIZATIONS}

# Additional assembler definitions:
AS_FLAGS := \
    -D__STARTUP_CLEAR_BSS \
    -D__START=main \
    -DRAM_MODE=0

# Linker stuff:

# To save code space, use --gc-sections to garbage collect sections
# that are of zero size:
GC := -Wl,--gc-sections

# Create map file
MAP := -Wl,-Map=$(PROGRAM).map

# Use newlib-nano. To disable it, specify USE_NANO :=  :
USE_NANO := --specs=nano.specs

# Use seimhosting or not:
USE_SEMIHOST := --specs=rdimon.specs -lc -lc -lrdimon
USE_NOHOST := -lc -lc -lnosys

CMSIS_FLAGS := \
    -D__BUILD_WITH_EXAMPLE__=1

# Make sure that we find the required scripts:
LD_SCRIPTS := -L$(COMMON_DIR) -T gcc_cortex_m3.ld
LINK_FLAGS := \
    ${ARCH_FLAGS} \
    $(LD_SCRIPTS) \
    $(USE_NANO) \
    $(USE_NOHOST) \
    $(GC) \
    $(MAP)

# Start-up objects:
STARTUP_OBJECTS := \
    startup_ARMCM3.o \
    system_LPC17xx.o

# CMSIS objects:
CMSIS_OBJECTS := \
    lpc17xx_clkpwr.o \
    lpc17xx_gpio.o \
    lpc17xx_pinsel.o \
    lpc17xx_pwm.o \
    lpc17xx_qei.o \
    lpc17xx_uart.o

# Common objects:
COMMON_OBJECTS := \
    buffer.o \
    ring_buffer.o \
    robus.o \
    serial.o \
    systick.o \
    trace.o \
    uart.o

# Rules to build assembly object files:
startup_ARMCM3.o: $(COMMON_DIR)/startup_ARM$(CORE).S
	$(CC) -c -o $@ $^ ${C_FLAGS} ${AS_FLAGS}

system_LPC17xx.o: $(CMSIS_LPC17XX_CORE)/system_LPC17xx.c
	$(CC) -c -o $@ $^ ${C_FLAGS} ${AS_FLAGS}

# Rules to build CMSIS object files:
lpc17xx_clkpwr.o: $(CMSIS_LPC17XX_DRIVERS)/lpc17xx_clkpwr.c
	$(CC) -c -o $@ $^ ${C_FLAGS} ${CMSIS_FLAGS}

lpc17xx_gpio.o: $(CMSIS_LPC17XX_DRIVERS)/lpc17xx_gpio.c
	$(CC) -c -o $@ $^ ${C_FLAGS} ${CMSIS_FLAGS}

lpc17xx_nvic.o: $(CMSIS_LPC17XX_DRIVERS)/lpc17xx_nvic.c
	$(CC) -c -o $@ $^ ${C_FLAGS} ${CMSIS_FLAGS}

lpc17xx_pinsel.o: $(CMSIS_LPC17XX_DRIVERS)/lpc17xx_pinsel.c
	$(CC) -c -o $@ $^ ${C_FLAGS} ${CMSIS_FLAGS}

lpc17xx_pwm.o: $(CMSIS_LPC17XX_DRIVERS)/lpc17xx_pwm.c
	$(CC) -c -o $@ $^ ${C_FLAGS} ${CMSIS_FLAGS}

lpc17xx_qei.o: $(CMSIS_LPC17XX_DRIVERS)/lpc17xx_qei.c
	$(CC) -c -o $@ $^ ${C_FLAGS} ${CMSIS_FLAGS}

lpc17xx_uart.o: $(CMSIS_LPC17XX_DRIVERS)/lpc17xx_uart.c
	$(CC) -c -o $@ $^ ${C_FLAGS} ${CMSIS_FLAGS}

# Rules to build common object files:
buffer.o: $(COMMON_DIR)/buffer.c
	$(CC) -c -o $@ $^ ${C_FLAGS}

ring_buffer.o: $(COMMON_DIR)/ring_buffer.c
	$(CC) -c -o $@ $^ ${C_FLAGS}

robus.o: $(COMMON_DIR)/robus.c
	$(CC) -c -o $@ $^ ${C_FLAGS}

serial.o: $(COMMON_DIR)/serial.c
	$(CC) -c -o $@ $^ ${C_FLAGS}

systick.o: $(COMMON_DIR)/systick.c
	$(CC) -c -o $@ $^ ${C_FLAGS}

trace.o: $(COMMON_DIR)/trace.c
	$(CC) -c -o $@ $^ ${C_FLAGS}

uart.o: $(COMMON_DIR)/uart.c
	$(CC) -c -o $@ $^ ${C_FLAGS}


