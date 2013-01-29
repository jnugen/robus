PROGRAM := motor3

all: $(PROGRAM).hex

PROJECT_ROOT := ../..
#include $(PROJECT_ROOT)/common/common.mk

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

# Selecting Cortex Core:
CORTEX_M := 3
CORE := CM$(CORTEX_M)

# Options for specific architecture:
ARCH_FLAGS := \
    -mthumb \
    -mcpu=cortex-m$(CORTEX_M)

# Compiler #include path:
INCLUDES := \
    -I$(CMSIS_DIR)/drivers/include \
    -I$(CMSIS_DIR)/core/include \
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
LD_SCRIPTS := -L. -L$(COMMON_DIR) -T gcc.ld
LINK_FLAGS := \
    $(LD_SCRIPTS) \
    $(USE_NANO) \
    $(USE_NOHOST) \
    $(GC) \
    $(MAP)

# The object files that make up the program:
OBJECTS := \
    startup_ARMCM3.o \
    system_LPC17xx.o \
    lpc17xx_clkpwr.o \
    lpc17xx_gpio.o \
    lpc17xx_pinsel.o \
    lpc17xx_pwm.o \
    lpc17xx_uart.o \
    buffer.o \
    ring_buffer.o \
    robus.o \
    serial.o \
    systick.o \
    trace.o \
    uart.o \
    motor.o \
    pwm.o \
    $(PROGRAM).o

# Rules to build assembly object files:
startup_ARMCM3.o: $(COMMON_DIR)/startup_ARM$(CORE).S
	$(CC) -c -o $@ $^ ${C_FLAGS} ${AS_FLAGS}

system_LPC17xx.o: $(CMSIS_DIR)/core/system_LPC17xx.c
	$(CC) -c -o $@ $^ ${C_FLAGS} ${AS_FLAGS}

# Rules to build CMSIS object files:
lpc17xx_clkpwr.o: $(CMSIS_DIR)/drivers/src/lpc17xx_clkpwr.c
	$(CC) -c -o $@ $^ ${C_FLAGS} ${CMSIS_FLAGS}

lpc17xx_gpio.o: $(CMSIS_DIR)/drivers/src/lpc17xx_gpio.c
	$(CC) -c -o $@ $^ ${C_FLAGS} ${CMSIS_FLAGS}

lpc17xx_nvic.o: $(CMSIS_DIR)/drivers/src/lpc17xx_nvic.c
	$(CC) -c -o $@ $^ ${C_FLAGS} ${CMSIS_FLAGS}

lpc17xx_pinsel.o: $(CMSIS_DIR)/drivers/src/lpc17xx_pinsel.c
	$(CC) -c -o $@ $^ ${C_FLAGS} ${CMSIS_FLAGS}

lpc17xx_pwm.o: $(CMSIS_DIR)/drivers/src/lpc17xx_pwm.c
	$(CC) -c -o $@ $^ ${C_FLAGS} ${CMSIS_FLAGS}

lpc17xx_uart.o: $(CMSIS_DIR)/drivers/src/lpc17xx_uart.c
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

# Rule to build main program object file:
$(PROGRAM).o: $(PROGRAM).c
	$(CC) -c -o $@ $^ ${C_FLAGS}

motor.o: motor.c
	$(CC) -c -o $@ $^ ${C_FLAGS}

pwm.o: pwm.c
	$(CC) -c -o $@ $^ ${C_FLAGS}

# Rule to build the .elf file (which is needed both for the debugger
# and to produce the .hex file):
$(PROGRAM).elf: ${OBJECTS}
	$(CC) -o $@ $^ ${LINK_FLAGS}
	$(SIZE) $(PROGRAM).elf $(PROGRAM).elf

# Rule to generate the .hex file:
$(PROGRAM).hex: $(PROGRAM).elf
	$(OBJCOPY) -O ihex $^ $@

# A clean target that cleans out generated files:
clean:
	rm -f ${OBJECTS} $(PROGRAM).elf $(PROGRAM).hex $(PROGRAM).map
