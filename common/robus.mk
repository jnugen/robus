# robus.mk
#
# This make file fragment is included from another makfile as follows:
#
#    PROJECT_ROOT := ../..             # As appropriate
#    include $(PROJECT_ROOT)/common/robus.mk
#    
# makefile fragment requires that both $(PROJECT_ROOT) and $(ARM_TOOLS_ROOT)
# be defined prior being included.  It is expected that $(ARM_TOOLS_ROOT)
# will be picked via an environment variable that is defined in .bashrc:
#
#    export ARM_TOOLS_ROOT = ...

# ARM tools definitions:
ARM_TOOLS_BIN := $(ARM_TOOLS_ROOT)/bin
ARM_TOOLS_BIN_PREFIX := $(ARM_TOOLS_BIN)/arm-none-eabi
CC := $(ARM_TOOLS_BIN_PREFIX)-gcc
AS := $(ARM_TOOLS_BIN_PREFIX)-as
OBJCOPY := $(ARM_TOOLS_BIN_PREFIX)-objcopy
SIZE := $(ARM_TOOLS_BIN_PREFIX)-size

# CMSIS is a vendor supplied library of code that can be used
# to access processor peripherals.
#
# CMSIS library locations:
COMMON := $(PROJECT_ROOT)/common
CMSIS := $(PROJECT_ROOT)/cmsis
CMSIS_CORE := $(CMSIS)/core
CMSIS_DRIVERS := $(CMSIS)/drivers

# Compiler -I flags:
INCLUDES := \
    -I$(CMSIS_DRIVERS)/include \
    -I$(CMSIS_CORE)/include \
    -I.

# All the other compiler flags:
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
LD_START_FLAGS := \
    -static \
    -mcpu=cortex-m3 \
    -mthumb \
    -mthumb-interwork
LIBRARIES := \
    -lc \
    -lg \
    -lstdc++ \
    -lsupc++ \
    -lgcc \
    -lm
LIBRARIES_GROUP := \
    -Wl,--start-group \
    -L$(ARM_TOOLS_ROOT)/lib/gcc/arm-none-eabi/4.6.3/thumb2 \
    -L$(ARM_TOOLS_ROOT)/arm-none-eabi/lib/thumb2 \
    ${LIBRARIES} \
    -Wl,--end-group
MAP_FLAGS := \
    -Xlinker -Map \
    -Xlinker raspi.map \
    -Xlinker -T $(COMMON)/ldscript_rom_gnu.ld    
LD_FLAGS := \
    ${LD_START_FLAGS} \
    ${LD_LIBRARIES_GROUP} \
    ${MAP_FLAGS}

common.o: $(COMMON)/common.c
	$(CC) -c ${CFLAGS} $(COMMON)/common.c -o common.o

# Dependencies for building CMS_CORE files.  All programs have these
# files:
core_cm3.o: $(CMSIS_CORE)/core_cm3.c
	$(CC) -c ${CFLAGS} $(CMSIS_CORE)/core_cm3.c -o core_cm3.o

system_LPC17xx.o: $(CMSIS_CORE)/system_LPC17xx.c \
    $(CMSIS_CORE)/include/LPC17xx.h
	$(CC) -c ${CFLAGS} $(CMSIS_CORE)/system_LPC17xx.c -o system_LPC17xx.o

startup_LPC17xx.o: $(CMSIS_CORE)/startup_LPC17xx.s
	$(AS) -mcpu=cortex-m3 -gdwarf-2  -gdwarf-2 --defsym RAM_MODE=0  $(CMSIS_CORE)/startup_LPC17xx.s -o startup_LPC17xx.o

# Define CMSIS_CORE_OBJECTS that will get built in local directory:
CMSIS_CORE_OBJECTS := \
    core_cm3.o \
    system_LPC17xx.o \
    startup_LPC17xx.o

# Dependencides for build individual CMSIS files.  These files are
# compiled as needed by the program.
#
# IMPORTANT: The file lpc17xx_libcfg.h provides compiler flags that
# specify what portions of each drive are acutally compiled.  If you
# need UART0, UART1, etc., be sure to edit this file to enable the
# code for the specified feature.

lpc17xx_clkpwr.o: $(CMSIS_DRIVERS)/src/lpc17xx_clkpwr.c \
    lpc17xx_libcfg.h \
    $(CMSIS_DRIVERS)/include/lpc17xx_clkpwr.h
	$(CC) -c ${CFLAGS} $(CMSIS_DRIVERS)/src/lpc17xx_clkpwr.c -o lpc17xx_clkpwr.o

lpc17xx_gpio.o: $(CMSIS_DRIVERS)/src/lpc17xx_gpio.c \
    lpc17xx_libcfg.h \
    $(CMSIS_DRIVERS)/include/lpc17xx_gpio.h
	$(CC) -c ${CFLAGS} $(CMSIS_DRIVERS)/src/lpc17xx_gpio.c -o lpc17xx_gpio.o

lpc17xx_pinsel.o: $(CMSIS_DRIVERS)/src/lpc17xx_pinsel.c \
    lpc17xx_libcfg.h \
    $(CMSIS_DRIVERS)/include/lpc17xx_pinsel.h
	$(CC) -c ${CFLAGS} $(CMSIS_DRIVERS)/src/lpc17xx_pinsel.c -o lpc17xx_pinsel.o

lpc17xx_pwm.o: $(CMSIS_DRIVERS)/src/lpc17xx_pwm.c \
    lpc17xx_libcfg.h \
    $(CMSIS_DRIVERS)/include/lpc17xx_pwm.h
	$(CC) -c ${CFLAGS} $(CMSIS_DRIVERS)/src/lpc17xx_pwm.c -o lpc17xx_pwm.o

lpc17xx_ssp.o: $(CMSIS_DRIVERS)/src/lpc17xx_ssp.c \
    lpc17xx_libcfg.h \
    $(CMSIS_DRIVERS)/include/lpc17xx_ssp.h
	$(CC) -c ${CFLAGS} $(CMSIS_DRIVERS)/src/lpc17xx_ssp.c -o lpc17xx_ssp.o

lpc17xx_uart.o: $(CMSIS_DRIVERS)/src/lpc17xx_uart.c \
    lpc17xx_libcfg.h \
    $(CMSIS_DRIVERS)/include/lpc17xx_clkpwr.h \
    $(CMSIS_DRIVERS)/include/lpc17xx_uart.h
	$(CC) -c ${CFLAGS} $(CMSIS_DRIVERS)/src/lpc17xx_uart.c -o lpc17xx_uart.o

