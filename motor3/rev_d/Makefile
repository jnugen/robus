PROGRAM := motor3

all: $(PROGRAM).hex

PROJECT_ROOT := ../..
include $(PROJECT_ROOT)/common/common.mk

# The object files that make up the program:
OBJECTS := \
    ${STARTUP_OBJECTS} \
    lpc17xx_clkpwr.o \
    lpc17xx_gpio.o \
    lpc17xx_pinsel.o \
    lpc17xx_pwm.o \
    lpc17xx_qei.o \
    lpc17xx_uart.o \
    ${COMMON_OBJECTS} \
    motor.o \
    pwm.o \
    qei.o \
    $(PROGRAM).o

# Rule to build main program object file:
$(PROGRAM).o: $(PROGRAM).c
	$(CC) -c -o $@ $^ ${C_FLAGS}

motor.o: motor.c
	$(CC) -c -o $@ $^ ${C_FLAGS}

pwm.o: pwm.c
	$(CC) -c -o $@ $^ ${C_FLAGS}

qei.o: qei.c
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
