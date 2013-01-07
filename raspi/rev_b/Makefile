# Define PROGRAM and all target:
PROGRAM := raspi
all: $(PROGRAM).hex

# Read the beginning of robus.mk for documentation about robus.mk:
PROJECT_ROOT := ../..
include $(PROJECT_ROOT)/common/robus.mk

# Define needed object files:
OBJECT_FILES := \
    raspi.o \
    ${COMMON_OBJECTS} \
    ${CMSIS_CORE_OBJECTS} \
    lpc17xx_clkpwr.o \
    lpc17xx_pinsel.o \
    lpc17xx_uart.o

$(PROGRAM).hex: $(PROGRAM).elf
	$(OBJCOPY) -O ihex $(PROGRAM).elf $(PROGRAM).hex

$(PROGRAM).elf: ${OBJECT_FILES}
	$(CC) ${OBJECT_FILES} ${LD_FLAGS} -o $(PROGRAM).elf
	$(SIZE) $(PROGRAM).elf

$(PROGRAM).o: $(PROGRAM).c
	$(CC) -c ${CFLAGS} $(PROGRAM).c -o $(PROGRAM).o

clean:
	rm -f ${OBJECT_FILES}
	rm -f $(PROGRAM).elf $(PROGRAM).hex $(PROGRAM).map


