# Copyright (c) 2013 by IMC. All rights reserved.

PROGRAM := motor3

all: $(PROGRAM).hex

PROJECT_ROOT := ../..
COMMON := $(PROJECT_ROOT)/common

include $(COMMON)/common.mk

OBJECTS := \
    $(PROGRAM).o \
    motor.o \
    pwm.o \
    ${CMSIS_OBJECTS}

$(PROGRAM).o: $(PROGRAM).c
	$(CC) -c ${CFLAGS} ${INCLUDES} $(PROGRAM).c -o $(PROGRAM).o

motor.o: motor.c
	$(CC) -c ${CFLAGS} ${INCLUDES} motor.c -o motor.o

pwm.o: pwm.c
	$(CC) -c ${CFLAGS} ${INCLUDES} pwm.c -o pwm.o

$(PROGRAM).elf: ${OBJECTS}
	$(CC) ${OBJECTS} ${LINK_FLAGS} -o $(PROGRAM).elf

$(PROGRAM).hex: $(PROGRAM).elf
	$(OBJCOPY) -O ihex  $(PROGRAM).elf $(PROGRAM).hex
	$(SIZE) $(PROGRAM).elf $(PROGRAM).elf $(PROGRAM).hex

clean:
	rm -f ${OBJECTS} $(PROGRAM).elf $(PROGRAM).hex $(PROGRAM).map
