PROGRAM := dynabus
all: $(PROGRAM).hex

PROJECT_ROOT := ../..
include $(PROJECT_ROOT)/common/common.mk

OBJECTS := \
    dynabus.o \
    led.o \
    uart-local.o \
    ${COMMON_OBJECTS} \
    ${CMSIS_OBJECTS}

$(PROGRAM).hex: $(PROGRAM).elf
	$(OBJCOPY) -O ihex  $(PROGRAM).elf $(PROGRAM).hex

$(PROGRAM).elf: ${OBJECTS}
	$(CC) ${OBJECTS} ${LINK_FLAGS} -o $(PROGRAM).elf
	$(SIZE) $(PROGRAM).elf

clean:
	rm -f ${OBJECTS}
	rm -f $(PROGRAM).elf $(PROGRAM).hex $(PROGRAM).map

dynabus.o: dynabus.c
	$(CC) -c ${CFLAGS} ${INCLUDES} dynabus.c -o $@

led.o: led.c
	$(CC) -c ${CFLAGS} ${INCLUDES} led.c -o $@

uart-local.o: uart-local.c
	$(CC) -c ${CFLAGS} ${INCLUDES} uart-local.c -o $@

