PROGRAM := dynabus
all: $(PROGRAM).hex

PROJECT_ROOT := ../..
include $(PROJECT_ROOT)/common/common.mk

OBJECT_FILES := \
    dynabus.o \
    led.o \
    uart-local.o \
    ${COMMON_OBJECTS} \
    ${CMSIS_OBJECTS}

$(PROGRAM).hex: $(PROGRAM).elf
	$(OBJCOPY) -O ihex  $(PROGRAM).elf $(PROGRAM).hex

$(PROGRAM).elf: ${OBJECT_FILES}
	$(CC) ${OBJECT_FILES} ${LINK_FLAGS} -o $(PROGRAM).elf
	$(SIZE) $(PROGRAM).elf

clean:
	rm -f ${OBJECT_FILES}
	rm -f $(PROGRAM).elf $(PROGRAM).hex $(PROGRAM).srec $(PROGRAM).map

dynabus.o: dynabus.c
	$(CC) -c ${CFLAGS} $(INCLUDES) dynabus.c -o $@

led.o: led.c
	$(CC) -c ${CFLAGS} $(INCLUDES) led.c -o $@

uart-local.o: uart-local.c
	$(CC) -c ${CFLAGS} $(INCLUDES) uart-local.c -o $@

