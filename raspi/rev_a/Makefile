PROGRAM := raspi

all: $(PROGRAM).hex

PROJECT_ROOT := ../..
include $(PROJECT_ROOT)/common/common.mk

OBJECTS := \
    $(CMSIS_OBJECTS) \
    $(PROGRAM).o

$(PROGRAM).o:
	$(CC) -c ${CFLAGS} ${INCLUDES} raspi.c -o $@

$(PROGRAM).elf: ${OBJECTS}
	$(CC) ${OBJECTS} ${LINK_FLAGS} -o $@
	$(SIZE) $(PROGRAM).elf raspi.elf

$(PROGRAM).hex: $(PROGRAM).elf
	$(OBJCOPY) -O ihex $(PROGRAM).elf $@

clean:
	rm -f ${OBJECTS} $(PROGRAM).elf $(PROGRAM).hex $(PROGRAM).map
