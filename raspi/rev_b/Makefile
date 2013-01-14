# Define PROGRAM and all target:
PROGRAM := raspi
all: $(PROGRAM).hex

# Read the beginning of robus.mk for documentation about robus.mk:
PROJECT_ROOT := ../..
include $(PROJECT_ROOT)/common/common.mk

# Define needed object files:
OBJECT_FILES := \
    $(PROGRAM).o \
    ${COMMON_OBJECTS} \
    ${CMSIS_OBJECTS}

$(PROGRAM).hex: $(PROGRAM).elf
	$(OBJCOPY) -O ihex $(PROGRAM).elf $(PROGRAM).hex

$(PROGRAM).elf: ${OBJECT_FILES}
	$(CC) ${OBJECT_FILES} ${LINK_FLAGS} -o $(PROGRAM).elf
	$(SIZE) $(PROGRAM).elf

$(PROGRAM).o: $(PROGRAM).c
	$(CC) -c ${CFLAGS} ${INCLUDES} $(PROGRAM).c -o $(PROGRAM).o

clean:
	rm -f ${OBJECT_FILES}
	rm -f $(PROGRAM).elf $(PROGRAM).hex $(PROGRAM).map


