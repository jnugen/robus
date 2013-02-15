PROGRAM := main

all: $(PROGRAM).bin

PROJECT_ROOT := ../..
include $(PROJECT_ROOT)/common/common.mk

$(PROGRAM).o: $(PROGRAM).c
	$(CC) -I. -c -fno-common -O0 -g -mcpu=cortex-m3 -mthumb $(PROGRAM).c

$(PROGRAM).elf: $(PROGRAM).o
	$(LD) -Tstm32.ld -nostartfiles -o $(PROGRAM).elf $(PROGRAM).o

$(PROGRAM).bin: $(PROGRAM).elf
	$(OBJCOPY) -Obinary $(PROGRAM).elf $(PROGRAM).bin

clean:
	rm -f $(PROGRAM).o $(PROGRAM).elf $(PROGRAM).bin

