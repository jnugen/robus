PROGRAM := motor3
all: $(PROGRAM).hex

PROJECT_ROOT := ../..
include $(PROJECT_ROOT)/common/robus.mk

OBJECT_FILES := \
    motor.o \
    pwm.o \
    ssp.o \
    motor3.o \
    ${COMMON_OBJECTS} \
    ${CMSIS_CORE_OBJECTS} \
    lpc17xx_clkpwr.o \
    lpc17xx_gpio.o \
    lpc17xx_pinsel.o \
    lpc17xx_pwm.o \
    lpc17xx_ssp.o \
    lpc17xx_uart.o

#    led.o \
#    uart.o \

all: motor3.hex

motor3.hex: motor3.elf
	$(OBJCOPY) -O ihex  motor3.elf motor3.hex

motor3.elf: ${OBJECT_FILES}
	$(CC) ${OBJECT_FILES} ${LD_FLAGS} -o motor3.elf
	$(SIZE) motor3.elf

clean:
	rm -f ${OBJECT_FILES}
	rm -f motor3.elf motor3.hex motor3.srec motor3.map

led.o: led.c
	$(CC) -c ${CFLAGS} led.c -o led.o

motor3.o: motor3.c
	$(CC) -c ${CFLAGS} motor3.c -o motor3.o

motor.o: motor.c
	$(CC) -c ${CFLAGS} motor.c -o motor.o

pwm.o: pwm.c
	$(CC) -c ${CFLAGS} pwm.c -o pwm.o

ssp.o: ssp.c
	$(CC) -c ${CFLAGS} ssp.c -o ssp.o

uart.o: uart.c
	$(CC) -c ${CFLAGS} uart.c -o uart.o

