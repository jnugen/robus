// Copyright (c) 2011-2013 by IMC.  All rights reserved.

#ifndef COMMON_H_INCLUDED
#define COMMON_H_INCLUDED 1

// Eventually these 3 #includes get removed:
#include "lpc17xx_uart.h"
#include "lpc17xx_libcfg.h"
#include "lpc17xx_pinsel.h"

#include "buffer.h"
#include "ring_buffer.h"
#include "robus.h"
#include "serial.h"
#include "systick.h"
#include "trace.h"
#include "types.h"
#include "uart.h"

// {Logical} stuff:
extern const Logical Logical__true;
extern const Logical Logical__false;

#endif // COMMON_H_INCLUDED
