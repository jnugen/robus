// Copyright (c) 2011-2013 by IMC.  All rights reserved.

#ifndef TYPES_H_INCLUDED
#define TYPES_H_INCLUDED 1

// Scalar typedef's with first letter capitalized:
typedef int8_t Byte;
typedef uint8_t Character;
typedef float Float;
typedef int32_t Integer;
typedef uint8_t Logical;
typedef int16_t Short;
typedef uint8_t UByte;
typedef uint32_t UInteger;
typedef uint16_t UShort;

// A frame contains 9-bits of data and potentially some other flags
// from the UART receive FIFE.  We use a signed value so that a negative
// value can indicate "no frame received".
typedef Short Frame;

#endif // TYPES_H_INCLUDED

