// Copyright (c) 2011-2013 by IMC.  All rights reserved.

#include "trace.h"

UShort trace_buffer[TRACE_SIZE];
UShort trace_index = 0;

void trace_dump(
  Serial serial)
{
    Integer index = 0;
    for (index = 0; index < TRACE_SIZE; index++) {
        // Fetch the {trace} value and clear it out of the trace buffer:
	UInteger masked_index = (index + trace_index) & TRACE_MASK;
	UInteger trace = trace_buffer[masked_index];
	trace_buffer[masked_index] = 0;

	// Show any tracing flags:
	if ((trace & 0x1000) != 0) {
	    // Received (Get) byte:
	    Serial__character_put(serial, 'G');
	    trace &= ~0x1000;
	}
	if ((trace & 0x200) != 0) {
	    // Transmitted (Put) byte:
	    Serial__character_put(serial, 'P');
	    trace &= ~0x200;
	}
	if ((trace & 0x800) != 0) {
	    // Timeout byte:
	    Serial__character_put(serial, 'T');
	    trace &= ~0x800;
	}

	// Output the remaining trace value:
	Serial__hex_put(serial, trace);
	char character = ' ';
	if ((index & 0xf) == 0xf) {
	    character = '\n';
	}
	Serial__character_put(serial, character);
    }
}
