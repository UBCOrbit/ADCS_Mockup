#ifndef SERIAL_DEBUG_H
#define SERIAL_DEBUG_H

#include "sci.h"
#include "FreeRTOS.h"
#include "os_semphr.h"

xSemaphoreHandle sci_mutex;

void sciSafeSend(sciBASE_t* sci, uint32 length, uint8* data);

#endif


