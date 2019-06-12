#ifndef ORBIT_DEBUG_H
#define ORBIT_DEBUG_H

#define ORBIT_DEBUG_RETRIES 3
#define ORBIT_DEBUG_SUCCESS 0
#define ORBIT_DEBUG_FAIL 69


#include "sci.h"

#include "FreeRTOS.h"
#include "os_semphr.h"
#include "os_task.h"

#include "stdlib.h"
#include "string.h"

typedef struct DebugMsg {
    char* msg;
    bool dynamic;
} DebugMsg;

xQueueHandle debugQueueHandle;

void initDebugTask();

uint8_t debugPrint(DebugMsg **p);

static void debugTask(void *p);

#endif
