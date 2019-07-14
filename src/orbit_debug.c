#include "orbit_debug.h"

void initDebugTask() {
    sciInit();
    debugQueueHandle = xQueueCreate(10, sizeof(DebugMsg*));

    if(debugQueueHandle == NULL)
        for(;;);

    if(xTaskCreate(
                debugTask,
                "Debug",
                configMINIMAL_STACK_SIZE,
                NULL, 1, NULL)
            != pdTRUE)
        for(;;);
}

void debugTask(void* p) {
    DebugMsg* m;
    const char* debug_init_msg = "[Debug] Thread Init\r\n";
    sciSend(scilinREG, strlen(debug_init_msg), (uint8*)debug_init_msg);
    for(;;) {
        if(xQueueReceive(debugQueueHandle, &m, 100)) {
            if(m != NULL) {
                sciSend(scilinREG, strlen(m->msg), (uint8*)m->msg);
                sciSend(scilinREG, 2, "\r\n");
                if(m->dynamic) {
                    free(m->msg);
                    free(m);
                }
            }
        }
    }
}

uint8_t debugPrint(DebugMsg **p) {
    uint8_t i;
    for(i=0; i<ORBIT_DEBUG_RETRIES; i++)
        if(xQueueSend(debugQueueHandle, (void*)p, 100))
            return ORBIT_DEBUG_SUCCESS;
    return ORBIT_DEBUG_FAIL;
}


