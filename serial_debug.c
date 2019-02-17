#include "serial_debug.h"

void sciSafeSend(sciBASE_t* sci, uint32 length, uint8* data){
    bool waitingToSend = true;
    while(waitingToSend) {
        waitingToSend = !xSemaphoreTake(sci_mutex, 100);
        if(!waitingToSend) {
            sciSend(sci, length, data);
            sciSend(sci, 2, "\r\n");
            xSemaphoreGive(sci_mutex);
        } else {
            vTaskDelay(10);
        }
    }
}
