/** @file sys_main.c 
*   @brief Application main file
*   @date 07-July-2017
*   @version 04.07.00
*
*   This file contains an empty main function,
*   which can be used for the application.
*/

/* 
* Copyright (C) 2009-2016 Texas Instruments Incorporated - www.ti.com 
* 
* 
*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*
*    Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the   
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/


/* USER CODE BEGIN (0) */
#include "sci.h"
#include "adc.h"
#include "stdlib.h"
#include "stdio.h"

#include "system.h"

#include "FreeRTOS.h"
#include "os_task.h"

#include "orbit_debug.h"
#include "orbit_i2c.h"
/* USER CODE END */

/* Include Files */

#include "sys_common.h"

/* USER CODE BEGIN (1) */

xTaskHandle localization_handle;

void vTaskLocalization(void *pvParameters) {
    adcData_t adc_data;
    adcData_t *adc_data_ptr = &adc_data;
    unsigned int value;

    char msgBuffer[128];
    DebugMsg debugMsg;
    DebugMsg* debugMsgPtr = &debugMsg;
    debugMsg.msg = msgBuffer;
    debugMsg.dynamic = false;

    uint8_t i2c_data[] = { 40 };

    i2c_cmd_t i2c_cmd;
    i2c_cmd_t* i2c_cmd_ptr = &i2c_cmd;
    i2c_cmd.cmd = 30;
    i2c_cmd.num_bytes = 1;
    i2c_cmd.data = i2c_data;
    i2c_cmd.destination = 0x1F;
    i2c_cmd.response_handle = &localization_handle;
    i2c_cmd.wr = WRITE_DATA;

    sprintf(
            msgBuffer,
            "[Localization] Thread Init");
    debugPrint(&debugMsgPtr);

    // FIXME
//    i2c_do_transaction(&i2c_cmd_ptr);

    for(;;) {
        adcStartConversion(adcREG1, adcGROUP1);
        while(!adcIsConversionComplete(adcREG1, adcGROUP1))
            vTaskDelay(10); // switch to other threads while adc does stuff

        adcGetData(adcREG1, adcGROUP1, adc_data_ptr);
        value = (unsigned int)adc_data_ptr->value;

        sprintf(msgBuffer, "[LOCALIZATION] Received data from sensor %d", value);
        debugPrint(&debugMsgPtr);

        vTaskDelay(100);

        // Do kalman filter stuff with the data here
    }
}

void vTaskPositioning(void *pvParameters) {
    char msgBuffer[64];
    DebugMsg debugMsg;
    DebugMsg* debugMsgPtr = &debugMsg;
    debugMsg.msg = msgBuffer;
    debugMsg.dynamic = false;
    for(;;) {
        sprintf(msgBuffer, "[POSITIONING] Unimplemented");
        debugPrint(&debugMsgPtr);
        vTaskDelay(1000);
    }
}

void vTaskCommunication(void *pvParameters) {
    // Try Dynamic allocation
    char *msgBuffer;
    DebugMsg* debugMsgPtr;
    for(;;) {
        msgBuffer = malloc(64*sizeof(char));
        debugMsgPtr = malloc(sizeof(DebugMsg));
        debugMsgPtr->msg = msgBuffer;
        debugMsgPtr->dynamic = true;

        sprintf(msgBuffer, "[COMMUNICATION] Unimplemented");
        debugPrint(&debugMsgPtr);
        vTaskDelay(1500);
    }
}
/* USER CODE END */

/** @fn void main(void)
*   @brief Application main function
*   @note This function is empty by default.
*
*   This function is called after startup.
*   The user can use this function to implement the application.
*/

/* USER CODE BEGIN (2) */
/* USER CODE END */

int main(void)
{
/* USER CODE BEGIN (3) */
    adcInit();

    initDebugTask();
    initI2CTask();

    if (xTaskCreate(vTaskLocalization, "Localization", (uint16_t)1024, NULL, 1, &localization_handle) != pdTRUE)
        for(;;);
    if (xTaskCreate(vTaskPositioning, "Positioning", configMINIMAL_STACK_SIZE, NULL, 1, NULL) != pdTRUE)
        for(;;);
    if (xTaskCreate(vTaskCommunication, "Communication", configMINIMAL_STACK_SIZE, NULL, 1, NULL) != pdTRUE)
        for(;;);

    vTaskStartScheduler();

    for(;;);


/* USER CODE END */

    return 0;
}


/* USER CODE BEGIN (4) */
void adcNotfication(adcBASE_t *adc, unsigned group) { return; }
void sciNotfication(sciBASE_t *sci, unsigned group) { return; }
void esmGroup1Notfication(int bit) { return; }
void esmGroup2Notfication(int bit) { return; }
/* USER CODE END */
