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
#include <stdio.h>
#include <string.h>
#include "sci.h"
#include "adc.h"
#include "stdlib.h"

#include "system.h"

#include "FreeRTOS.h"
#include "os_semphr.h"
#include "os_task.h"

#include "LSM9DS1.h"
#include "serial_debug.h"
/* USER CODE END */

/* Include Files */

#include "sys_common.h"

/* USER CODE BEGIN (1) */


void vTaskLocalization(void *pvParameters) {
    imu_t imu;
    const char imu_error[] = "[LOCALIZATION] Error intializing IMU";
    const char imu_success[] = "[LOCALIZATION] IMU initialized!";

    imu_config_t imu_config =
    {
     .enable_accel = true,
     .enable_mag = true,
     .enable_gyro = false,
     .ag_addr = LSM9DS1_AG_ADDR(1),
     .mag_addr = LSM9DS1_M_ADDR(1),
     .calibrate = false,
     .low_power_mode = true
    };

    if(!LSM9DS1_init(&imu, &imu_config)) {
        sciSafeSend(scilinREG, strlen(imu_error), imu_error);
    } else {
        sciSafeSend(scilinREG, strlen(imu_error), imu_success);
    }

    adcData_t adc_data;
    adcData_t *adc_data_ptr = &adc_data;
    unsigned int  value;
    char buffer[64];

    for(;;) {
        adcStartConversion(adcREG1, adcGROUP1);
        while(!adcIsConversionComplete(adcREG1, adcGROUP1))
            vTaskDelay(10); // switch to other threads while adc does stuff

        adcGetData(adcREG1, adcGROUP1, adc_data_ptr);
        value = (unsigned int)adc_data_ptr->value;
        sprintf(buffer, "[LOCALIZATION] Received data from sensor: %d\r\n", value);
        sciSafeSend(scilinREG, strlen(buffer), buffer);
        vTaskDelay(100);

        // Do kalman filter stuff with the data here
    }
}

void vTaskPositioning(void *pvParameters) {
    bool waitingToSend;
    for(;;) {
        waitingToSend = true;
        while(waitingToSend) {
            waitingToSend = !xSemaphoreTake(sci_mutex, 100);
            if(!waitingToSend) {
                sciSend(scilinREG, 27, "[POSITIONING] Unimplemented");
                sciSend(scilinREG, 2, (unsigned char*)"\r\n");
                xSemaphoreGive(sci_mutex);
            } else {
                vTaskDelay(10);
            }
        }
        vTaskDelay(1000);
    }
}

void vTaskCommunication(void *pvParameters) {
    bool waitingToSend;
    for(;;) {
        waitingToSend = true;
        while(waitingToSend) {
            waitingToSend = !xSemaphoreTake(sci_mutex, 100);
            if(!waitingToSend) {
                sciSend(scilinREG, 29, "[COMMUNICATION] Unimplemented");
                sciSend(scilinREG, 2, (unsigned char*)"\r\n");
                xSemaphoreGive(sci_mutex);
            } else {
                vTaskDelay(10);
            }
        }
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
    sciInit();
    adcInit();

    sci_mutex = xSemaphoreCreateMutex();

    if (xTaskCreate(vTaskLocalization, "Localization", (uint16_t)1024, NULL, 1, NULL) != pdTRUE)
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
