#ifndef ORBIT_I2C_H
#define ORBIT_I2C_H

#define I2C                      i2cREG1
#define I2C_TIMEOUT_MAX          200000 // number of attempts before timing out (units are attempts at register check), typically takes ~3500 for a correct wait

#define I2C_SUCCESS 0
#define I2C_ERR_START            -4     // Failed to begin a transaction because I2C bus was not available (busy bit set)
#define I2C_UNEXPECTED_NUM_BYTES -42
#define I2C_TX_FAIL         -5          // Bit indicating that transmit data has been copied into transmit shift register has not been set
#define I2C_RX_FAIL         -6          // Bit indicating that receive data has been copied into receive shift register has not been set
#define I2C_ERR_MST              -8     // I2C failed to clear MST bit after transaction
#define I2C_ERR_BB_CLEAR         -9     // I2C failed to clear busy but after transaction
#define I2C_ERR_NACK        -2          // Unexpected NACK received when trying to write to and I2C device

#define I2C_TRANSACTION_REQUESTED 0
#define I2C_TRANSACTION_FAIL -67
#define I2C_TRANSACTION_RETRIES 3

#define MAX_WRITE_DATA_SIZE 2

#include "i2c.h"
#include "FreeRTOS.h"
#include "os_task.h"
#include "os_queue.h"
#include "orbit_debug.h"

#include "stdio.h"


xQueueHandle i2c_cmd_queue;

typedef int8_t i2c_result_t;

typedef enum {
    READ_DATA,
    WRITE_DATA,
} i2c_wr_t;

/**
 * i2c_cmd_t
 *
 * Struct to store the information for an I2C transaction
 *
 * cmd: I2C register
 * response_handle: Pointer to a TaskHandle for the calling task
 * destination: Address of slave device
 * wr: READ_DATA or WRITE_DATA
 * num_bytes: number of bytes to read or write
 * data: data buffer to write from or read to
 *
 */
typedef struct {
    uint8_t cmd;
    TaskHandle_t* response_handle;
    uint8_t destination;
    i2c_wr_t wr;
    uint8_t num_bytes;
    uint8_t *data;
} i2c_cmd_t;

void initI2CTask();

static void I2CTask(void *p);

i2c_result_t i2c_do_transaction(i2c_cmd_t **p);
i2c_result_t i2c_read(uint8_t addr, uint8_t reg_read, uint32_t bytes_rcv, uint8_t* data_rcv);
i2c_result_t i2c_write(uint8_t addr, uint32_t bytes_send, const uint8_t* data_send);


#endif
