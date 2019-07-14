#include "orbit_i2c.h"

void notify_sender(i2c_cmd_t* command, i2c_result_t error);
void i2c_setup_transaction(uint8_t addr, uint32_t dir);
int8_t i2c_complete_transaction();
void i2c_clear_nack();
int8_t i2c_send(uint32_t length, const uint8_t * data);
int8_t i2c_receive(uint32_t length, uint8_t * data);


void initI2CTask() {
    i2cInit();
    i2c_cmd_queue = xQueueCreate(10, sizeof(i2c_cmd_t*));

    if(i2c_cmd_queue == NULL)
        for(;;);

    if(xTaskCreate(
                I2CTask,
                "I2C",
                configMINIMAL_STACK_SIZE,
                NULL, 1, NULL)
            != pdTRUE)
        for(;;);
}

void I2CTask(void *p){
    i2c_cmd_t cmd;
    uint8_t write_data_buffer[MAX_WRITE_DATA_SIZE + 1] = {0};
    int8_t errcode;

    char msg_buffer[128];
    DebugMsg dbg_msg;
    DebugMsg* dbg_msg_ptr = &dbg_msg;
    dbg_msg.msg = msg_buffer;
    dbg_msg.dynamic = false;
    sprintf(
            msg_buffer,
            "[I2C] Thread Init");
    debugPrint(&dbg_msg_ptr);

    for(;;) {
        xQueueReceive(
                i2c_cmd_queue,
                (void*) &cmd,
                portMAX_DELAY);

        sprintf(
                msg_buffer, 
                "[I2C] %s request received",
                cmd.wr == WRITE_DATA ? "Write" : "Read");
        debugPrint(&dbg_msg_ptr);
        
        if(cmd.wr == WRITE_DATA) {
            if(cmd.num_bytes == 1 || cmd.num_bytes == 2) { 
                write_data_buffer[0] = cmd.cmd;
                write_data_buffer[1] = cmd.data[0];
                if(cmd.num_bytes == 2)
                    write_data_buffer[2] = cmd.data[1];
                errcode = i2c_write(
                        cmd.destination,
                        cmd.num_bytes + 1,
                        write_data_buffer);
            } else {
                errcode = I2C_UNEXPECTED_NUM_BYTES;
            }
        } else if(cmd.wr == READ_DATA) {
            errcode = i2c_read(
                    cmd.destination,
                    cmd.cmd,
                    cmd.num_bytes,
                    cmd.data);
        }

        //TODO: remove
        if(errcode != I2C_SUCCESS) {
            notify_sender(&cmd, errcode);
            continue;
        }

        notify_sender(&cmd, I2C_SUCCESS);
    }
}

void notify_sender(i2c_cmd_t* cmd, i2c_result_t result) {
    if(xTaskNotify(
                cmd->response_handle, 
                (uint32_t)result, 
                eSetValueWithoutOverwrite) != pdPASS) {
        DebugMsg* err_msg = malloc(sizeof(DebugMsg));
        err_msg->dynamic = true;
        err_msg->msg = malloc(32*sizeof(char));
        sprintf(err_msg->msg, "[I2C] Error %d", (int)result);
        debugPrint(&err_msg);
    }
}

i2c_result_t i2c_do_transaction(i2c_cmd_t **p){
    uint8_t i;
    for(i=0; i<I2C_TRANSACTION_RETRIES; i++)
        if(xQueueSend(i2c_cmd_queue, (void*)p, 100))
            return I2C_TRANSACTION_REQUESTED;
    return I2C_TRANSACTION_FAIL;
}

/**
 * i2c_read
 *
 * Reads specified number of bytes from I2C device
 *
 * addr: I2C device address
 * reg_read: register address to read data from
 * bytes_rcv: number of bytes to read from the device
 * data_rcv: buffer to store read bytes
 *
 * returns: I2C module status
 */
i2c_result_t i2c_read(uint8_t addr, uint8_t reg_read, uint32_t bytes_rcv, uint8_t* data_rcv) {
    if (i2cIsBusBusy(I2C)) {
        return I2C_ERR_START;
    }
    i2cSetStop(I2C);
    i2c_setup_transaction(addr, I2C_TRANSMITTER);
    I2C->MDR |= I2C_REPEATMODE;
    i2cSetStart(I2C);
    int8_t errcode = i2c_send(1, &reg_read);
    if (errcode != I2C_SUCCESS) {
        return errcode;
    }
    errcode = i2c_complete_transaction();
    if (errcode != I2C_SUCCESS) {
        return errcode;
    }

    i2c_setup_transaction(addr, I2C_RECEIVER);
    i2cSetStart(I2C);
    i2c_receive(bytes_rcv - 1, data_rcv);
    /* TMS570 Manual, Section 28.3.2 Master Receiver Mode:
     * "Due to the double buffer implementation on the receive side, the master
     * must generate the stop condition (STP =1) after reading the (message size - 1)th data."
     * http://www.ti.com/lit/ug/spnu499c/spnu499c.pdf
     */
    i2cSetStop(I2C);
    i2c_receive(1, &data_rcv[bytes_rcv - 1]);
    errcode = i2c_complete_transaction();
    if (errcode != I2C_SUCCESS) {
        return errcode;
    }
    i2cClearSCD(I2C);
    i2c_clear_nack();
    return I2C_SUCCESS;
}

/**
 * i2c_write
 *
 * Writes specified number of bytes to I2C device
 *
 * addr: I2C device address
 * bytes_send: number of bytes to send to the device
 * data_send: buffer containing bytes to write
 *
 * returns: I2C module status
 */
i2c_result_t i2c_write(uint8_t addr, uint32_t bytes_send, const uint8_t* data_send) {
    if (i2cIsBusBusy(I2C)) {
        return I2C_ERR_START;
    }
    i2cSetStop(I2C);
    I2C->MDR &= ~(I2C_REPEATMODE);
    i2c_setup_transaction(addr, I2C_TRANSMITTER);
    i2cSetCount(I2C, bytes_send);
    i2cSetStart(I2C);
    int8_t errcode = i2c_send(bytes_send, data_send);
    if (errcode != I2C_SUCCESS) {
        return errcode;
    }
    errcode = i2c_complete_transaction();
    if (errcode != I2C_SUCCESS) {
        return errcode;
    }
    i2cSetStop(I2C);
    i2cClearSCD(I2C);
    return I2C_SUCCESS;
}
/**
 * i2c_send
 *
 * Sends a specified number of bytes to a slave I2C device

 *  length: number of bytes for transmission
 *  data: pointer to an array containing data for transmission
 *
 *  returns: I2C module status
 *          - I2C_TX_FAIL if TX bit was not set
 *          - I2C_ERR_NACK if NACK was received
 *          - I2C_OK otherwise
 */
int8_t i2c_send(uint32_t length, const uint8_t * data) {
    uint32_t timeout_count = 0;
    while (length > 0U) {
        while (((I2C->STR & I2C_TX_INT ) == 0U) && (timeout_count < I2C_TIMEOUT_MAX)) {
            timeout_count++;
        }
        if (timeout_count >= I2C_TIMEOUT_MAX) {
            return I2C_TX_FAIL;
        }

        if (I2C->STR & I2C_NACK_INT) {
            i2cSetStop(I2C);
            i2c_clear_nack();
            return I2C_ERR_NACK;
        }

        I2C->DXR = *data;
        data++;
        length--;
    }
    return I2C_SUCCESS;
}

/**
 * i2c_receive
 *
 * Received a specified number of bytes from a slave I2C device
 *
 * length: number of bytes for transmission
 * data: pointer to a byte array to store received data
 *
 *  returns: I2C module status
 *          - I2C_RX_FAIL if RX bit was not set
 *          - I2C_OK otherwise
 */
int8_t i2c_receive(uint32_t length, uint8_t * data) {
    uint32_t timeout_count;
    timeout_count = 0;
    while (length > 0U) {
        while (((I2C->STR & I2C_RX_INT) == 0U) && (timeout_count < I2C_TIMEOUT_MAX)) {
            timeout_count++;
        }
        if (timeout_count >= I2C_TIMEOUT_MAX) {
            return I2C_RX_FAIL;
        }

        *data = I2C->DRR;
        data++;
        length--;
    }
    return I2C_SUCCESS;
}


/**
 * i2c_setup_transaction
 *
 * Sets I2C bus for a transaction, required before sending data
 *
 *  addr: address of I2C slave device
 *  dir: mode of I2C module: I2C_TRANSMITTER or I2C_RECEIVER
 *  num_bytes: number of bytes to transmit or receive
 */
void i2c_setup_transaction(uint8_t addr, uint32_t dir) {
    i2cSetSlaveAdd(I2C, addr);
    i2cSetDirection(I2C, dir);
    i2cSetMode(I2C, I2C_MASTER);
}

/**
 * i2c_complete_transaction
 *
 * Waits for I2C Master/Slave and bus busy bits to reset, required before initiating new transaction
 *
 * returns: I2C module status
 *          - I2C_ERR_BB_CLEAR if bus busy bit failed to clear
 *          - I2C_ERR_MST if Master Bit failed to clear
 *          - I2C_OK otherwise
 */
int8_t i2c_complete_transaction() {
    uint8_t counter = 0;
    while((!i2cIsMasterReady(I2C) || i2cIsBusBusy(I2C))) {
        counter++;
        if (counter > I2C_TIMEOUT_MAX) {
            if (i2cIsBusBusy(I2C)) {
                return I2C_ERR_BB_CLEAR;
            } else {
                return I2C_ERR_MST;
            }
        }
    }
    return I2C_SUCCESS;
}

/**
 * i2c_clear_nack
 *
 * Clears NACK bit
 */
void i2c_clear_nack() {
    I2C->STR |= I2C_NACK;
}



