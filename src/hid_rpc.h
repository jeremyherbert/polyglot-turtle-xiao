#ifndef POLYGLOT_TURTLE_XIAO_PROJ_HID_RPC_H
#define POLYGLOT_TURTLE_XIAO_PROJ_HID_RPC_H

void start_hid_rpc_task();

#define I2C_SPI_TRANSACTION_BUFFER_SIZE 2048
extern uint8_t transaction_buffer[I2C_SPI_TRANSACTION_BUFFER_SIZE];

#endif //POLYGLOT_TURTLE_XIAO_PROJ_HID_RPC_H
