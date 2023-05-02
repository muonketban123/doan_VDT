#ifndef UART_H
#define UART_H
#include "esp_log.h"
#include "driver/uart.h"
void initUart(void);
//void sendData(const char* data);
//void checkConnectSIM800l(void * arg);
//void checkConnectNetwork(void *arg);
//void call_set_SIM800l(void *arg);
void a7670c_setup(void);
void call_SIM800l(void *arg);
#endif