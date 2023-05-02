#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_log.h"
#include "driver/uart.h"

#define TXD0 1
#define RXD0 3
#define RX_BUF_SIZE 1024

static char buffer[RX_BUF_SIZE];

void initUart(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD0, RXD0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void readData(void *arg)
{
     int len = uart_read_bytes(UART_NUM_0,(uint8_t*) buffer, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
     printf("Received data len: %d\n", len);
}

