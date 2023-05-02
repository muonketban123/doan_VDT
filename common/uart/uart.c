#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_log.h"
#include "driver/uart.h"

#define TXD2 17
#define RXD2 16
//#define RX_BUF_SIZE 1024
#define BUF_SIZE 1024

//static char buffer[RX_BUF_SIZE];
 static char resp[BUF_SIZE];

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
    uart_driver_install(UART_NUM_2, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, TXD2, RXD2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void uart_write(const char* data) {
    uart_write_bytes(UART_NUM_2, data, strlen(data));
}

void uart_read(char* data) {
    int len = uart_read_bytes(UART_NUM_2, (uint8_t*)data, BUF_SIZE, 20 / portTICK_RATE_MS);
    data[len] = '\0';
}

void a7670c_setup(void) {
    uart_write("AT\r\n");
    uart_read(resp);
    printf("A7670C response: %s\n", resp);

    uart_write("AT+CPIN?\r\n");
    uart_read(resp);
    printf("A7670C response: %s\n", resp);

    uart_write("AT+COPS?\r\n");
    uart_read(resp);
    printf("A7670C response: %s\n", resp);

   // uart_write("AT+CGDCONT=1,\"IP\",\"<APN>\"\r\n"); // Thay <APN> bằng APN của nhà mạng
    //uart_read(resp);
    //printf("A7670C response: %s\n", resp);

    uart_write("AT+CGATT=1\r\n");
    uart_read(resp);
    printf("A7670C response: %s\n", resp);

    uart_write("AT+CGACT=1,1\r\n");
    uart_read(resp);
    printf("A7670C response: %s\n", resp);

    //uart_write("AT+CGPADDR\r\n");
    //uart_read(resp);
    //printf("A7670C response: %s\n", resp);
}

void call_SIM800l(void *arg)
{
    //sendData("AT");
    //sendData("AT+COPS?");
     //uart_read_bytes(UART_NUM_2, (uint8_t*)buffer, RX_BUF_SIZE, 100 / portTICK_PERIOD_MS);
    uart_write("ATD0353375794;\r\n");
     uart_read(resp);
   // vTaskDelay(pdMS_TO_TICKS(5000));
    //sendData("ATH\r\n"); // kết thúc cuộc gọi
    // uart_read_bytes(UART_NUM_2, (uint8_t*)buffer, RX_BUF_SIZE, 100 / portTICK_PERIOD_MS);
    //vTaskDelay(pdMS_TO_TICKS(1000));
}