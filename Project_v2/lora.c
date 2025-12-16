//
// Created by Quinn on 5.12.2025.
//
#include <stdio.h>
#include <string.h>
#include "lora.h"
#include "hardware/gpio.h"
#include "pico/bootrom.h"

// initialise lora uart
void init_lora() {
    uart_init(UART, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(UART, DATA_BITS, STOP_BITS,PARITY);
}

// return a string received from uart within timeout
bool read_line(char *buffer, const int len, const int timeout_ms) {
    const uint32_t us = timeout_ms * 1000; // convert to microseconds
    // Wait for data to become available within timeout
    int i = 0;
    while (i < len - 1) {
        if (uart_is_readable_within_us(UART, us)) {
            const char c = uart_getc(UART);
            if (c == '\n') {
                break; // End of line
            }
            if (c != '\r') { // Ignore carriage return
                buffer[i++] = c;
            }
        }
        else {
            buffer[i] = '\0'; // Null-terminate resulting string
            return false; // No data received within timeout
        }
    }
    buffer[i] = '\0';
    return true;
}

// send first command 'AT' to check lora connection
void lora_first_at() {
    sleep_ms(2000);
    const uint8_t init_cmd[] = "AT\r\n";
    char receivedStr[LORA_BUF_LEN];
    bool success = false;

    for (size_t i = 0; i < 5; i++) {
        uart_write_blocking(UART, init_cmd, strlen((const char*)init_cmd));
        success = read_line(receivedStr, LORA_BUF_LEN, LORA_TIMEOUT_MS);

        if (success) {
            printf("Connected to LoRa module: %s\n", receivedStr);
            break;
        } else {
            printf("Module not responding!\n");
        }
    }
        if (!success) {
            printf("Failed to connect to LoRa module after 5 attempts.\n");
        }
    }

// send command to lora
void send_cmd (enum cmd_enum cmd) {
    uart_write_blocking(UART, (const uint8_t *) commands[cmd],
    strlen(commands[cmd]));
}

// return whether response received from lora within time limit
bool get_cmd_rsp(enum cmd_enum cmd, bool loading_bar) {
    char response[STRLEN_LORA];
    bool line_read = false;

    while (true) {
        line_read = read_line(response, STRLEN_LORA, UART_WAIT_US_FOR_RESP);
        if (!line_read) {
            printf("\nTimeout waiting for response.\n");
            break;
        }
        if (loading_bar) printf("#");
        if (strcmp(response, fail_rsp[cmd]) == 0) {
            printf("\nResponse: %s\n", response);
            return false;
        }
        if (strcmp(response, success_rsp[cmd]) == 0) return true;
    }
    return false;
}

// return whether lora connection succeeded
bool connect_network() {
    printf("Connecting to Lora server...\n");

    for (enum cmd_enum cmd = MODE; cmd <= JOIN; cmd++) {
        send_cmd(cmd);
        if (!get_cmd_rsp(cmd,true)) {
            fprintf(stderr,"\nLora command failed: %s""Connection failed.\n\n",commands[cmd]);
            return false;
        }
    }
    printf("LoRa connection successful.\n");
    return true;
}

// send message to lora
void send_msg(const char *content) {
    char data[STRLEN_LORA];
    sprintf(data,commands[MSG],content);
    uart_write_blocking(UART,(uint8_t *)data, strlen(data));
    get_cmd_rsp(MSG, false);
}
