//
// Created by Quinn on 5.12.2025.
//

#ifndef QIN_V2_LORA_H
#define QIN_V2_LORA_H

#include "hardware/uart.h"
#include "eeprom.h"

#define UART uart1 // LoRa module UART1
#define UART_TX_PIN 4 // UART0 TX (GP4) - to LoRa
#define UART_RX_PIN 5 // UART0 RX (GP5) - from LoRa

#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE
#define STRLEN_LORA 80
#define LORA_BUF_LEN 128
#define LORA_TIMEOUT_MS 1000
#define UART_WAIT_US_FOR_RESP 10000000 // 10s for each command
#define UAR_WAIT_US_FOR_A_STR 500000


static const char * commands[] =
{"AT\r\n",
 "AT+MODE=LWOTAA\r\n",
 "AT+KEY=APPKEY,\"9e60829e8cabb5e29c3be1ecd551671f\"\r\n",
 "AT+CLASS=A\r\n",
 "AT+PORT=8\r\n",
 "AT+JOIN\r\n",
 "AT+MSG=\"%s\"\r\n"};

static const char * success_rsp[] =
{"+AT: OK",
 "+MODE: LWOTAA",
 "+KEY: APPKEY 9E60829E8CABB5E29C3BE1ECD551671F",
 "+CLASS: A",
 "+PORT: 8",
 "+JOIN: Done",
 "+MSG: Done"
 };

static const char * fail_rsp[] =
{"unknown",
 "unknown",
 "unknown",
 "unknown",
 "unknown",
 "+JOIN: Join failed",
 "+MSG: Please join network first."};

enum cmd_enum {
    AT,
    MODE,
    APPKEY,
    CLASS,
    PORT,
    JOIN,
    MSG
};

void init_lora();
bool read_line(char *buffer, const int len, const int timeout_ms);
void lora_first_at();
bool connect_network();
void send_cmd (enum cmd_enum cmd);
void send_msg(const char *content);

#endif //QIN_V2_LORA_H