//
// Created by Quinn on 1.12.2025.
//

#ifndef MAIN_H
#define MAIN_H

#include <stdio.h>
#include "hardware/i2c.h"


#define I2C i2c0
// I2C pins
#define I2C_SDA 16 // Serial Data Line
#define I2C_SCL 17 // Serial Clock Line
#define I2C_SIZE 2
#define I2C_MEMORY_SIZE 32768 // total memory size of eeprom
#define BAUD_RATE 9600 // communication speed, set to 100 kHz

#define FIRST_ADDRESS 0
#define LOG_INDEX_ADDR 32767
#define CALIBRATION_STEPS_ADDR 32766
#define DISPENSED_COUNT_ADDR 32765
#define DEVICE_STATE_ADDR 32764 // address for the current state

#define EEPROM_ADDRESS 0x50 // 7-bits EEPROM I2C address
#define EEPROM_ADDR_LEN 2
#define LOG_ENTRY_SIZE 64 // total size of a single log entry block in bytes
#define MAX_ENTRIES 32 // maximum number of log entries
#define STRLEN_EEPROM 62 // maxium length of the string data itself

#define CRC_CHAR 2 // size fo the crc-16 checksum appended to the log entry (2 bytes)
#define WRITE_BYTE_SLEEP_MS 5 // Write byte sleep ms

static const uint i2cs[] = {I2C_SDA, I2C_SCL};

void init_i2c();
void write_byte(uint16_t const address, uint8_t const *value, size_t length);
uint16_t read_byte(uint16_t const address, uint8_t *value, size_t length);
uint16_t return_stored_value(uint16_t address);
uint16_t crc16(const uint8_t *data, size_t length);
void write_log_entry(const char *str, uint8_t *index);
void read_log_entry(uint8_t index);
void erase_logs(uint8_t *log_index);

#endif