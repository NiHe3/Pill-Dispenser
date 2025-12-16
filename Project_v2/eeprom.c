#include <stdio.h>
#include <string.h>

#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "eeprom.h"

// initialise i2c0 pin
void init_i2c() {
    i2c_init(I2C, BAUD_RATE);
    for (int i = 0; i < I2C_SIZE; i++) {
        gpio_set_function(i2cs[i], GPIO_FUNC_I2C);
        gpio_pull_up(i2cs[i]);
    }
}

// write an array of data to eeprom
void write_byte(uint16_t const address, uint8_t const *value, size_t length) {
    uint8_t buffer[EEPROM_ADDR_LEN + length];

    // MSB (Most Significant Byte) address
    buffer[0] = address >> 8;
    // LSB (Least Significant Byte) address
    buffer[1] = address & 0xFF;

    // copy the data to be written after the address bytes
        memcpy(buffer + EEPROM_ADDR_LEN, value, length);
    // Send 3 bytes:
    // - 2 bytes of address
    // - 1 byte of data
    i2c_write_blocking(I2C, EEPROM_ADDRESS, buffer, length + EEPROM_ADDR_LEN, false);
    sleep_ms(WRITE_BYTE_SLEEP_MS * (length + EEPROM_ADDR_LEN));
}

// read an array of data from eeprom
uint16_t read_byte(uint16_t const address, uint8_t *value, size_t length) {
    uint8_t buffer[EEPROM_ADDR_LEN];
    int bytes_read = 0;

    buffer[0] = address >> 8; // MSB
    buffer[1] = address & 0xFF; // LSB
    i2c_write_blocking(I2C, EEPROM_ADDRESS, buffer, EEPROM_ADDR_LEN,true);
    // read the data directly into buffer
    bytes_read = i2c_read_blocking(I2C, EEPROM_ADDRESS, value, length, false);
    return bytes_read;
}

// return data stored in eeprom
uint16_t return_stored_value(uint16_t address) {
    uint8_t value_buf[2];
    read_byte(address, value_buf, 2);
    return (uint16_t)(value_buf[0] << 8) | value_buf[1];
}

// check error log in eeprom
uint16_t crc16(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;
    while (length--) {
        uint8_t i = crc >> 8 ^ *data++;
        i ^= i >> 4;
        crc = (crc << 8) ^ ((uint16_t)(i << 5) ^ (uint16_t) i);
    }
    return crc;
}

// write a string log to eeprom. If log entries reach maximum, the log is erased before write to eeprom
void write_log_entry(const char *str, uint8_t *index) {
    // check for log overflow and erase if needed
    if (*index >= MAX_ENTRIES) {
        printf("Maximum log entries reached (%d). Erasing logs\n", MAX_ENTRIES);
        erase_logs(index);
    }
    size_t string_length = strlen(str) + 1; //include NULL terminator

    if (string_length > STRLEN_EEPROM) {
        string_length = STRLEN_EEPROM;
    }
    uint8_t log_buf[LOG_ENTRY_SIZE]; // Use full entry size to ensure buffer doesn't overflow

    //copy string to uint8_t array
    for (int a = 0; a < strlen(str); ++a) {
        log_buf[a] = (uint8_t) str[a];
    }
    log_buf[strlen(str)] = '\0';

    //add CRC to log buffer
    uint16_t crc = crc16(log_buf, string_length);
    log_buf[string_length] = (uint8_t)(crc >> 8);
    log_buf[string_length + 1] = (uint8_t)crc;         //check again the size length

    //write to EEPROM
    uint16_t write_address = (uint16_t) FIRST_ADDRESS + (*index * (uint16_t) LOG_ENTRY_SIZE);
    if (write_address < LOG_ENTRY_SIZE * MAX_ENTRIES) {
        write_byte(write_address, log_buf, LOG_ENTRY_SIZE);

        // update log index in ram and eeprom
        *index += 1;
        write_byte(LOG_INDEX_ADDR, index, 1);
    }
}

// read all the log entries that are valid from eeprom using crc to check
void read_log_entry(uint8_t index) {
    printf("Reading log entries (%d total)\n", index);

    for (int i = 0; i < index; ++i) {
        uint8_t read_buf[LOG_ENTRY_SIZE];
        uint16_t read_address = (uint16_t) FIRST_ADDRESS + (i * (uint16_t) LOG_ENTRY_SIZE);

        if (read_byte(read_address, read_buf, LOG_ENTRY_SIZE) != LOG_ENTRY_SIZE) {
            fprintf(stderr, "Error reading EEPROM block at address %u\n", read_address);
                break;
        }
        int term_zero_index = 0;
        while (read_buf[term_zero_index] != '\0') {
            term_zero_index++;
        }
        if (read_buf[0] != 0 && crc16(read_buf, (term_zero_index + 3)) == 0 && term_zero_index < (LOG_ENTRY_SIZE - 2)) {
            printf("Log entry index %d: ", i);
            int b_index = 0;
            while (read_buf[b_index]) {
                printf("%c", read_buf[b_index++]);
            }
            printf("\n");
        } else {
            printf("Invalid or empty log entry at index %d\n", i);
            break; // Stop if an invalid entry is encountered
        }
    }
    printf("\nStop\n");
}

// erase all log entries in eeprom
void erase_logs(uint8_t *log_index) {
    printf("Erase the log messages\n");
    for (int i = 0; i < MAX_ENTRIES; ++i) {
        uint16_t write_address = FIRST_ADDRESS + (i * LOG_ENTRY_SIZE);
        // printf("%u  ", write_address);
        uint8_t buffer[] = {00};
        write_byte(write_address, buffer, 2);
    }
    *log_index = 0;
    write_byte(LOG_INDEX_ADDR, log_index, 2);
    printf("\n");
}
