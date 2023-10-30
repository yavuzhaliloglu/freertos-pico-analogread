#ifndef DEFINES_H
#define DEFINES_H

// FLASH DEFINES

// this is the start offset of the program
#define FLASH_PROGRAM_OFFSET 36 * 1024
#define FLASH_PROGRAM_SIZE 220 * 1024
// this is the start offset of sector information that load profile records will written
#define FLASH_SECTOR_OFFSET 512 * 1024
// this is the start offset of device serial number information
#define FLASH_SERIAL_OFFSET 512 * 1024 - FLASH_SECTOR_SIZE
// this is the start offset of OTA program will written
#define FLASH_REPROGRAM_OFFSET 256 * 1024
#define FLASH_REPROGRAM_SIZE 256 * 1024 - FLASH_SECTOR_SIZE
// this is the size of OTA program block will written to flash. it has to be multiple size of flash area.
#define FLASH_RPB_BLOCK_SIZE 7 * FLASH_PAGE_SIZE
// this is the count of total sectors in flash expect first 512kB + 8kB of flash (main program(256kB), OTA program(256kB), sector information(4kB))
#define FLASH_TOTAL_SECTORS 382
// this is the start offset of load profile records will written
#define FLASH_DATA_OFFSET (512 * 1024) + FLASH_SECTOR_SIZE
// this is the size of one load profile record
#define FLASH_RECORD_SIZE 16
// this is the count of total records can be kept in a flash
#define FLASH_TOTAL_RECORDS (PICO_FLASH_SIZE_BYTES - (FLASH_DATA_OFFSET)) / FLASH_RECORD_SIZE

#define MD5_DIGEST_LENGTH 16

// UART DEFINES
#define ENTRY_MAGIC 0xb105f00d
#define UART0_ID uart0 // UART0 for RS485
#define BAUD_RATE 300
#define UART0_TX_PIN 0
#define UART0_RX_PIN 1
#define DATA_BITS 7
#define STOP_BITS 1
#define PARITY UART_PARITY_EVEN
#define UART_TASK_PRIORITY 3
#define UART_TASK_STACK_SIZE (1024 * 3)
#define DEVICE_PASSWORD "12345678"

// RESET PIN DEFINE
#define RESET_PULSE_PIN 2
#define INTERVAL_MS 60000

// ADC DEFINES
#define VRMS_SAMPLE 500
#define VRMS_BUFFER_SIZE 15
#define VRMS_SAMPLING_PERIOD 60000
#define CLOCK_DIV 4 * 9600
#define ADC_READ_PIN 26
#define ADC_BIAS_PIN 27
#define ADC_SELECT_INPUT 0
#define ADC_BIAS_INPUT 1
#define DEBUG 1
#define BIAS_SAMPLE 100

// RTC DEFINES
#define PT7C4338_REG_SECONDS 0x00
#define RTC_SET_PERIOD_MIN 10
#define I2C_PORT i2c0
#define I2C_ADDRESS 0x68
#define RTC_I2C_SDA_PIN 20
#define RTC_I2C_SCL_PIN 21

#endif
