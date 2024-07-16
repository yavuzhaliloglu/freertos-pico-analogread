#ifndef DEFINES_H
#define DEFINES_H

//    36kb                236kB 256kB                    512kB                                                                                                                                         2048kB
// |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
// |   |                    | | |                        | | |                                                                                                                                           |
// | B |      Main Program  |X|T|      OTA Program       |N|S|                                                            Records                                                                        |
// |   |                    | | |                        | | |                                                                                                                                           |
// |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
//                          240kB                      508kB 516kB
//  B -> Bootloader
//  T -> Threshold Contents
//  N -> Serial Number Contents
//  S -> Sector Contents
//  X -> Threshold Sector and Variable Contents

// FLASH DEFINES

// this is the start offset of the program
#define FLASH_PROGRAM_OFFSET 36 * 1024
// threshold values offset
#define FLASH_THRESHOLD_INFO_OFFSET (256 * 1024) - (5 * FLASH_SECTOR_SIZE)
// threshold values offset
#define FLASH_THRESHOLD_OFFSET (256 * 1024) - (4 * FLASH_SECTOR_SIZE)
// this is the start offset of OTA program will written
#define FLASH_REPROGRAM_OFFSET 256 * 1024
// reset count offset
#define FLASH_RESET_COUNT_OFFSET (512 * 1024) - (2 * FLASH_SECTOR_SIZE)
// this is the start offset of device serial number information
#define FLASH_SERIAL_OFFSET (512 * 1024) - FLASH_SECTOR_SIZE
// this is the start offset of sector information that load profile records will written
#define FLASH_SECTOR_OFFSET 512 * 1024
// this is the start offset of load profile records will written
#define FLASH_DATA_OFFSET (512 * 1024) + FLASH_SECTOR_SIZE
// this is the size of reprogram area
#define FLASH_PROGRAM_SIZE 220 * 1024
// repgrogram area size
#define FLASH_REPROGRAM_SIZE 256 * 1024 - FLASH_SECTOR_SIZE
// this is the size of OTA program block will written to flash. it has to be multiple size of flash area.
#define FLASH_RPB_BLOCK_SIZE 7 * FLASH_PAGE_SIZE
// this is the count of total sectors in flash expect first 512kB + 4kB of flash (main program(256kB), OTA program(256kB), sector information(4kB))
#define FLASH_TOTAL_SECTORS 380
// this is the size of one load profile record
#define FLASH_RECORD_SIZE 16
// this is the count of total records can be kept in a flash
#define FLASH_TOTAL_RECORDS (PICO_FLASH_SIZE_BYTES - (FLASH_DATA_OFFSET)) / FLASH_RECORD_SIZE

// UART DEFINES

// MD5 checksum length
#define MD5_DIGEST_LENGTH 16
// This is the required number for the watchdog timer.
#define ENTRY_MAGIC 0xb105f00d
// UART ID for communication
#define UART0_ID uart0
// Baud Rate for UART. 300 bits per second.
#define BAUD_RATE 300
// Those are TX (Transmit) and RX (Receiver) pins for UART
#define UART0_TX_PIN 0
#define UART0_RX_PIN 1
// Data format to send UART includes 7 data bit (ASCII)
#define DATA_BITS 7
// 1 Stop bit for transfer format
#define STOP_BITS 1
// Even parity bit for transfer format (Check 1's in data)
#define PARITY UART_PARITY_EVEN
// UART Task priority for the FreeRTOS Kernel
#define UART_TASK_PRIORITY 3
// Stack size for UART Task
#define UART_TASK_STACK_SIZE (1024 * 3)

// RESET DEFINES

// Reset Pin Select
#define RESET_PULSE_PIN 2
// Standby Time for the Task
#define INTERVAL_MS 60000

// THRESHOLD PIN DEFINE
#define THRESHOLD_PIN 17

// ADC DEFINES

// samples to collect from ADC Pin
#define VRMS_SAMPLE 500
// VRMS buffer size to calculate min, max and mean values and write to flash
#define VRMS_BUFFER_SIZE 15
// VRMS values buffer size, is the size of calculated VRMS values in a minute
#define VRMS_VALUES_BUFFER_SIZE 60
//
#define CLOCK_DIV 4 * 9600
// ADC Voltage Pin
#define ADC_READ_PIN 26
// ADC BIAS Voltage Pin
#define ADC_BIAS_PIN 27
// ADC Voltage Input
#define ADC_SELECT_INPUT 0
// ADC BIAS Input
#define ADC_BIAS_INPUT 1
// BIAS Sample count
#define BIAS_SAMPLE 100

// RTC DEFINES

#define PT7C4338_REG_SECONDS 0x00
// I2C Port Select
#define I2C_PORT i2c0
// I2C Address
#define I2C_ADDRESS 0x68
// SDA and SCL pins for I2C
#define RTC_I2C_SDA_PIN 20
#define RTC_I2C_SCL_PIN 21

#endif
