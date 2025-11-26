#ifndef DEFINES_H
#define DEFINES_H

//                                         512kB    528kB              588kB                   656kB            672kB
// |---------------------------------------------------------------------------------------------------------------------------------------------   ---------------------------------------------------------------|
// |                                      | | | | | |            | | | |                     | |               | |                                             |                                                   |
// |              Main Program            |P|N|P|S|P| LP Records |P|X|P|  Threshold Records  |P|  Reset Dates  |P|         Sudden Amplitude Change Records     |                     Empty Area                    | // NEW MEMORY MAP
// |                (508kb)               | | | | | |   (48kB)   | | | |       (64kB)        | |    (4kB)      | |                   (400kB)                   |                                                   |
// |---------------------------------------------------------------------------------------------------------------------------------------------   ---------------------------------------------------------------|
//                                          520kB              580kB                     652kB             668kB                                           1636kB

//  P -> Padding(Empty area) (4kB)
//  N -> Serial Number Contents (4kB)
//  S -> Sector Contents (4kB)
//  X -> Threshold Variable and Sector Contents (4kB)

// MESSAGES THAT THIS DEVICE ACCEPTS

// Request Message Without Serial Number and Flag:  /?!\r\n                                                     -> Length: 5
// Request Message Without Serial Number:           /?ALP!\r\n                                                  -> Length: 8
// Request Message with Serial Number and Flag:     /?ALP612400001!\r\n                                         -> Length: 17

// Reboot device message                            /?RBTDVC?\r\n                                               -> Length: 11
// Reset To Factory Settings message                /?RSTFS?\r\n                                                -> Length: 10

// Acknowledgement Message:                         [ACK]0ZX\r\n                                                -> Length: 6

// Password:                                        [SOH]P1[STX](12345678)[ETX][BCC]                            -> Length: 16
// Load Profile with Date:                          [SOH]R2[STX]P.01(24-07-13,13:00;24-07-14,14:00)[ETX][BCC]   -> Length: 41
// Load Profile without Date:                       [SOH]R2[STX]P.01(;)[ETX][BCC]                               -> Length: 13
// Time Set:                                        [SOH]W2[STX]0.9.1(13:00:00)[ETX][BCC]                       -> Length: 21
// Date Set:                                        [SOH]W2[STX]0.9.2(24-07-15)[ETX][BCC]                       -> Length: 21
// Production Information:                          [SOH]R2[STX]96.1.3()[ETX][BCC]                              -> Length: 14
// Set Threshold Value:                             [SOH]W2[STX]96.3.12(000)[ETX][BCC]                            -> Length: 16
// Get Threshold Value:                             [SOH]R2[STX]T.R.1()[ETX][BCC]                               -> Length: 13
// Set Threshold PIN:                               [SOH]W2[STX]T.P.1()[ETX][BCC]                               -> Length: 13
// Get Sudden Amplitude Change Records              [SOH]R2[STX]9.9.0()[ETX][BCC]                               -> Length: 13
// Read Current Time                                [SOH]R2[STX]0.9.1()[ETX][BCC]                               -> Length: 13
// Read Current Date                                [SOH]R2[STX]0.9.2()[ETX][BCC]                               -> Length: 13
// Read Serial Number                               [SOH]R2[STX]0.0.0()[ETX][BCC]                               -> Length: 13
// Read Last VRMS Max                               [SOH]R2[STX]32.7.0()[ETX][BCC]                              -> Length: 14
// Read Last VRMS Min                               [SOH]R2[STX]52.7.0()[ETX][BCC]                              -> Length: 14
// Read Last VRMS Mean                              [SOH]R2[STX]72.7.0()[ETX][BCC]                              -> Length: 14
// Read Reset Dates                                 [SOH]R2[STX]R.D.0()[ETX][BCC]                               -> Length: 13
// End Connection:                                  [SOH]B0[ETX]q                                               -> Length: 5

// FLASH DEFINES
// padding size
#define PADDING_SIZE FLASH_SECTOR_SIZE
// program size
#define FLASH_PROGRAM_SIZE (508 * 1024)
// this is the start offset of device serial number information
#define FLASH_SERIAL_NUMBER_ADDR FLASH_PROGRAM_SIZE + PADDING_SIZE
// this is the size of serial number flah area
#define FLASH_SERIAL_NUMBER_AREA_SIZE FLASH_SECTOR_SIZE
// this is the start offset of sector information that load profile records will written
#define FLASH_LOAD_PROFILE_LAST_SECTOR_DATA_ADDR (FLASH_SERIAL_NUMBER_ADDR + FLASH_SERIAL_NUMBER_AREA_SIZE) + PADDING_SIZE
// this is the size of sector information area
#define FLASH_LOAD_PROFILE_LAST_SECTOR_DATA_SIZE FLASH_SECTOR_SIZE
// this is the start offset of load profile records will written
#define FLASH_LOAD_PROFILE_RECORD_ADDR (FLASH_LOAD_PROFILE_LAST_SECTOR_DATA_ADDR + FLASH_LOAD_PROFILE_LAST_SECTOR_DATA_SIZE) + PADDING_SIZE
// this is the count of total sectors in flash
#define FLASH_LOAD_PROFILE_AREA_TOTAL_SECTOR_COUNT 12
// this is the size of load profile records
#define FLASH_LOAD_PROFILE_RECORD_AREA_SIZE (FLASH_LOAD_PROFILE_AREA_TOTAL_SECTOR_COUNT * FLASH_SECTOR_SIZE)
// threshold values offset (first 2 byte value is threshold value, second 2 byte value is threshold records sector value)
#define FLASH_THRESHOLD_PARAMETERS_ADDR (FLASH_LOAD_PROFILE_RECORD_ADDR + FLASH_LOAD_PROFILE_RECORD_AREA_SIZE) + PADDING_SIZE
// threshold parameters size
#define FLASH_THRESHOLD_PARAMETERS_SIZE FLASH_SECTOR_SIZE
// threshold values offset
#define FLASH_THRESHOLD_RECORDS_ADDR (FLASH_THRESHOLD_PARAMETERS_ADDR + FLASH_THRESHOLD_PARAMETERS_SIZE) + PADDING_SIZE
// threshold records area size
#define FLASH_THRESHOLD_RECORDS_SIZE (16 * FLASH_SECTOR_SIZE)
// reset count offset
#define FLASH_RESET_DATES_ADDR (FLASH_THRESHOLD_RECORDS_ADDR + FLASH_THRESHOLD_RECORDS_SIZE) + PADDING_SIZE 
// reset dates area size
#define FLASH_RESET_DATES_AREA_SIZE FLASH_SECTOR_SIZE
// amplitude change records offset
#define FLASH_AMPLITUDE_CHANGE_OFFSET (FLASH_RESET_DATES_ADDR + FLASH_RESET_DATES_AREA_SIZE) + PADDING_SIZE
// amplitude records size as sectors
#define FLASH_AMPLITUDE_RECORDS_TOTAL_SECTOR 100
// this is the size of one load profile record
#define FLASH_RECORD_SIZE (16)
// serial number size
#define SERIAL_NUMBER_SIZE 9

// UART DEFINES

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
// Stack sizes for tasks
#define ADC_READ_TASK_STACK_SIZE (5 * 1024)
#define UART_TASK_STACK_SIZE (2 * 1024)
#define WRITE_DEBUG_TASK_STACK_SIZE (3 * 256)
#define RESET_TASK_STACK_SIZE (3 * 256)
#define ADC_SAMPLE_TASK_STACK_SIZE (4 * 1024)
#define POWER_BLINK_STACK_SIZE (3 * 256)
// POWER LED
#define POWER_LED_PIN 18

// RESET DEFINES

// Reset Pin Select
#define RESET_PULSE_PIN 2
// Standby Time for the Task
#define INTERVAL_MS 60000

// THRESHOLD PIN DEFINE
#define THRESHOLD_PIN 17

// ADC DEFINES

#define ADC_FIFO_SIZE 4000
// samples to collect from ADC Pin
#define VRMS_SAMPLE_SIZE 2000
// sample size per vrms calculation
#define SAMPLE_SIZE_PER_VRMS_CALC 200
// VRMS buffer size to calculate min, max and mean values and write to flash
#define VRMS_BUFFER_SIZE 900
// ADC Voltage Pin
#define ADC_READ_PIN 26
// ADC BIAS Voltage Pin
#define ADC_BIAS_PIN 27
// ADC Voltage Input
#define ADC_VRMS_SAMPLE_INPUT 0
// ADC BIAS Input
#define ADC_BIAS_INPUT 1
// BIAS Sample count
#define BIAS_SAMPLE_SIZE 2000
// amplitude threshold
#define AMPLITUDE_THRESHOLD 5
// mean calculation window size
#define MEAN_CALCULATION_WINDOW_SIZE 20
// mean calculation shifting size
#define MEAN_CALCULATION_SHIFT_SIZE 5

// RTC DEFINES

#define PT7C4338_REG_SECONDS 0x00
// I2C Port Select
#define I2C_PORT i2c0
// I2C Address
#define I2C_ADDRESS 0x68
// SDA and SCL pins for I2C
#define RTC_I2C_SDA_PIN 20
#define RTC_I2C_SCL_PIN 21

// SPECIAL CHARACTERS
#define SOH 0x01
#define STX 0x02
#define ETX 0x03
#define ACK 0x06
#define NACK 0x15

#endif
