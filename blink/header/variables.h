#ifndef VARIABLES_H
#define VARIABLES_H

#include "defines.h"

// ADC VARIABLES

// these 3 variables keeps max,m,n and mean values of vrms_buffer content
uint8_t vrms_max = 0;
uint8_t vrms_min = 0;
uint8_t vrms_mean = 0;
uint8_t vrms_max_dec = 0;
uint8_t vrms_min_dec = 0;
uint8_t vrms_mean_dec = 0;
// this is a buffer that keeps samples in ADC FIFO in ADC Input 1 to calculate VRMS value
uint16_t sample_buffer[VRMS_SAMPLE];
// this is a buffer that keeps samples in ADC FIFO in ADC Input 0 to calculate BIAS Voltage
uint16_t bias_buffer[BIAS_SAMPLE];
// this flag variable is used to detect if time is changed. If time is changed in device, that flag provides to align the task's execution time to beginning of next minute again
volatile uint8_t time_change_flag;
// vrms threshold value
uint16_t vrms_threshold;
// threshold flag value, used for set threshold pin and hold it until command comes and resets it
uint8_t threshold_set_before = 0;

uint8_t vrms_buffer_count = 0;
double vrms_buffer[VRMS_BUFFER_SIZE] = {0};

// UART VARIABLES

// States type keeps 4 value and these values are meaning like:
// Greeting: Initial state. When modem sends a message just like /?[SERIALNUM]!\r\n in this state, program handles that message and if serial number and message format is true, device will response and set the state setting.
// Setting: This state handles to set Baud rate and also to send readout data. When coming message is [ACK]0Z0\r\n, that means device will be in readout mode and send readout data. If message is [ACK]0Z1\r\n, that means device is in programming mode and state is changed to Listening.
// Listening: This state accepts request messages and process these messages, like reading, changing time and date, production info etc. In this state, messages handles in different states again, THese states are called ListeningStates which is described below.
// ReProgram: This state handles task behaviors, cleaning the new program flash area and sends the ACK message
enum States
{
    Greeting = 0,
    Setting = 1,
    Listening = 2,
    ReProgram = 3
};

// ListeningStates type keeps 7 values are meaning like:
// DataError: In this state, device sends NACK message
// Reading: In this state, device parses load profile data's date values, if exist, and show records according to dates.
// TimeSet: This state handles to change time in device
// DateSet: This state handles to change date in device
// WriteProgram: In this state, device accepts all the characters as program variables and writes them in selected flash area, after writing device reboots itself
// ProductionInfo: In this state, device sends production info about this device
// Password: In this state, device controls password of this device. If password is correct, modem can change time and date in this device.
enum ListeningStates
{
    DataError = -1,
    Reading = 0,
    TimeSet = 1,
    DateSet = 2,
    WriteProgram = 3,
    ProductionInfo = 4,
    Password = 5,
    SetThreshold = 6,
    GetThreshold = 7,
    ThresholdPin = 8
};
//
volatile TaskHandle_t xTaskToNotify_UART = NULL;
// state varible that keeps current state
enum States state = Greeting;
// this variable is represent max baud rate that this device can reach.
uint16_t max_baud_rate = 19200;
uint8_t rx_buffer[256] = {0};
uint8_t rx_buffer_len = 0;
// this buffer stores start time for load profile data
uint8_t reading_state_start_time[14] = {0};
// this buffer stores end time for load profile data
uint8_t reading_state_end_time[14] = {0};
// this is a flag that controls if password is correct
bool password_correct_flag = false;

// SPI VARIABLES

// this is the current sector data variable in flash
uint8_t *flash_sector_content = (uint8_t *)(XIP_BASE + FLASH_SECTOR_OFFSET);
// this is the serial number variable in flash
uint8_t *serial_number = (uint8_t *)(XIP_BASE + FLASH_SERIAL_OFFSET);
// sector data variable keeps current sector to write records to flash
static uint16_t sector_data = 0;
// // threshold records sector data
static uint16_t th_sector_data = 0;
// Flash data structure is used to keep different formats of data. This struct includes character variables and also uint8_t integer variables to create a record for flash.
struct FlashData
{
    char year[2];
    char month[2];
    char day[2];
    char hour[2];
    char min[2];
    uint8_t max_volt;
    uint8_t max_volt_dec;
    uint8_t min_volt;
    uint8_t min_volt_dec;
    uint8_t mean_volt;
    uint8_t mean_volt_dec;
};
// This is a buffer that keeps record contents. When a record should be written in flash, current sector content is copied to this buffer, new record adds after last record, and this buffer is written to flash again.
struct FlashData flash_data[FLASH_SECTOR_SIZE / sizeof(struct FlashData)] = {0};
// Threshold data to write and read from flash correctly with struct type
struct ThresholdData
{
    char year[2];
    char month[2];
    char day[2];
    char hour[2];
    char min[2];
    uint16_t vrms;
    uint16_t variance;
    uint8_t padding[2];
};
// Threshold buffer that can keep 1 Sector size threshold record
struct ThresholdData flash_th_buf[FLASH_SECTOR_SIZE / sizeof(struct ThresholdData)] = {0};
// this is a buffer that stores 1792 bytes (7 * 256 bytes) new program data and writes it to flash when it's full.
uint8_t rpb[FLASH_RPB_BLOCK_SIZE] = {0};
// this small buffer keeps 8 bytes of program data and this buffer is converted 8-bit from 7-bit
uint8_t data_pck[8] = {0};
// this variables keeps length of characters in rpb buffer
int rpb_len = 0;
// this variables stores that how many blocks of new program data are written in flash. It is also used to jump next block while programming flash with new block data
int ota_block_count = 0;
// this variables stores length of characters in data_pck buffer
int data_cnt = 0;
// this flag variable is used to write reamining contents in rpb buffer to flash
bool is_program_end = false;

// RTC VARIABLES

// this buffer keeps current datetime value
char datetime_buffer[64];
// this is a string to keep datetime value as string
char *datetime_str = &datetime_buffer[0];
// datetime variable to keep current time
datetime_t current_time = {
    .year = 2023,
    .month = 06,
    .day = 05,
    .dotw = 5, // 0 is Sunday, so 5 is Friday
    .hour = 15,
    .min = 46,
    .sec = 50};

// This is ADCTaskHandler. This handler is used to delete ADCReadTask in ReProgram State.
TaskHandle_t xADCHandle;

#endif
