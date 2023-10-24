#ifndef VARIABLES_H
#define VARIABLES_H

#include "defines.h"

// ADC VARIABLES
uint8_t vrms_max = 0;
uint8_t vrms_min = 0;
uint8_t vrms_mean = 0;
uint16_t sample_buffer[VRMS_SAMPLE];
TickType_t adc_remaining_time = 0;
volatile uint8_t time_change_flag;

// UART VARIABLES
enum States
{
    Greeting = 0,
    Setting = 1,
    Listening = 2,
    ReProgram = 3
};
enum ListeningStates
{
    DataError = -1,
    Reading = 0,
    TimeSet = 1,
    DateSet = 2,
    WriteProgram = 3,
    ProductionInfo = 4,
    Password = 5
};
volatile TaskHandle_t xTaskToNotify_UART = NULL;
enum States state = Greeting;
uint16_t max_baud_rate = 9600;
uint8_t rx_buffer[256] = {};
uint8_t rx_buffer_len = 0;
uint8_t reading_state_start_time[14] = {0};
uint8_t reading_state_end_time[14] = {0};
volatile uint8_t test_flag = 0;
bool reprogram_is_done = false;
bool is_adc_block = false;

// SPI VARIABLES
uint8_t *flash_sector_content = (uint8_t *)(XIP_BASE + FLASH_SECTOR_OFFSET);
static uint8_t sector_data = 0;
struct FlashData
{
    char year[2];
    char month[2];
    char day[2];
    char hour[2];
    char min[2];
    char sec[2];
    uint8_t max_volt;
    uint8_t min_volt;
    uint8_t mean_volt;
    uint8_t eod_character;
};
struct FlashData flash_data[FLASH_SECTOR_SIZE / sizeof(struct FlashData)] = {0};
uint8_t rpb[FLASH_RPB_BLOCK_SIZE] = {0};
uint8_t data_pck[8] = {0};
int rpb_len = 0;
int ota_block_count = 0;
int data_cnt = 0;
bool is_program_end = false;
int reprogram_size = 0;
uint8_t *serial_number = (uint8_t *)(XIP_BASE + FLASH_SERIAL_OFFSET);

// RTC VARIABLES
char datetime_buffer[64];
char *datetime_str = &datetime_buffer[0];
datetime_t current_time = {
    .year = 2020,
    .month = 06,
    .day = 05,
    .dotw = 5, // 0 is Sunday, so 5 is Friday
    .hour = 15,
    .min = 45,
    .sec = 00};


TaskHandle_t xDebugHandle;
TaskHandle_t xADCHandle;

void printBinary(uint8_t value) {
    uint8_t debug2[4] = {0};
    snprintf(debug2,4,"%02X ",value);
    printf("%s",debug2);

    for(int i = 7; i >= 0; i--) {
        uint8_t debug[2] = {0};
        snprintf(debug,2,"%d", (value & (1 << i)) ? 1 : 0);
        printf("%s",debug);
    }
    printf("\n");
}

#endif
