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
    WriteProgram = 3
};
enum ListeningStates
{
    DataError = -1,
    Reading = 0,
    TimeSet = 1,
    DateSet = 2,
    ReProgram = 3,
    ProductionInfo = 4
};
volatile TaskHandle_t xTaskToNotify_UART = NULL;
enum States state = Greeting;
uint16_t max_baud_rate = 9600;
uint8_t rx_buffer[256] = {};
uint8_t rx_buffer_len = 0;
uint8_t reading_state_start_time[10];
uint8_t reading_state_end_time[10];
volatile uint8_t test_flag = 0;

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
uint8_t data_pck[9] = {0};
int rpb_len = 0;
int ota_block_count = 0;
int data_cnt = 0;
int reprogram_size = 0;

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

#endif
