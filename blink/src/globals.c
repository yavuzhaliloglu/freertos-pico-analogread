#include "header/project_globals.h"

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================

// ADC VARIABLES
ADC_FIFO adc_fifo;
uint8_t load_profile_record_period = 15;
volatile float vrms_max_last = 0.0;
volatile float vrms_min_last = 0.0;
volatile float vrms_mean_last = 0.0;
uint16_t vrms_threshold = 5;
uint8_t threshold_set_before = 0;
float bias_voltage = 0;

// UART VARIABLES
bool password_correct_flag = false;

// SPI VARIABLES
uint8_t serial_number[SERIAL_NUMBER_SIZE + 1] = {0};
uint16_t sector_data = 0;
uint16_t th_sector_data = 0;
struct FlashData flash_data[FLASH_SECTOR_SIZE / sizeof(struct FlashData)] = {0};
struct ThresholdData th_flash_buf[FLASH_SECTOR_SIZE / sizeof(struct ThresholdData)] = {0};

// RTC VARIABLES
char datetime_buffer[64];
char *datetime_str = &datetime_buffer[0];
datetime_t current_time = {
    .year = 2026,
    .month = 01,
    .day = 01,
    .dotw = 4, // 0 is Sunday, so 5 is Friday
    .hour = 0,
    .min = 0,
    .sec = 0};

// FreeRTOS HANDLES
TaskHandle_t xADCHandle;
TaskHandle_t xADCSampleHandle;
TaskHandle_t xUARTHandle;
TaskHandle_t xResetHandle;
TaskHandle_t xGetRTCHandle;
TaskHandle_t xStatusLedHandle;

SemaphoreHandle_t xFlashMutex;
SemaphoreHandle_t xFIFOMutex;
SemaphoreHandle_t xVRMSLastValuesMutex;
SemaphoreHandle_t xVRMSThresholdMutex;
SemaphoreHandle_t xThresholdSetFlagMutex;

const uint16_t pattern_idle[] = {0, 1000};

// Error Patterns
const uint16_t led_pattern_uart_not_readable[] = {50, 950};
const uint16_t led_pattern_message_timeout[] = {50, 100, 50, 800};
const uint16_t led_pattern_invalid_request_mode[] = {50, 100, 50, 100, 50, 650};
const uint16_t led_pattern_invalid_serial_number[] = {50, 100, 50, 100, 50, 100, 50, 500};
const uint16_t led_pattern_fifo_mutex_not_taken[] = {50, 100, 50, 100, 50, 100, 50, 100, 50, 500};

const LedPattern patterns[] = {
    {pattern_idle, 2},
    {led_pattern_uart_not_readable, 2},
    {led_pattern_message_timeout, 4},
    {led_pattern_invalid_request_mode, 6},
    {led_pattern_invalid_serial_number, 8},
    {led_pattern_fifo_mutex_not_taken, 10}};

volatile int current_pattern_id = 0;
