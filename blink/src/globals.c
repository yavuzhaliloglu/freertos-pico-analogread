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
TaskHandle_t xWatchdogHandle;

SemaphoreHandle_t xFlashMutex;
SemaphoreHandle_t xFIFOMutex;
SemaphoreHandle_t xVRMSLastValuesMutex;
SemaphoreHandle_t xVRMSThresholdMutex;
SemaphoreHandle_t xThresholdSetFlagMutex;

const uint16_t pattern_idle[] = {0, 1000};

// Error Patterns
const uint16_t led_pattern_uart_not_readable[] = {50, 950}; // 1 Short
const uint16_t led_pattern_message_timeout[] = {250, 750}; // 1 Long
const uint16_t led_pattern_invalid_request_mode[] = {50, 100, 50, 800}; // 2 Short
const uint16_t led_pattern_invalid_serial_number[] = {250, 100, 250, 400}; // 2 Long
const uint16_t led_pattern_flash_mutex_not_taken[] = {50, 100, 50, 100, 50, 650}; // 3 Short
const uint16_t led_pattern_fifo_mutex_not_taken[] = {25, 50, 25, 900}; // Heartbeat (2 Fast)
const uint16_t led_pattern_vrms_values_mutex_not_taken[] = {50, 100, 250, 600}; // Short-Long
const uint16_t led_pattern_vrms_threshold_mutex_not_taken[] = {250, 100, 50, 600}; // Long-Short
const uint16_t led_pattern_threshold_set_mutex_not_taken[] = {25, 25, 25, 25, 25, 25, 25, 25, 25, 775}; // 5 Fast
const uint16_t led_pattern_rx_buffer_overflow_isr[] = {50, 50, 50, 50, 50, 50, 250, 450}; // 3 Fast, 1 Long

const LedPattern patterns[] = {
    {pattern_idle, 2},
    {led_pattern_uart_not_readable, 2},
    {led_pattern_message_timeout, 2},
    {led_pattern_invalid_request_mode, 4},
    {led_pattern_invalid_serial_number, 4},
    {led_pattern_flash_mutex_not_taken, 6},
    {led_pattern_fifo_mutex_not_taken, 4},
    {led_pattern_vrms_values_mutex_not_taken, 4},
    {led_pattern_vrms_threshold_mutex_not_taken, 4},
    {led_pattern_threshold_set_mutex_not_taken, 10},
    {led_pattern_rx_buffer_overflow_isr, 8}};

volatile int current_pattern_id = 0;

// Watchdog Variables
volatile uint32_t task_health_flags = 0;
