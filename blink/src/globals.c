#include "header/project_globals.h"

// =============================================================================
// GLOBAL DEĞİŞKEN TANIMLAMALARI
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
volatile TaskHandle_t xTaskToNotify_UART = NULL;
enum States state = Greeting;
uint8_t rx_buffer[RX_BUFFER_SIZE] = {0};
uint8_t rx_buffer_len = 0;
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
    .year = 2023,
    .month = 06,
    .day = 05,
    .dotw = 5, // 0 is Sunday, so 5 is Friday
    .hour = 15,
    .min = 46,
    .sec = 50
};

// FreeRTOS HANDLES
TaskHandle_t xADCHandle;
TaskHandle_t xADCSampleHandle;
TaskHandle_t xUARTHandle;
TaskHandle_t xResetHandle;
TaskHandle_t xGetRTCHandle;

SemaphoreHandle_t xFlashMutex;
SemaphoreHandle_t xFIFOMutex;
SemaphoreHandle_t xVRMSLastValuesMutex;
SemaphoreHandle_t xVRMSThresholdMutex;
SemaphoreHandle_t xThresholdSetFlagMutex;