#ifndef PROJECT_GLOBALS_H
#define PROJECT_GLOBALS_H

#include "FreeRTOS.h"
#include "hardware/flash.h"
#include "pico/util/datetime.h"
#include "semphr.h"
#include "task.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "defines.h"
#include "project_conf.h"

extern volatile int current_pattern_id;
extern volatile bool play_once;

typedef struct
{
    uint16_t data[ADC_FIFO_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
} ADC_FIFO;

typedef struct
{
    uint8_t vrms_max;
    uint8_t vrms_min;
    uint8_t vrms_mean;
    uint8_t vrms_max_dec;
    uint8_t vrms_min_dec;
    uint8_t vrms_mean_dec;
} VRMS_VALUES_RECORD;

enum ListeningStates {
    BCCError = 0,
    DataError,
    Password,
    Reading,
    TimeSet,
    DateSet,
    ProductionInfo,
    SetThreshold,
    GetThreshold,
    ThresholdPin,
    GetSuddenAmplitudeChange,
    ReadTime,
    ReadDate,
    ReadSerialNumber,
    ReadLastVRMSMax,
    ReadLastVRMSMin,
    ReadLastVRMSMean,
    ReadResetDates,
    GetThresholdObis,
    BreakMessage
};

struct FlashData {
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

struct ThresholdData {
    char year[2];
    char month[2];
    char day[2];
    char hour[2];
    char min[2];
    char sec[2];
    uint16_t vrms;
    uint16_t variance;
};

struct AmplitudeChangeData {
    char year[2];
    char month[2];
    char day[2];
    char hour[2];
    char min[2];
    char sec[2];
    uint8_t sample_buffer[ADC_FIFO_SIZE];
    float vrms_values_buffer[VRMS_SAMPLE_SIZE / SAMPLE_SIZE_PER_VRMS_CALC];
    uint16_t variance;
    uint8_t padding[42];
};

struct AmplitudeChangeTimerCallbackParameters {
    float vrms_values_buffer[VRMS_SAMPLE_SIZE / SAMPLE_SIZE_PER_VRMS_CALC];
    uint16_t variance;
    size_t adc_fifo_size;
    size_t vrms_values_buffer_size_bytes;
};

typedef struct {
    const uint16_t *sequence;
    uint16_t length;
} LedPattern;

// ADC VARIABLES
extern ADC_FIFO adc_fifo;
extern uint8_t load_profile_record_period;
extern volatile float vrms_max_last;
extern volatile float vrms_min_last;
extern volatile float vrms_mean_last;
extern uint16_t vrms_threshold;
extern uint8_t threshold_set_before;
extern float bias_voltage;

// UART VARIABLES
extern bool password_correct_flag;

// SPI VARIABLES
extern uint8_t serial_number[SERIAL_NUMBER_SIZE + 1];
extern uint16_t sector_data;
extern uint16_t th_sector_data;
extern struct FlashData flash_data[FLASH_SECTOR_SIZE / sizeof(struct FlashData)];
extern struct ThresholdData th_flash_buf[FLASH_SECTOR_SIZE / sizeof(struct ThresholdData)];

// RTC VARIABLES
extern char datetime_buffer[64];
extern char *datetime_str;
extern datetime_t current_time;

// FreeRTOS HANDLES
extern TaskHandle_t xADCHandle;
extern TaskHandle_t xADCSampleHandle;
extern TaskHandle_t xUARTHandle;
extern TaskHandle_t xResetHandle;
extern TaskHandle_t xGetRTCHandle;
extern TaskHandle_t xStatusLedHandle;
extern TaskHandle_t xWatchdogHandle;

extern SemaphoreHandle_t xFlashMutex;
extern SemaphoreHandle_t xFIFOMutex;
extern SemaphoreHandle_t xVRMSLastValuesMutex;
extern SemaphoreHandle_t xVRMSThresholdMutex;
extern SemaphoreHandle_t xThresholdSetFlagMutex;

extern const uint16_t pattern_idle[];

// Error Patterns
extern const uint16_t led_pattern_uart_not_readable[];
extern const uint16_t led_pattern_message_timeout[];
extern const uint16_t led_pattern_invalid_request_mode[];
extern const uint16_t led_pattern_invalid_serial_number[];
extern const uint16_t led_pattern_flash_mutex_not_taken[];
extern const uint16_t led_pattern_fifo_mutex_not_taken[];
extern const uint16_t led_pattern_vrms_values_mutex_not_taken[];
extern const uint16_t led_pattern_vrms_threshold_mutex_not_taken[];
extern const uint16_t led_pattern_threshold_set_mutex_not_taken[];
extern const uint16_t led_pattern_rx_buffer_overflow_isr[];
extern const LedPattern patterns[];

#endif