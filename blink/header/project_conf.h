#ifndef PROJECT_CONF_H
#define PROJECT_CONF_H

#include <stdio.h>

// version
#define HARDWARE_VERSION 3

// Device Password (will be written to flash)
#define DEVICE_PASSWORD "12345678"
// Device software version number
#define SOFTWARE_VERSION "V1.5.0"
// production date of device (yy-mm-dd)
#define PRODUCTION_DATE "26-06-12"
// Debugs
#define DEBUG 1
// vrms multiplier value -- trafo ile olculmus gercek bolucu orani, donanim
// surumune bagli. Ikisi de tek nokta (32.5 V multimetre) uzerinden cikarildi:
//   v2: cihaz 33.86 V (carpan 150 ile) -> 150 * 32.5/33.86 = 144.0
//   v3: cihaz 32.92 V (carpan 150 ile) -> 150 * 32.5/32.92 = 148.1
// Ikinci bir trafo kademesiyle dogrusallik henuz dogrulanmadi.
#if HARDWARE_VERSION >= 3
#define VRMS_MULTIPLICATION_VALUE 148.1f
#else
#define VRMS_MULTIPLICATION_VALUE 144.0f
#endif
// watchdog timeout ms to reset device
// RP2040 HW watchdog is clamped to ~8388ms (24-bit counter at 2 ticks/us).
// We stay safely below this hard limit; longer task blocks must chunk + heartbeat.
#define WATCHDOG_TIMEOUT_MS 8000
#define WATCHDOG_CHECK_PERIOD_MS 5000
// RX Buffer Size
#define RX_BUFFER_SIZE 256
// identification response buffer size
#define IDENTIFICATION_RESPONSE_BUFFER_SIZE 64
// meter identify parameters
#define METER_VERSION 2
#define METER_MAX_SUPPORTED_BAUDRATE 6
#define METER_FLAG_CODE "ALP"
// max message retry count
#define MAX_MESSAGE_RETRY_COUNT 3
// request modes
#define REQUEST_MODE_SHORT_READ 0x36
#define REQUEST_MODE_LONG_READ  0x30
#define REQUEST_MODE_PROGRAMMING 0x31
// Functions of Devices

#define CONF_LOAD_PROFILE_ENABLED 1
#define CONF_TIME_SET_ENABLED 1
#define CONF_DATE_SET_ENABLED 1
#define CONF_PRODUCTION_INFO_ENABLED 1
#define CONF_THRESHOLD_ENABLED 1
#define CONF_THRESHOLD_PIN_ENABLED 0
#define CONF_SUDDEN_AMPLITUDE_CHANGE_ENABLED 0
#define CONF_TIME_READ_ENABLED 1
#define CONF_DATE_READ_ENABLED 1
#define CONF_SERIAL_NUMBER_READ_ENABLED 1
#define CONF_VRMS_MAX_READ_ENABLED 1
#define CONF_VRMS_MIN_READ_ENABLED 1
#define CONF_VRMS_MEAN_READ_ENABLED 1
#define CONF_RESET_DATES_READ_ENABLED 1
#define CONF_THRESHOLD_OBIS_ENABLED 1
#define CONF_BIAS_SAMPLING_ENABLED 0

#if HARDWARE_VERSION >= 3
#undef CONF_THRESHOLD_PIN_ENABLED
#define CONF_THRESHOLD_PIN_ENABLED 0
#endif

// LED PIN Error Codes
#define LED_ERROR_CODE_UART_NOT_READABLE 1
#define LED_ERROR_CODE_MESSAGE_TIMEOUT 2
#define LED_ERROR_CODE_INVALID_REQUEST_MODE 3
#define LED_ERROR_CODE_INVALID_SERIAL_NUMBER 4
#define LED_ERROR_CODE_FLASH_MUTEX_NOT_TAKEN 5
#define LED_ERROR_CODE_FIFO_MUTEX_NOT_TAKEN 6
#define LED_ERROR_CODE_VRMS_VALUES_MUTEX_NOT_TAKEN 7
#define LED_ERROR_CODE_VRMS_THRESHOLD_MUTEX_NOT_TAKEN 8
#define LED_ERROR_CODE_THRESHOLD_SET_MUTEX_NOT_TAKEN 9
#define LED_ERROR_CODE_RX_BUFFER_OVERFLOW_ISR 10
#define LED_ERROR_CODE_STACK_OVERFLOW 11
// Flash'taki sektor numarasi / esik degeri gecerli araligin disinda cikti ve
// guvenli degere cekildi. Kayitlarin bir kismi kaybolmus olabilir.
#define LED_ERROR_CODE_FLASH_METADATA_CORRUPT 12

// indexed obis configuration
#define THRESHOLD_RECORD_OBIS_COUNT 10
#define RESET_DATES_OBIS_COUNT 12

// Watchdog Bits
#define WDT_FLAG_ADC_SAMPLE    (1 << 0)
#define WDT_FLAG_ADC_READ      (1 << 1)
#define WDT_FLAG_UART          (1 << 2)

#define WDT_ALL_TASKS_OK       (WDT_FLAG_ADC_SAMPLE | WDT_FLAG_ADC_READ | WDT_FLAG_UART)
extern volatile uint32_t task_health_flags;

// DEBUG MACRO
#if DEBUG
#define PRINTF(x, ...) printf(x, ##__VA_ARGS__)
#else
#define PRINTF(x, ...)
#endif

// Seri numarasinin tek kaynagi. Flash'a yazilmaz, flash'tan okunmaz.
// SERIAL_NUMBER_SIZE (9) + sonlandirici sigacak kadar buyuk olmali.
static const char s_number[16] = "612400080";

#endif