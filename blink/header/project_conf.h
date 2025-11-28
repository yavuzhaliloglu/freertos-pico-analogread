#ifndef PROJECT_CONF_H
#define PROJECT_CONF_H

#include <stdio.h>

// Device Password (will be written to flash)
#define DEVICE_PASSWORD "12345678"
// Device software version number
#define SOFTWARE_VERSION "V1.2.0"
// production date of device (yy-mm-dd)
#define PRODUCTION_DATE "25-08-20"
// Debugs
#define DEBUG 1
// bootloader select
#define WITHOUT_BOOTLOADER 1
// vrms multiplier value
#define VRMS_MULTIPLICATION_VALUE 150
// watchdog timeout ms to reset device
#define WATCHDOG_TIMEOUT_MS 5000
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
#define CONF_THRESHOLD_SET_ENABLED 1
#define CONF_THRESHOLD_GET_ENABLED 1
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

// indexed obis configuration
#define THRESHOLD_RECORD_OBIS_COUNT 10
#define RESET_DATES_OBIS_COUNT 12

// DEBUG MACRO
#if DEBUG
#define PRINTF(x, ...) printf(x, ##__VA_ARGS__)
#else
#define PRINTF(x, ...)
#endif

#if WITHOUT_BOOTLOADER
static const char s_number[256] = "612400073";
#endif

#endif