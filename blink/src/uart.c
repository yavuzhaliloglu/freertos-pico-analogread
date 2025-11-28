#include "header/uart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hardware/flash.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/rtc.h"
#include "hardware/uart.h"
#include "hardware/watchdog.h"
#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "header/bcc.h"
#include "header/mutex.h"
#include "header/print.h"
#include "header/project_globals.h"
#include "header/rtc.h"
#include "header/spiflash.h"

typedef enum {
    CMD_TYPE_ANY,
    CMD_TYPE_READ,
    CMD_TYPE_WRITE
} CommandType;

// struct type for lookup table
typedef struct
{
    const char *obis_code;
    enum ListeningStates state;
    CommandType type;
} ObisCommand;

// Lookup table for listening state
static const ObisCommand command_table[] = {
    {"P.01", Reading, CMD_TYPE_ANY},
    {"0.9.1", TimeSet, CMD_TYPE_WRITE},
    {"0.9.2", DateSet, CMD_TYPE_WRITE},
    {"0.9.1", ReadTime, CMD_TYPE_READ},
    {"0.9.2", ReadDate, CMD_TYPE_READ},
    {"0.0.0", ReadSerialNumber, CMD_TYPE_READ},
    {"96.1.3", ProductionInfo, CMD_TYPE_ANY},
    {"96.3.12", SetThreshold, CMD_TYPE_WRITE},
    {"96.3.12", GetThresholdObis, CMD_TYPE_READ},
    {"96.77.4*", GetThreshold, CMD_TYPE_ANY},
    {"96.3.10", ThresholdPin, CMD_TYPE_ANY},
    {"9.9.0", GetSuddenAmplitudeChange, CMD_TYPE_ANY},
    {"32.7.0", ReadLastVRMSMax, CMD_TYPE_ANY},
    {"52.7.0", ReadLastVRMSMin, CMD_TYPE_ANY},
    {"72.7.0", ReadLastVRMSMean, CMD_TYPE_ANY},
    {"0.1.2*", ReadResetDates, CMD_TYPE_ANY}};

// Ana komutlar hala aynı
static const uint8_t password_cmd[4] = {0x01, 0x50, 0x31, 0x02};
static const uint8_t reading_control_cmd[4] = {0x01, 0x52, 0x32, 0x02};
static const uint8_t reading_control_alt_cmd[4] = {0x01, 0x52, 0x35, 0x02};
static const uint8_t writing_control_cmd[4] = {0x01, 0x57, 0x32, 0x02};
static const uint8_t break_command[5] = {0x01, 0x42, 0x30, 0x03, 0x71};

void sendResetDates() {
}

uint8_t is_message_break_command(uint8_t *buf) {
    return (memcmp(buf, break_command, sizeof(break_command)) == 0);
}

void sendDeviceInfo() {
    uint8_t *flash_records = (uint8_t *)(XIP_BASE + FLASH_LOAD_PROFILE_RECORD_ADDR);
    uint32_t offset = 0;
    uint16_t record_count = 0;

    char debug_uart_buffer[45] = {0};
    sprintf(debug_uart_buffer, "sector count is: %d\r\n", sector_data);
    uart_puts(UART0_ID, debug_uart_buffer);

    uart_puts(UART0_ID, "System Time is: ");
    uart_puts(UART0_ID, datetime_str);
    uart_puts(UART0_ID, "\r\n");

    sprintf(debug_uart_buffer, "serial number of this device is: %s\r\n", serial_number);
    uart_puts(UART0_ID, debug_uart_buffer);

    if (xSemaphoreTake(xFlashMutex, pdMS_TO_TICKS(250)) == pdTRUE) {
        PRINTF("SEND DEVICE INFO: set data mutex received\n");
        for (offset = 0; offset < FLASH_LOAD_PROFILE_RECORD_AREA_SIZE; offset += FLASH_RECORD_SIZE) {
            if (flash_records[offset] == 0xFF || flash_records[offset] == 0x00) {
                continue;
            }

            char year[3] = {flash_records[offset], flash_records[offset + 1], 0x00};
            char month[3] = {flash_records[offset + 2], flash_records[offset + 3], 0x00};
            char day[3] = {flash_records[offset + 4], flash_records[offset + 5], 0x00};
            char hour[3] = {flash_records[offset + 6], flash_records[offset + 7], 0x00};
            char minute[3] = {flash_records[offset + 8], flash_records[offset + 9], 0x00};
            uint8_t max = flash_records[offset + 10];
            uint8_t max_dec = flash_records[offset + 11];
            uint8_t min = flash_records[offset + 12];
            uint8_t min_dec = flash_records[offset + 13];
            uint8_t mean = flash_records[offset + 14];
            uint8_t mean_dec = flash_records[offset + 15];

            sprintf(debug_uart_buffer, "%s-%s-%s;%s:%s;%d.%d,%d.%d,%d.%d\r\n", year, month, day, hour, minute, max, max_dec, min, min_dec, mean, mean_dec);
            uart_puts(UART0_ID, debug_uart_buffer);
            record_count++;
        }

        xSemaphoreGive(xFlashMutex);
    }

    sprintf(debug_uart_buffer, "usage of flash is: %d/%d bytes\n", record_count * 16, FLASH_LOAD_PROFILE_RECORD_AREA_SIZE);
    uart_puts(UART0_ID, debug_uart_buffer);

    sendResetDates();
}

// This function check the data which comes when State is Listening, and compares the message to defined strings, and returns a ListeningState value to process the request
enum ListeningStates checkListeningData(uint8_t *data_buffer, uint8_t size) {
    if (!bccControl(data_buffer, size)) {
        return BCCError;
    }

    if (memcmp(data_buffer, password_cmd, sizeof(password_cmd)) == 0) {
        return Password;
    }

    if (memcmp(data_buffer, break_command, sizeof(break_command)) == 0) {
        PRINTF("CHECKLISTENINGDATA: Break command received.\n");
        return BreakMessage;
    }

    const bool is_any_reading_msg = (memcmp(data_buffer, reading_control_cmd, sizeof(reading_control_cmd)) == 0) ||
                                    (memcmp(data_buffer, reading_control_alt_cmd, sizeof(reading_control_alt_cmd)) == 0);
    const bool is_writing_msg = (memcmp(data_buffer, writing_control_cmd, sizeof(writing_control_cmd)) == 0);

    if (is_any_reading_msg || is_writing_msg) {
        const char *buffer_as_char = (const char *)data_buffer;
        for (size_t i = 0; i < (sizeof(command_table) / sizeof(command_table[0])); ++i) {
            if (strstr(buffer_as_char, command_table[i].obis_code) != NULL) {
                bool type_match = false;
                if (command_table[i].type == CMD_TYPE_ANY) {
                    type_match = true;
                } else if (command_table[i].type == CMD_TYPE_READ && is_any_reading_msg) {
                    type_match = true;
                } else if (command_table[i].type == CMD_TYPE_WRITE && is_writing_msg) {
                    type_match = true;
                }

                if (type_match) {
                    PRINTF("CHECKLISTENINGDATA: State %d is accepted.\n", command_table[i].state);
                    return command_table[i].state;
                }
            }
        }
    }

    return DataError;
}
// This function deletes a character from a given string
void deleteChar(uint8_t *str, uint8_t len, char chr) {
    uint8_t i, j;
    for (i = j = 0; i < len; i++) {
        if (str[i] != chr) {
            str[j++] = str[i];
        }
    }
    str[j] = '\0';
}

// This function gets the load profile data and finds the date characters and add them to time arrays
void parseLoadProfileDates(uint8_t *buffer, uint8_t len, uint8_t *reading_state_start_time, uint8_t *reading_state_end_time) {
    uint8_t date_start[14] = {0};
    uint8_t date_end[14] = {0};

    for (uint8_t i = 0; i < len; i++) {
        if (buffer[i] == 0x28) {
            uint8_t k;

            for (k = i + 1; buffer[k] != 0x3B; k++) {
                if (k == len) {
                    break;
                }

                date_start[k - (i + 1)] = buffer[k];
            }

            // Delete the characters from start date array
            if (len == 41) {
                deleteChar(date_start, strlen((char *)date_start), '-');
                deleteChar(date_start, strlen((char *)date_start), ',');
                deleteChar(date_start, strlen((char *)date_start), ':');
            }

            // add end time to array
            for (uint8_t l = k + 1; buffer[l] != 0x29; l++) {
                if (l == len) {
                    break;
                }

                date_end[l - (k + 1)] = buffer[l];
            }

            if (len == 41) {
                // Delete the characters from end date array
                deleteChar(date_end, strlen((char *)date_end), '-');
                deleteChar(date_end, strlen((char *)date_end), ',');
                deleteChar(date_end, strlen((char *)date_end), ':');
            }

            break;
        }
    }

    memcpy(reading_state_start_time, date_start, 10);
    memcpy(reading_state_end_time, date_end, 10);

    PRINTF("DATE START:\n");
    printBufferHex(reading_state_start_time, 10);
    PRINTF("\n");
    PRINTF("DATE END:\n");
    printBufferHex(reading_state_end_time, 10);
    PRINTF("\n");
}

void parseThresholdRequestDates(uint8_t *buffer, uint8_t *start_date_inc, uint8_t *end_date_inc) {
    char *date_start_ptr = strchr((char *)buffer, '(');
    char *date_end_ptr = strchr((char *)buffer, ')');
    char *date_division_ptr = strchr((char *)buffer, ';');

    uint8_t sd_temp[20] = {0};
    uint8_t ed_temp[20] = {0};

    memcpy(sd_temp, date_start_ptr + 1, date_division_ptr - date_start_ptr - 1);
    memcpy(ed_temp, date_division_ptr + 1, date_end_ptr - date_division_ptr - 1);

    PRINTF("SD_TEMP:\n");
    printBufferHex(sd_temp, 20);
    PRINTF("\n");
    PRINTF("ED_TEMP:\n");
    printBufferHex(ed_temp, 20);
    PRINTF("\n");

    deleteChar(sd_temp, strlen((char *)sd_temp), '-');
    deleteChar(sd_temp, strlen((char *)sd_temp), ',');
    deleteChar(sd_temp, strlen((char *)sd_temp), ':');

    deleteChar(ed_temp, strlen((char *)ed_temp), '-');
    deleteChar(ed_temp, strlen((char *)ed_temp), ',');
    deleteChar(ed_temp, strlen((char *)ed_temp), ':');

    memcpy(start_date_inc, sd_temp, 12);
    memcpy(end_date_inc, ed_temp, 12);

    PRINTF("START DATE :\n");
    printBufferHex(start_date_inc, 12);
    PRINTF("\n");

    PRINTF("END DATE :\n");
    printBufferHex(end_date_inc, 12);
    PRINTF("\n");
}

void parseACRequestDate(uint8_t *buffer, uint8_t *start_date, uint8_t *end_date) {
    char *date_start_ptr = strchr((char *)buffer, '(');
    char *date_end_ptr = strchr((char *)buffer, ')');
    char *date_division_ptr = strchr((char *)buffer, ';');

    uint8_t sd_temp[20] = {0};
    uint8_t ed_temp[20] = {0};

    memcpy(sd_temp, date_start_ptr + 1, date_division_ptr - date_start_ptr - 1);
    memcpy(ed_temp, date_division_ptr + 1, date_end_ptr - date_division_ptr - 1);

    PRINTF("SD_TEMP:\n");
    printBufferHex(sd_temp, 20);
    PRINTF("\n");

    PRINTF("ED_TEMP:\n");
    printBufferHex(ed_temp, 20);
    PRINTF("\n");

    deleteChar(sd_temp, strlen((char *)sd_temp), '-');
    deleteChar(sd_temp, strlen((char *)sd_temp), ',');
    deleteChar(sd_temp, strlen((char *)sd_temp), ':');

    deleteChar(ed_temp, strlen((char *)ed_temp), '-');
    deleteChar(ed_temp, strlen((char *)ed_temp), ',');
    deleteChar(ed_temp, strlen((char *)ed_temp), ':');

    memcpy(start_date, sd_temp, 12);
    memcpy(end_date, ed_temp, 12);

    PRINTF("START DATE :\n");
    printBufferHex(start_date, 12);
    PRINTF("\n");

    PRINTF("END DATE :\n");
    printBufferHex(end_date, 12);
    PRINTF("\n");
}

uint8_t is_message_reset_factory_settings_message(uint8_t *msg_buf, uint8_t msg_len) {
    uint8_t rst_msg[] = "/?RSTFS?\r\n";
    return msg_len == 10 && strncmp((char *)msg_buf, (char *)rst_msg, 10) == 0;
}

uint8_t is_message_reboot_device_message(uint8_t *msg_buf, uint8_t msg_len) {
    uint8_t rbt_msg[] = "/?RBTDVC?\r\n";
    return msg_len == 11 && strncmp((char *)msg_buf, (char *)rbt_msg, 11) == 0;
}

void reboot_device() {
    PRINTF("Rebooting Device...\n");
    watchdog_reboot(0, 0, 0);
}

void reset_to_factory_settings() {
    if (xSemaphoreTake(xFlashMutex, pdMS_TO_TICKS(250)) == pdTRUE) {
        uint32_t ints = save_and_disable_interrupts();
        flash_range_erase(FLASH_LOAD_PROFILE_LAST_SECTOR_DATA_ADDR, FLASH_LOAD_PROFILE_LAST_SECTOR_DATA_SIZE);
        flash_range_erase(FLASH_LOAD_PROFILE_RECORD_ADDR, FLASH_LOAD_PROFILE_RECORD_AREA_SIZE);
        flash_range_erase(FLASH_THRESHOLD_PARAMETERS_ADDR, FLASH_THRESHOLD_PARAMETERS_SIZE);
        flash_range_erase(FLASH_THRESHOLD_RECORDS_ADDR, FLASH_THRESHOLD_RECORDS_SIZE);
        flash_range_erase(FLASH_RESET_DATES_ADDR, FLASH_RESET_DATES_AREA_SIZE);
        flash_range_erase(FLASH_AMPLITUDE_CHANGE_OFFSET, (FLASH_AMPLITUDE_RECORDS_TOTAL_SECTOR * FLASH_SECTOR_SIZE));
        restore_interrupts(ints);
        xSemaphoreGive(xFlashMutex);
    }

    sleep_ms(1000);
    watchdog_reboot(0, 0, 0);
}

uint8_t is_end_connection_message(uint8_t *msg_buf) {
    uint8_t end_connection_str[5] = {0x01, 0x42, 0x30, 0x03, 0x71}; // [SOH]B0[ETX]q

    return (strncmp((char *)msg_buf, (char *)end_connection_str, 5) == 0) ? 1 : 0;
}

// This function gets the baud rate number like 1-6
uint8_t getProgramBaudRate(uint16_t b_rate) {
    uint8_t baudrate = 0;

    switch (b_rate) {
        case 300:
            baudrate = 0;
            break;
        case 600:
            baudrate = 1;
            break;
        case 1200:
            baudrate = 2;
            break;
        case 2400:
            baudrate = 3;
            break;
        case 4800:
            baudrate = 4;
            break;
        case 9600:
            baudrate = 5;
            break;
        case 19200:
            baudrate = 6;
            break;
    }

    return baudrate;
}

// This function sets the device's baud rate according to given number like 0,1,2,3,4,5,6
void set_device_baud_rate(uint8_t b_rate_hex) {
    uint set_baud_rate = 0;

    switch (b_rate_hex) {
        case 0x30:
            set_baud_rate = 300;
            break;
        case 0x31:
            set_baud_rate = 600;
            break;
        case 0x32:
            set_baud_rate = 1200;
            break;
        case 0x33:
            set_baud_rate = 2400;
            break;
        case 0x34:
            set_baud_rate = 4800;
            break;
        case 0x35:
            set_baud_rate = 9600;
            break;
        case 0x36:
            set_baud_rate = 19200;
            break;
        default:
            set_baud_rate = 300;
            break;
    }
    uart_set_baudrate(UART0_ID, set_baud_rate);
}

// This function sets state to Greeting and resets rx_buffer and it's len. Also it sets the baud rate to 300, which is initial baud rate.
void reset_uart() {
    PRINTF("Reset State Timer Trigger!\n");
    set_init_baud_rate();
}

uint8_t *get_serial_number_ptr() {
    return (uint8_t *)serial_number;
}

void set_init_baud_rate() {
    uart_tx_wait_blocking(UART0_ID);
    uart_set_baudrate(UART0_ID, 300);
    // Clear the RX FIFO to remove any garbage characters received during baud rate switch
    while (uart_is_readable(UART0_ID)) {
        uart_getc(UART0_ID);
    }
}

bool control_serial_number(uint8_t *identification_req_buf, size_t req_size) {
    uint8_t i;
    uint8_t *serial_num_ptr = get_serial_number_ptr();

    if (serial_num_ptr == NULL || identification_req_buf == NULL) {
        PRINTF("Serial number pointer is NULL!\n");
        return false;
    }

    for (i = 0; *identification_req_buf != '/'; identification_req_buf++, i++) {
        if (i == req_size) {
            PRINTF("Identification is wrong!\n");
            return false;
        }
    }

    if (strncmp((char *)identification_req_buf, (char *)"/?!\r\n", req_size) == 0) {
        return true;
    } else if (strncmp((char *)identification_req_buf, (char *)serial_num_ptr, SERIAL_NUMBER_SIZE) != 0) {
        return false;
    }

    return true;
}

size_t create_identify_response_message(char *response_buf, size_t buf_size) {
    memset(response_buf, 0, buf_size);
    return snprintf(response_buf, buf_size, "/%s%d<%d>---(MA-V3)\r\n", METER_FLAG_CODE, METER_MAX_SUPPORTED_BAUDRATE, METER_VERSION);
}

uint8_t exract_baud_rate_and_mode_from_message(uint8_t *msg_buf, size_t msg_len, int8_t *requested_mode) {

    if (msg_buf == NULL || msg_len < 3) {
        PRINTF("Message buffer is NULL or message length is invalid!\n");
        return 0;
    }

    uint8_t baud_rate = msg_buf[2];
    *requested_mode = (int8_t)msg_buf[3];

    return baud_rate;
}

void send_threshold_records(uint8_t *xor_result) {
    uint8_t threshold_records_raw[FLASH_RECORD_SIZE * THRESHOLD_RECORD_OBIS_COUNT];
    // record area offset pointer
    uint8_t *record_ptr = (uint8_t *)(XIP_BASE + FLASH_THRESHOLD_RECORDS_ADDR);
    // buffer to format
    uint8_t buffer[48] = {0};
    // copy the flash content in struct
    char year[3] = {0};
    char month[3] = {0};
    char day[3] = {0};
    char hour[3] = {0};
    char min[3] = {0};
    char sec[3] = {0};
    uint16_t vrms = 0;
    uint16_t variance = 0;
    int result;

    memset(buffer, 0, sizeof(buffer));
    memset(threshold_records_raw, 0, sizeof(threshold_records_raw));

    if (xSemaphoreTake(xFlashMutex, pdMS_TO_TICKS(250)) == pdTRUE) {
        memcpy(threshold_records_raw, record_ptr, sizeof(threshold_records_raw));
        xSemaphoreGive(xFlashMutex);
    } else {
        PRINTF("SEND THRESHOLD RECORDS: Could not take flash mutex!\n");
        sendErrorMessage((char *)"FLASHMUTEXERR");
        return;
    }

    for (size_t i = 0, idx = THRESHOLD_RECORD_OBIS_COUNT; i < THRESHOLD_RECORD_OBIS_COUNT; i++, idx--) {
        size_t offset = i * FLASH_RECORD_SIZE;

        if (threshold_records_raw[offset] == 0xFF || threshold_records_raw[offset] == 0x00) {
            result = snprintf((char *)buffer, sizeof(buffer), "96.77.4*%d(00-00-00,00:00:00)(000,00000)\r\n", idx);
        } else {
            snprintf(year, sizeof(year), "%c%c", threshold_records_raw[offset], threshold_records_raw[offset + 1]);
            snprintf(month, sizeof(month), "%c%c", threshold_records_raw[offset + 2], threshold_records_raw[offset + 3]);
            snprintf(day, sizeof(day), "%c%c", threshold_records_raw[offset + 4], threshold_records_raw[offset + 5]);
            snprintf(hour, sizeof(hour), "%c%c", threshold_records_raw[offset + 6], threshold_records_raw[offset + 7]);
            snprintf(min, sizeof(min), "%c%c", threshold_records_raw[offset + 8], threshold_records_raw[offset + 9]);
            snprintf(sec, sizeof(sec), "%c%c", threshold_records_raw[offset + 10], threshold_records_raw[offset + 11]);
            vrms = threshold_records_raw[offset + 13];
            vrms = (vrms << 8);
            vrms += threshold_records_raw[offset + 12];
            variance = threshold_records_raw[offset + 15];
            variance = (variance << 8);
            variance += threshold_records_raw[offset + 14];

            result = snprintf((char *)buffer, sizeof(buffer), "96.77.4*%d(%s-%s-%s,%s:%s:%s)(%03d,%05d)\r\n", idx, year, month, day, hour, min, sec, vrms, variance);
        }

        // xor all bytes of formatted array
        bccGenerate(buffer, result, xor_result);

        PRINTF("threshold record to send is: \n");
        printBufferHex(buffer, result);
        PRINTF("\n");

        if (result >= (int)sizeof(buffer)) {
            PRINTF("GETTHRESHOLDRECORD: Buffer Overflow! Sending NACK.\n");
            sendErrorMessage((char *)"THBUFFEROVERFLOW");
        } else {
            // Send the readout data
            uart_puts(UART0_ID, (char *)buffer);
        }

        vTaskDelay(pdMS_TO_TICKS(15));
    }

    uart_tx_wait_blocking(UART0_ID);
}

void send_reset_dates(uint8_t *xor_result) {
    uint8_t *reset_dates_flash = (uint8_t *)(XIP_BASE + FLASH_RESET_DATES_ADDR);
    uint8_t reset_dates_raw[RESET_DATES_OBIS_COUNT * FLASH_RECORD_SIZE];
    char date_buffer[32];
    int result;

    memset(reset_dates_raw, 0, sizeof(reset_dates_raw));
    memset(date_buffer, 0, sizeof(date_buffer));

    if (xSemaphoreTake(xFlashMutex, pdMS_TO_TICKS(250)) == pdTRUE) {
        memcpy(reset_dates_raw, reset_dates_flash, sizeof(reset_dates_raw));
        xSemaphoreGive(xFlashMutex);
    } else {
        PRINTF("SEND RESET DATES: Could not take flash mutex!\n");
        sendErrorMessage((char *)"FLASHMUTEXERR");
        return;
    }

    for (uint8_t i = 0,idx = RESET_DATES_OBIS_COUNT; i < RESET_DATES_OBIS_COUNT; i++,idx--) {
        size_t offset = i * FLASH_RECORD_SIZE;
        if (reset_dates_raw[offset] == 0xFF || reset_dates_raw[offset] == 0x00) {
            result = snprintf(date_buffer, sizeof(date_buffer), "0.1.2*%d(00-00-00,00:00:00)\r\n",idx);
        } else {
            char year[3] = {reset_dates_raw[offset], reset_dates_raw[offset + 1], 0x00};
            char month[3] = {reset_dates_raw[offset + 2], reset_dates_raw[offset + 3], 0x00};
            char day[3] = {reset_dates_raw[offset + 4], reset_dates_raw[offset + 5], 0x00};
            char hour[3] = {reset_dates_raw[offset + 6], reset_dates_raw[offset + 7], 0x00};
            char min[3] = {reset_dates_raw[offset + 8], reset_dates_raw[offset + 9], 0x00};
            char sec[3] = {reset_dates_raw[offset + 10], reset_dates_raw[offset + 11], 0x00};
            xSemaphoreGive(xFlashMutex);

            result = snprintf(date_buffer, sizeof(date_buffer), "0.1.2*%d(%s-%s-%s,%s:%s:%s)\r\n",idx, year, month, day, hour, min, sec);

            if (result >= (int)sizeof(date_buffer)) {
                sendErrorMessage((char *)"DATEBUFFERSMALL");
                continue;
            }
        }

        bccGenerate((uint8_t *)date_buffer, result, xor_result);
        uart_puts(UART0_ID, date_buffer);
    }
}

void send_readout_message(uint8_t request_mode) {
    char readout_line_buffer[32];
    int result = 0;
    uint8_t readout_xor = 0x00;

    memset(readout_line_buffer, 0, sizeof(readout_line_buffer));

    uart_putc(UART0_ID, STX);

    result = snprintf(readout_line_buffer, sizeof(readout_line_buffer), "0.0.0(%s)\r\n", serial_number);
    bccGenerate((uint8_t *)readout_line_buffer, result, &readout_xor);
    uart_puts(UART0_ID, readout_line_buffer);

    result = snprintf(readout_line_buffer, sizeof(readout_line_buffer), "0.2.0(%s)\r\n", SOFTWARE_VERSION);
    bccGenerate((uint8_t *)readout_line_buffer, result, &readout_xor);
    uart_puts(UART0_ID, readout_line_buffer);

    result = snprintf(readout_line_buffer, sizeof(readout_line_buffer), "0.8.4(%d*min)\r\n", load_profile_record_period);
    bccGenerate((uint8_t *)readout_line_buffer, result, &readout_xor);
    uart_puts(UART0_ID, readout_line_buffer);

    result = snprintf(readout_line_buffer, sizeof(readout_line_buffer), "0.9.1(%02d:%02d:%02d)\r\n", current_time.hour, current_time.min, current_time.sec);
    bccGenerate((uint8_t *)readout_line_buffer, result, &readout_xor);
    uart_puts(UART0_ID, readout_line_buffer);

    result = snprintf(readout_line_buffer, sizeof(readout_line_buffer), "0.9.2(%02d-%02d-%02d)\r\n", current_time.year, current_time.month, current_time.day);
    bccGenerate((uint8_t *)readout_line_buffer, result, &readout_xor);
    uart_puts(UART0_ID, readout_line_buffer);

    result = snprintf(readout_line_buffer, sizeof(readout_line_buffer), "96.1.3(%s)\r\n", PRODUCTION_DATE);
    bccGenerate((uint8_t *)readout_line_buffer, result, &readout_xor);
    uart_puts(UART0_ID, readout_line_buffer);

    result = snprintf(readout_line_buffer, sizeof(readout_line_buffer), "96.3.12(%03d)\r\n", getVRMSThresholdValue());
    bccGenerate((uint8_t *)readout_line_buffer, result, &readout_xor);
    uart_puts(UART0_ID, readout_line_buffer);

    if (request_mode == REQUEST_MODE_LONG_READ) {
        send_threshold_records(&readout_xor);
        send_reset_dates(&readout_xor);
    }

    if (xSemaphoreTake(xVRMSLastValuesMutex, pdMS_TO_TICKS(250)) == pdTRUE) {
        result = snprintf(readout_line_buffer, sizeof(readout_line_buffer), "32.7.0(%.2f)\r\n", vrms_max_last);
        bccGenerate((uint8_t *)readout_line_buffer, result, &readout_xor);
        uart_puts(UART0_ID, readout_line_buffer);

        result = snprintf(readout_line_buffer, sizeof(readout_line_buffer), "52.7.0(%.2f)\r\n", vrms_min_last);
        bccGenerate((uint8_t *)readout_line_buffer, result, &readout_xor);
        uart_puts(UART0_ID, readout_line_buffer);

        result = snprintf(readout_line_buffer, sizeof(readout_line_buffer), "72.7.0(%.2f)\r\n", vrms_mean_last);
        bccGenerate((uint8_t *)readout_line_buffer, result, &readout_xor);
        uart_puts(UART0_ID, readout_line_buffer);

        xSemaphoreGive(xVRMSLastValuesMutex);
    }

    result = snprintf(readout_line_buffer, sizeof(readout_line_buffer), "!\r\n%c", ETX);
    bccGenerate((uint8_t *)readout_line_buffer, result, &readout_xor);
    uart_puts(UART0_ID, readout_line_buffer);

    PRINTF("SETTINGSTATEHANDLER: readout XOR is: %02X.\n", readout_xor);
    uart_putc(UART0_ID, readout_xor);
    uart_tx_wait_blocking(UART0_ID);
}

void send_programming_acknowledgement() {
    // Generate the message to send UART
    uint8_t ack_buff[32];
    uint8_t ack_bcc = SOH;

    memset(ack_buff, 0, sizeof(ack_buff));

    int result = snprintf((char *)ack_buff, sizeof(ack_buff), "%cP0%c(%s)%c", SOH, STX, serial_number, ETX);
    if (result >= (int)sizeof(ack_buff)) {
        sendErrorMessage((char *)"ACKBUFOVERFLOW");
        return;
    }

    // Generate BCC for acknowledgement message
    bccGenerate(ack_buff, result, &ack_bcc);

    PRINTF("<--- ");
    printBufferHex(ack_buff, 18);
    PRINTF("\n");

    // SEND Message
    uart_puts(UART0_ID, (char *)ack_buff);
    uart_putc(UART0_ID, ack_bcc);

    PRINTF("Waiting to block\n");
    uart_tx_wait_blocking(UART0_ID);
    PRINTF("block finish!\n");
}

uint8_t verifyHourMinSec(uint8_t hour, uint8_t min, uint8_t sec) {
    if (hour > 23 || min > 59 || sec > 59) {
        return 0;
    }

    return 1;
}

uint8_t verifyYearMonthDay(uint8_t year, uint8_t month, uint8_t day) {
    if (year > 99 || month > 12 || day > 31) {
        return 0;
    }

    return 1;
}

// This function sets time via UART
void setTimeFromUART(uint8_t *buffer) {
    if (!password_correct_flag) {
        sendErrorMessage((char *)"NOPWENTERED");
        return;
    }

    // initialize variables
    uint8_t time_buffer[9] = {0};
    uint8_t hour;
    uint8_t min;
    uint8_t sec;

    // get pointers to get time data between () characters
    char *start_ptr = strchr((char *)buffer, '(');
    start_ptr++;
    char *end_ptr = strchr((char *)buffer, ')');

    if (start_ptr == NULL && end_ptr == NULL) {
        return;
    }

    // copy time data to buffer and delete the ":" character. First two characters of the array is hour info, next 2 character is minute info, last 2 character is the second info. Also there are two ":" characters to seperate informations.
    strncpy((char *)time_buffer, start_ptr, end_ptr - start_ptr);
    deleteChar(time_buffer, strlen((char *)time_buffer), ':');

    // set hour,min and sec variables to change time.
    hour = (time_buffer[0] - '0') * 10 + (time_buffer[1] - '0');
    min = (time_buffer[2] - '0') * 10 + (time_buffer[3] - '0');
    sec = (time_buffer[4] - '0') * 10 + (time_buffer[5] - '0');

    PRINTF("SETTIMEFROMUART: hour: %d, min: %d, sec: %d\n", hour, min, sec);

    if (verifyHourMinSec(hour, min, sec)) {
        // Get the current time and set chip's Time value according to variables and current date values
        if (!setTimePt7c4338(I2C_PORT, I2C_ADDRESS, sec, min, hour, (uint8_t)current_time.dotw, (uint8_t)current_time.day, (uint8_t)current_time.month, (uint8_t)current_time.year)) {
            sendErrorMessage((char *)"PT7CTIMENOTSET");
            return;
        }

        if (!getTimePt7c4338(&current_time)) {
            sendErrorMessage((char *)"PT7CTIMENOTGET");
            return;
        }

        // Get new current time from RTC Chip and set to RP2040's RTC module

        // ??
        if (current_time.dotw < 0 || current_time.dotw > 6) {
            PRINTF("SETTIMEFROMUART: invalid day of the week: %d!\n", current_time.dotw);
            current_time.dotw = 2;
        }
        //

        bool is_rtc_set = rtc_set_datetime(&current_time);

        if (is_rtc_set) {
            PRINTF("SETTIMEFROMUART: time was set to: %d:%d:%d\n", current_time.hour, current_time.min, current_time.sec);
            uart_putc(UART0_ID, ACK);
        } else {
            PRINTF("SETTTIMEFROMUART: time was not set!\n");
            sendErrorMessage((char *)"TIMENOTSET");
        }
    } else {
        PRINTF("SETTIMEFROMUART: invalid tim values!\n");
        sendErrorMessage((char *)"INVALIDTIMEVAL");
    }

    password_correct_flag = false;
}

// This function sets date via UART
void setDateFromUART(uint8_t *buffer) {
    if (!password_correct_flag) {
        sendErrorMessage((char *)"NOPWENTERED");
        return;
    }

    // initialize variables
    uint8_t date_buffer[9] = {0};
    uint8_t year;
    uint8_t month;
    uint8_t day;

    // get pointers to get date data between () characters
    char *start_ptr = strchr((char *)buffer, '(');
    start_ptr++;
    char *end_ptr = strchr((char *)buffer, ')');

    if (start_ptr == NULL && end_ptr == NULL) {
        return;
    }

    // copy date data to buffer and delete the "-" character. First two characters of the array is year info, next 2 character is month info, last 2 character is the day info. Also there are two "-" characters to seperate informations.
    strncpy((char *)date_buffer, start_ptr, end_ptr - start_ptr);
    deleteChar(date_buffer, strlen((char *)date_buffer), '-');

    // set year,month and day variables to change date.
    year = (date_buffer[0] - '0') * 10 + (date_buffer[1] - '0');
    month = (date_buffer[2] - '0') * 10 + (date_buffer[3] - '0');
    day = (date_buffer[4] - '0') * 10 + (date_buffer[5] - '0');

    PRINTF("SETDATEFROMUART: year: %d, month: %d, day: %d\n", year, month, day);

    if (verifyYearMonthDay(year, month, day)) {
        // Get the current time and set chip's date value according to variables and current time values
        if (!setTimePt7c4338(I2C_PORT, I2C_ADDRESS, (uint8_t)current_time.sec, (uint8_t)current_time.min, (uint8_t)current_time.hour, (uint8_t)current_time.dotw, day, month, year)) {
            sendErrorMessage((char *)"PT7CDATENOTSET");
            return;
        }
        // Get new current time from RTC Chip and set to RP2040's RTC module
        if (!getTimePt7c4338(&current_time)) {
            sendErrorMessage((char *)"PT7CDATENOTGET");
            return;
        }

        if (current_time.dotw < 0 || current_time.dotw > 6) {
            PRINTF("SETDATEFROMUART: invalid day of the week: %d!\n", current_time.dotw);
            current_time.dotw = 2;
        }

        bool is_rtc_set = rtc_set_datetime(&current_time);

        if (is_rtc_set) {
            PRINTF("SETDATEFROMUART: date was set to: %d-%d-%d\n", current_time.year, current_time.month, current_time.day);
            uart_putc(UART0_ID, ACK);
        } else {
            PRINTF("SETDATEFROMUART: date was not set!\n");
            sendErrorMessage((char *)"DATENOTSET");
        }
    } else {
        PRINTF("SETDATEFROMUART: invalid date values!\n");
        sendErrorMessage((char *)"INVALIDDATEVAL");
    }

    password_correct_flag = false;
}

// This function generates a product,on info message and sends it to UART
void sendProductionInfo() {
    // initialize the buffer and get the current time
    char production_obis_buffer[24] = {0};
    uint8_t production_bcc = STX;

    // generate the message and add BCC to the message
    int result = snprintf(production_obis_buffer, sizeof(production_obis_buffer), "%c96.1.3(%s)\r\n%c", STX, PRODUCTION_DATE, ETX);
    if (result >= (int)sizeof(production_obis_buffer)) {
        PRINTF("SENDPRODUCTIONINFO: production buffer is too small.\n");
        sendErrorMessage((char *)"SMALLBUFFERSIZE");
        return;
    }

    bccGenerate((uint8_t *)production_obis_buffer, result, &production_bcc);
    PRINTF("SENDPRODUCTIONINFO: production info message to send: %s\n", production_obis_buffer);

    // send the message to UART
    uart_puts(UART0_ID, production_obis_buffer);
    uart_putc(UART0_ID, production_bcc);
}

// This function gets a password and controls the password, if password is true, device sends an ACK message, if not, device sends NACK message
void passwordHandler(uint8_t *buffer) {
    char *ptr = strchr((char *)buffer, '(');
    ptr++;

    if (strncmp(ptr, DEVICE_PASSWORD, 8) == 0) {
        uart_putc(UART0_ID, ACK);
        password_correct_flag = true;
    } else {
        sendErrorMessage((char *)"PWNOTCORRECT");
    }
}

void __not_in_flash_func(setThresholdValue)(uint8_t *data) {
    if (!password_correct_flag) {
        sendErrorMessage((char *)"NOPWENTERED");
        return;
    }
    PRINTF("SETTHRESHOLDVALUE: threshold value before change is: %d\n", getVRMSThresholdValue());

    // get value start and end pointers
    char *start_ptr = strchr((char *)data, '(');
    char *end_ptr = strchr((char *)data, ')');

    // VALIDATION: Check if pointers are valid
    if (start_ptr == NULL || end_ptr == NULL || end_ptr <= start_ptr) {
        PRINTF("SETTHRESHOLDVALUE: Invalid data format\n");
        sendErrorMessage((char *)"DATAFORMATERROR");
        return;
    }

    // inc start pointer
    start_ptr++;

    // get string threshold value
    size_t len = end_ptr - start_ptr;

    // VALIDATION: Check length to prevent buffer overflow (buffer is size 4)
    if (len == 0 || len >= 4) {
        PRINTF("SETTHRESHOLDVALUE: Invalid value length\n");
        sendErrorMessage((char *)"VALUELENGTHERROR");
        return;
    }

    // to keep string threshold value
    char threshold_val_str[4];
    // converted threshold value from string
    uint16_t threshold_val;
    // array and pointer to write updated values to flash memory
    uint16_t th_arr[FLASH_PAGE_SIZE / sizeof(uint16_t)] = {0};
    uint16_t *th_ptr = (uint16_t *)(XIP_BASE + FLASH_THRESHOLD_PARAMETERS_ADDR);

    strncpy(threshold_val_str, start_ptr, len);
    threshold_val_str[len] = '\0';
    // convert threshold value to uint16_t type
    threshold_val = atoi(threshold_val_str);

    PRINTF("SETTHRESHOLDVALUE: threshold value as string is: %s\n", threshold_val_str);
    PRINTF("SETTHRESHOLDVALUE: threshold value as int is: %d\n", threshold_val);

    // if the current value is the same with requested value, do nothing
    if (getVRMSThresholdValue() == threshold_val) {
        uart_putc(UART0_ID, ACK);
        return;
    }

    // set the threshold value to use in program
    setVRMSThresholdValue(threshold_val);

    // set array variables to updated values (just threshold changed)
    th_arr[0] = getVRMSThresholdValue();
    if (xSemaphoreTake(xFlashMutex, pdMS_TO_TICKS(250)) == pdTRUE) {
        PRINTF("SETTHRESHOLDVALUE: write data mutex received\n");
        // Read existing value safely before erasing
        th_arr[1] = th_ptr[1];

        // CRITICAL SECTION: Disable interrupts on this core to prevent context switch during flash op
        taskENTER_CRITICAL();
        flash_range_erase(FLASH_THRESHOLD_PARAMETERS_ADDR, FLASH_THRESHOLD_PARAMETERS_SIZE);
        flash_range_program(FLASH_THRESHOLD_PARAMETERS_ADDR, (uint8_t *)th_arr, FLASH_PAGE_SIZE);
        taskEXIT_CRITICAL();

        xSemaphoreGive(xFlashMutex);
    } else {
        PRINTF("SETTHRESHOLDVALUE: MUTEX CANNOT RECEIVED!\n");
        sendErrorMessage((char *)"FLASHBUSY");
        return;
    }

    PRINTF("SETTHRESHOLDVALUE: threshold info content is:  \n");
    printBufferHex((uint8_t *)th_ptr, FLASH_PAGE_SIZE);
    PRINTF("\n");

    uart_putc(UART0_ID, ACK);
    PRINTF("SETTHRESHOLDVALUE: ACK send from set threshold value.\n");

    password_correct_flag = false;
}

#if CONF_THRESHOLD_PIN_ENABLED
void resetThresholdPIN() {
    if (!password_correct_flag) {
        sendErrorMessage((char *)"NOPWENTERED");
        return;
    }

    if (getThresholdSetBeforeFlag()) {
        PRINTF("RESETTHRESHOLDPIN: Threshold PIN set before, resetting pin...\n");

        gpio_put(THRESHOLD_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        setThresholdSetBeforeFlag(0);

        PRINTF("RESETTHRESHOLDPIN: ACK send from set threshold pin.\n");
        uart_putc(UART0_ID, ACK);
    } else {
        PRINTF("RESETTHRESHOLDPIN: Threshold PIN not set before, sending NACK.\n");
        sendErrorMessage((char *)"NOPINSET");
    }

    password_correct_flag = false;
}

void setThresholdPIN() {
    if (!getThresholdSetBeforeFlag()) {
        PRINTF("SETTHRESHOLDPIN: Threshold PIN set before, setting pin...\n");

        gpio_put(THRESHOLD_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
        setThresholdSetBeforeFlag(1);

        PRINTF("SETTHRESHOLDPIN: Threshold PIN set\n");
    }
}
#endif

void getSuddenAmplitudeChangeRecords(uint8_t *reading_state_start_time, uint8_t *reading_state_end_time, enum ListeningStates state) {
    datetime_t start = {0};
    datetime_t end = {0};
    int32_t start_index = -1;
    int32_t end_index = -1;
    uint8_t *flash_sudden_amp_content = (uint8_t *)(XIP_BASE + FLASH_AMPLITUDE_CHANGE_OFFSET);

    PRINTF("GETSUDDEAMPLITUDECHANGERECORDS: All Records are going to send.\n");
    getAllRecords(&start_index, &end_index, &start, &end, FLASH_AMPLITUDE_CHANGE_OFFSET, FLASH_AMPLITUDE_RECORDS_TOTAL_SECTOR * FLASH_SECTOR_SIZE, FLASH_SECTOR_SIZE, state);

    PRINTF("SEARCHDATAINFLASH: Start index is: %ld\n", start_index);
    PRINTF("SEARCHDATAINFLASH: End index is: %ld\n", end_index);

    if (start_index >= 0 && end_index >= 0) {
        // initialize the variables
        uint8_t xor_result = 0x00;
        uint32_t start_addr = start_index;
        uint32_t end_addr = start_index <= end_index ? end_index : (int32_t)((FLASH_AMPLITUDE_RECORDS_TOTAL_SECTOR * FLASH_SECTOR_SIZE) - FLASH_SECTOR_SIZE);
        uint8_t ac_record_buf[FLASH_SECTOR_SIZE];

        uart_putc(UART0_ID, STX);

        for (; start_addr <= end_addr;) {
            if (xSemaphoreTake(xFlashMutex, pdMS_TO_TICKS(250)) == pdTRUE) {
                if (flash_sudden_amp_content[0] == 0xFF || flash_sudden_amp_content[0] == 0x00) {
                    xSemaphoreGive(xFlashMutex);
                    continue;
                }
                memcpy(ac_record_buf, flash_sudden_amp_content + start_addr, FLASH_SECTOR_SIZE);
                xSemaphoreGive(xFlashMutex);
            }

            for (uint16_t j = 0; j < FLASH_SECTOR_SIZE; j++) {
                // send record as bytes
                xor_result ^= ac_record_buf[j];
                uart_putc(UART0_ID, ac_record_buf[j]);
            }

            // send cr and lf
            uart_putc(UART0_ID, '\r');
            xor_result ^= '\r';

            uart_putc(UART0_ID, '\n');
            xor_result ^= '\n';

            if (start_addr == end_addr) {
                // last sector and record control
                if (start_index > end_index && start_addr == ((FLASH_AMPLITUDE_RECORDS_TOTAL_SECTOR * FLASH_SECTOR_SIZE) - FLASH_SECTOR_SIZE)) {
                    start_addr = 0;
                    end_addr = end_index;
                } else {
                    uart_putc(UART0_ID, '\r');
                    xor_result ^= '\r';

                    uart_putc(UART0_ID, ETX);
                    xor_result ^= ETX;

                    PRINTF("GETTHRESHOLDRECORD: lp data block xor is: %02X\n", xor_result);
                    uart_putc(UART0_ID, xor_result);
                }
            }

            vTaskDelay(pdMS_TO_TICKS(15));

            // jump to next record
            start_addr += FLASH_SECTOR_SIZE;
        }
    } else {
        PRINTF("SEARCHDATAINFLASH: data not found.\n");
        sendErrorMessage((char *)"NODATAFOUND");
    }

    memset(reading_state_start_time, 0, 10);
    memset(reading_state_end_time, 0, 10);
}

void readTime() {
    char buffer[20] = {0};
    uint8_t xor_result = 0x02;

    int result = snprintf((char *)buffer, sizeof(buffer), "%c0.9.1(%02d:%02d:%02d)%c", 0x02, current_time.hour, current_time.min, current_time.sec, 0x03);

    if (result >= (int)sizeof(buffer)) {
        PRINTF("READTIME: Buffer Overflow! Sending NACK.\n");
        sendErrorMessage((char *)"TIMEBUFFEROVERFLOW");
        return;
    }

    bccGenerate((uint8_t *)buffer, result, &xor_result);

    PRINTF("READTIME: buffer to send is:\n");
    printBufferHex((uint8_t *)buffer, result);
    PRINTF("\n");

    uart_puts(UART0_ID, buffer);
    uart_putc(UART0_ID, xor_result);
}

void readDate() {
    char buffer[20] = {0};
    uint8_t xor_result = 0x02;

    int result = snprintf((char *)buffer, sizeof(buffer), "%c0.9.2(%02d:%02d:%02d)%c", 0x02, current_time.year, current_time.month, current_time.day, 0x03);
    if (result >= (int)sizeof(buffer)) {
        PRINTF("READDATE: Buffer Overflow! Sending NACK.\n");
        sendErrorMessage((char *)"DATEBUFFEROVERFLOW");
        return;
    }

    bccGenerate((uint8_t *)buffer, result, &xor_result);

    PRINTF("READDATE: buffer to send is:\n");
    printBufferHex((uint8_t *)buffer, result);
    PRINTF("\n");

    uart_puts(UART0_ID, buffer);
    uart_putc(UART0_ID, xor_result);
}

void readSerialNumber() {
    char buffer[22] = {0};
    uint8_t xor_result = 0x02;

    int result = snprintf((char *)buffer, sizeof(buffer), "%c0.0.0(%s)%c", 0x02, serial_number, 0x03);
    if (result >= (int)sizeof(buffer)) {
        PRINTF("READSERIALNUMBER: Buffer Overflow! Sending NACK.\n");
        sendErrorMessage((char *)"SERIALBUFFEROVERFLOW");
        return;
    }

    bccGenerate((uint8_t *)buffer, result, &xor_result);

    PRINTF("READSERIALNUMBER: buffer to send is:\n");
    printBufferHex((uint8_t *)buffer, result);
    PRINTF("\n");

    uart_puts(UART0_ID, buffer);
    uart_putc(UART0_ID, xor_result);
}

void sendLastVRMSXValue(enum ListeningStates vrmsState) {
    char buffer[20] = {0};
    int result = -1;
    uint8_t xor_result = 0x02;

    switch (vrmsState) {
        case ReadLastVRMSMax:
            result = snprintf((char *)buffer, sizeof(buffer), "%c32.7.0(%.2f)%c", 0x02, vrms_max_last, 0x03);
            break;

        case ReadLastVRMSMin:
            result = snprintf((char *)buffer, sizeof(buffer), "%c52.7.0(%.2f)%c", 0x02, vrms_min_last, 0x03);
            break;

        case ReadLastVRMSMean:
            result = snprintf((char *)buffer, sizeof(buffer), "%c72.7.0(%.2f)%c", 0x02, vrms_mean_last, 0x03);
            break;

        default:
            PRINTF("SENDLASTVRMSXVALUE: Unknown state!\n");
            break;
    }

    if (result == -1 || result >= (int)sizeof(buffer)) {
        PRINTF("SENDLASTVRMSXVALUE: Buffer Overflow or Unknown state! Sending NACK.\n");
        sendErrorMessage((char *)"VRMSBUFFEROVERFLOW");
        return;
    }

    bccGenerate((uint8_t *)buffer, result, &xor_result);

    PRINTF("SENDLASTVRMSXVALUE: buffer to send is:\n");
    printBufferHex((uint8_t *)buffer, result);
    PRINTF("\n");

    uart_puts(UART0_ID, buffer);
    uart_putc(UART0_ID, xor_result);
}

void sendThresholdObis() {
    char buffer[22] = {0};
    uint8_t xor_result = 0x02;

    int result = snprintf((char *)buffer, sizeof(buffer), "%c96.3.12(%03d)%c", 0x02, getVRMSThresholdValue(), 0x03);
    if (result >= (int)sizeof(buffer)) {
        PRINTF("SENDTHRESHOLDOBIS: Buffer Overflow! Sending NACK.\n");
        sendErrorMessage((char *)"SERIALBUFFEROVERFLOW");
        return;
    }

    bccGenerate((uint8_t *)buffer, result, &xor_result);

    PRINTF("SENDTHRESHOLDOBIS: buffer to send is:\n");
    printBufferHex((uint8_t *)buffer, result);
    PRINTF("\n");

    uart_puts(UART0_ID, buffer);
    uart_putc(UART0_ID, xor_result);
}
