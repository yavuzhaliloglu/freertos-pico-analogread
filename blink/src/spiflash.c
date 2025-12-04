#include "header/spiflash.h"
#include "FreeRTOS.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "header/bcc.h"
#include "header/fifo.h"
#include "header/mutex.h"
#include "header/print.h"
#include "header/project_globals.h"
#include "pico/stdlib.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

uint8_t read_sr(uint8_t sr_index) {
    uint8_t tx[1];

    if (sr_index == 1) {
        // tx[2] = {0x05, 0x00}; // Read Status Register 1 command
        memcpy(tx, (uint8_t[]){0x05}, sizeof(tx));
    } else if (sr_index == 2) {
        // uint8_t tx[2] = {0x35, 0x00}; // Read Status Register 2 command
        memcpy(tx, (uint8_t[]){0x35}, sizeof(tx));
    } else if (sr_index == 3) {
        // uint8_t tx[2] = {0x15, 0x00}; // Read Status Register 3 command
        memcpy(tx, (uint8_t[]){0x15}, sizeof(tx));
    } else {
        PRINTF("Invalid SR index: %d\n", sr_index);
        return 255;
    }

    uint8_t rx[1 + 1];

    // Fill rx with zeros (not required but cleaner)
    memset(rx, 0, sizeof(rx));

    PRINTF("Sending command to read SR: %02X\n", tx[0]);

    // Execute the flash command
    flash_do_cmd(tx, rx, sizeof(rx));

    // First 5 bytes are command, next 8 bytes are response
    PRINTF("SR Content:\n");
    for (int i = 1; i < 2; ++i) {
        PRINTF("0x%02X\n", rx[i]);
        PRINTF("\n");
        return rx[i]; // Return the value of the requested status register
    }

    return 255;
}

void __not_in_flash_func(send_write_enable_command)() {
    uint8_t tx_wel[1] = {0x50}; // Write Enable command
    flash_do_cmd(tx_wel, NULL, 1);
    PRINTF("Write Enable command sent.\n");
}

void __not_in_flash_func(send_write_protect_command)() {
    uint8_t tx_wsr1[2] = {0x01, 0x2C};
    flash_do_cmd(tx_wsr1, NULL, 2);
    PRINTF("Write Protect command sent.\n");
}

void read_flash_status_registers() {
    read_sr(1);
    read_sr(2);
    read_sr(3);
    PRINTF("Flash status registers read.\n");
}

// This function gets the contents like sector data, last records contents from
// flash and sets them to variables.
void getFlashContents() {
    // get sector count of records
    uint16_t *flash_sector_content =
        (uint16_t *)(XIP_BASE + FLASH_LOAD_PROFILE_LAST_SECTOR_DATA_ADDR);
    sector_data = flash_sector_content[0];

    // get record data
    uint8_t *flash_data_contents =
        (uint8_t *)(XIP_BASE + FLASH_LOAD_PROFILE_RECORD_ADDR +
                    (sector_data * FLASH_SECTOR_SIZE));
    memcpy(flash_data, flash_data_contents, FLASH_SECTOR_SIZE);

    // get threshold data
    uint16_t *th_ptr = (uint16_t *)(XIP_BASE + FLASH_THRESHOLD_PARAMETERS_ADDR);
    vrms_threshold = th_ptr[0];
    th_sector_data = th_ptr[1];

    // set serial number
    uint8_t *serial_number_offset =
        (uint8_t *)(XIP_BASE + FLASH_SERIAL_NUMBER_ADDR);
    memcpy(serial_number, serial_number_offset, SERIAL_NUMBER_SIZE);

    PRINTF("GETFLASHCONTENTS: vrms threshold value is: %d\n", vrms_threshold);
    PRINTF("GETFLASHCONTENTS: flash sector is: %d\n", sector_data);
    PRINTF("GETFLASHCONTENTS: threshold sector is: %d\n", th_sector_data);
}

// This function writes current sector data to flash.
void __not_in_flash_func(setSectorData)(uint16_t sector_value) {
    uint16_t sector_buffer[FLASH_PAGE_SIZE / sizeof(uint16_t)] = {0};
    sector_buffer[0] = sector_value;

    PRINTF("SETSECTORDATA: sector data which is going to be written: %d\n",
           sector_value);

    if (xSemaphoreTake(xFlashMutex, pdMS_TO_TICKS(250)) == pdTRUE) {
        PRINTF("SETSECTORDATA: write flash mutex received\n");
        uint32_t ints = save_and_disable_interrupts();
        flash_range_erase(FLASH_LOAD_PROFILE_LAST_SECTOR_DATA_ADDR,
                          FLASH_LOAD_PROFILE_LAST_SECTOR_DATA_SIZE);
        flash_range_program(FLASH_LOAD_PROFILE_LAST_SECTOR_DATA_ADDR,
                            (uint8_t *)sector_buffer, FLASH_PAGE_SIZE);
        restore_interrupts(ints);
        xSemaphoreGive(xFlashMutex);
    } else {
        PRINTF("MUTEX CANNOT RECEIVED!\n");
        led_blink_pattern(LED_ERROR_CODE_FLASH_MUTEX_NOT_TAKEN);
    }
}

// This function converts the datetime value to char array as characters to
// write flash correctly
void setDateToCharArray(int value, char *array) {
    // if value is smaller than 10, second character of array will be always 0,
    // first element of array will be value
    if (value < 10) {
        array[0] = '0';
        array[1] = value + '0';
    }
    // if value is bigger than 10, two character will be written to char array,
    // first element of array keeps ones place, second element of array keeps
    // tens place
    else {
        array[0] = value / 10 + '0';
        array[1] = value % 10 + '0';
    }
}

// this function converts a float value's floating value to uint8_t value.
uint8_t floatDecimalDigitToUint8t(float float_value) {
    // get floating value of float value and multiply it with 10, so we can get
    // the first digit. We set that value an uint8_t value because we don't want
    // to get rest of the floating digits.
    uint8_t floating_value = (float_value - (float)floor(float_value)) * 10;

    PRINTF("float value before subtraction: %lf\n", float_value);
    PRINTF("floating value after floor and uint8_t: %d\n\n", floating_value);

    return floating_value;
}

// This function gets a buffer which includes VRMS values, and calculate the
// max, min and mean values of this buffer and sets the variables.
VRMS_VALUES_RECORD vrmsSetMinMaxMean(float *buffer, uint16_t size) {
    float buffer_max = buffer[0];
    float buffer_min = buffer[0];
    float buffer_sum = buffer[0];
    VRMS_VALUES_RECORD vrms_values;

    for (uint16_t i = 1; i < size; i++) {
        if (buffer[i] > buffer_max) {
            buffer_max = buffer[i];
        }

        if (buffer[i] < buffer_min) {
            buffer_min = buffer[i];
        }

        buffer_sum += buffer[i];
    }

    vrms_values.vrms_max = (uint8_t)floor(buffer_max);
    vrms_values.vrms_min = (uint8_t)floor(buffer_min);
    vrms_values.vrms_max_dec = floatDecimalDigitToUint8t(buffer_max);
    vrms_values.vrms_min_dec = floatDecimalDigitToUint8t(buffer_min);

    if (size == 0) {
        vrms_values.vrms_mean = 0;
        vrms_values.vrms_mean_dec = 0;
    } else {
        vrms_values.vrms_mean = (uint8_t)(buffer_sum / size);
        vrms_values.vrms_mean_dec =
            floatDecimalDigitToUint8t(buffer_sum / size);
    }

    PRINTF("buffer max: %lf, vrms_max: %d, vrms_max_dec: %d\n", buffer_max,
           vrms_values.vrms_max, vrms_values.vrms_max_dec);
    PRINTF("buffer min: %lf, vrms_min: %d, vrms_min_dec: %d\n", buffer_min,
           vrms_values.vrms_min, vrms_values.vrms_min_dec);
    PRINTF("buffer mean: %lf, vrms_mean: %d, vrms_mean_dec: %d\n",
           buffer_sum / size, vrms_values.vrms_mean, vrms_values.vrms_mean_dec);

    return vrms_values;
}

float convertVRMSValueToFloat(uint8_t value, uint8_t value_dec) {
    return value + value_dec / 10.0;
}

// This function sets the current time values which are 16 bytes total and
// calculated VRMS values to flash
void setFlashData(VRMS_VALUES_RECORD *vrms_values) {
    // initialize the variables
    struct FlashData data;
    uint8_t *flash_data_contents =
        (uint8_t *)(XIP_BASE + FLASH_LOAD_PROFILE_RECORD_ADDR +
                    (sector_data * FLASH_SECTOR_SIZE));
    uint16_t offset = 0;

    // set date values and VRMS values to FlashData struct variable
    setDateToCharArray(current_time.year, data.year);
    setDateToCharArray(current_time.month, data.month);
    setDateToCharArray(current_time.day, data.day);
    setDateToCharArray(current_time.hour, data.hour);
    setDateToCharArray(current_time.min, data.min);
    data.max_volt = vrms_values->vrms_max;
    data.min_volt = vrms_values->vrms_min;
    data.mean_volt = vrms_values->vrms_mean;
    data.max_volt_dec = vrms_values->vrms_max_dec;
    data.min_volt_dec = vrms_values->vrms_min_dec;
    data.mean_volt_dec = vrms_values->vrms_mean_dec;

    if (xSemaphoreTake(xVRMSLastValuesMutex, pdMS_TO_TICKS(250)) == pdTRUE) {
        // convert last record vrms values to float
        vrms_max_last = convertVRMSValueToFloat(vrms_values->vrms_max,
                                                vrms_values->vrms_max_dec);
        vrms_min_last = convertVRMSValueToFloat(vrms_values->vrms_min,
                                                vrms_values->vrms_min_dec);
        vrms_mean_last = convertVRMSValueToFloat(vrms_values->vrms_mean,
                                                 vrms_values->vrms_mean_dec);

        xSemaphoreGive(xVRMSLastValuesMutex);
    } else {
        led_blink_pattern(LED_ERROR_CODE_VRMS_VALUES_MUTEX_NOT_TAKEN);
        vrms_max_last = 0.0;
        vrms_min_last = 0.0;
        vrms_mean_last = 0.0;
    }

    if (xSemaphoreTake(xFlashMutex, pdMS_TO_TICKS(250)) == pdTRUE) {
        PRINTF("SETFLASHDATA: offset loop mutex received\n");
        // find the last offset of flash records and write current values to
        // last offset of flash_data buffer
        for (offset = 0; offset < FLASH_SECTOR_SIZE;
             offset += FLASH_RECORD_SIZE) {
            if (flash_data_contents[offset] == '\0' ||
                flash_data_contents[offset] == 0xff) {
                if (offset == 0) {
                    PRINTF("SETFLASHDATA: last record is not found.\n");
                } else {
                    PRINTF("SETFLASHDATA: last record is start in %d offset\n",
                           offset - 16);
                }

                flash_data[offset / FLASH_RECORD_SIZE] = data;

                PRINTF("SETFLASHDATA: record saved to offset: %d. used %d/%d "
                       "of sector.\n",
                       offset, offset + 16, FLASH_SECTOR_SIZE);

                break;
            }
        }

        xSemaphoreGive(xFlashMutex);
    } else {
        led_blink_pattern(LED_ERROR_CODE_FLASH_MUTEX_NOT_TAKEN);
        return;
    }

    // if offset value is equals or bigger than FLASH_SECTOR_SIZE, (4096 bytes)
    // it means current sector is full and program should write new values to
    // next sector
    if (offset >= FLASH_SECTOR_SIZE) {
        PRINTF("SETFLASHDATA: offset value is equals to sector size. Current "
               "sector data is: %d. Sector is changing...\n",
               sector_data);

        // if current sector is last sector of flash, sector data will be 0 and
        // the program will start to write new records to beginning of the flash
        // record offset
        if (sector_data == FLASH_LOAD_PROFILE_AREA_TOTAL_SECTOR_COUNT - 1) {
            sector_data = 0;
        } else {
            sector_data++;
        }

        PRINTF("SETFLASHDATA: new sector value is: %d\n", sector_data);

        // reset variables and call setSectorData()
        memset(flash_data, 0, FLASH_SECTOR_SIZE);
        flash_data[0] = data;
        setSectorData(sector_data);

        PRINTF("SETFLASHDATA: Sector changing written to flash.\n");
    }
}

// This function writes flash_data content to flash area
void __not_in_flash_func(SPIWriteToFlash)(VRMS_VALUES_RECORD *vrms_values) {
    PRINTF("SPIWRITETOFLASH: Setting flash data...\n");
    setFlashData(vrms_values);

    if (xSemaphoreTake(xFlashMutex, pdMS_TO_TICKS(250)) == pdTRUE) {
        PRINTF("SPIWRITETOFLASH: write flash mutex received\n");
        uint32_t ints = save_and_disable_interrupts();
        flash_range_erase(FLASH_LOAD_PROFILE_RECORD_ADDR +
                              (sector_data * FLASH_SECTOR_SIZE),
                          FLASH_SECTOR_SIZE);
        flash_range_program(FLASH_LOAD_PROFILE_RECORD_ADDR +
                                (sector_data * FLASH_SECTOR_SIZE),
                            (uint8_t *)flash_data, FLASH_SECTOR_SIZE);
        restore_interrupts(ints);
        xSemaphoreGive(xFlashMutex);
    } else {
        PRINTF("MUTEX CANNOT RECEIVED!\n");
        led_blink_pattern(LED_ERROR_CODE_FLASH_MUTEX_NOT_TAKEN);
    }
}

// this function converts an array to datetime value
void arrayToDatetime(datetime_t *dt, uint8_t *arr) {
    dt->year = (arr[0] - '0') * 10 + (arr[1] - '0');
    dt->month = (arr[2] - '0') * 10 + (arr[3] - '0');
    dt->day = (arr[4] - '0') * 10 + (arr[5] - '0');
    dt->hour = (arr[6] - '0') * 10 + (arr[7] - '0');
    dt->min = (arr[8] - '0') * 10 + (arr[9] - '0');
}

// This functon compares two datetime values and return an int value
int datetimeComp(datetime_t *dt1, datetime_t *dt2) {
    if (dt1->year - dt2->year != 0) {
        return dt1->year - dt2->year;
    }

    else if (dt1->month - dt2->month != 0) {
        return dt1->month - dt2->month;
    }

    else if (dt1->day - dt2->day != 0) {
        return dt1->day - dt2->day;
    }

    else if (dt1->hour - dt2->hour != 0) {
        return dt1->hour - dt2->hour;
    }

    else if (dt1->min - dt2->min != 0) {
        return dt1->min - dt2->min;
    }

    else if (dt1->sec - dt2->sec != 0) {
        return dt1->sec - dt2->sec;
    }

    return 0;
}

// This function copies a datetime value to another datetime value
void datetimeCopy(datetime_t *src, datetime_t *dst) {
    dst->year = src->year;
    dst->month = src->month;
    dst->day = src->day;
    dst->dotw = src->dotw;
    dst->hour = src->hour;
    dst->min = src->min;
    dst->sec = src->sec;
}

uint8_t is_datetime_empty(datetime_t *dt) {
    if (dt->year == 0 && dt->month == 0 && dt->day == 0 && dt->hour == 0 &&
        dt->min == 0 && dt->sec == 0) {
        return 1;
    } else {
        return 0;
    }
}

uint8_t get_record_indexes(int64_t *start, int64_t *end, datetime_t *dt_start, datetime_t *dt_end) {
    datetime_t start_dt_local = {0};
    datetime_t end_dt_local = {0};
    uint8_t start_dt_empty_flag = 0;
    uint8_t end_dt_empty_flag = 0;

    memcpy(&start_dt_local, dt_start, sizeof(datetime_t));
    memcpy(&end_dt_local, dt_end, sizeof(datetime_t));

    if (xSemaphoreTake(xFlashMutex, pdMS_TO_TICKS(250)) == pdTRUE) {

        uint8_t *flash_records_ptr = (uint8_t *)(XIP_BASE + FLASH_LOAD_PROFILE_RECORD_ADDR);

        if (is_datetime_empty(dt_start)) {
            PRINTF("GETRECORDINDEXES: start datetime is empty, setting to first record datetime\n");
            arrayToDatetime(dt_start, &flash_records_ptr[0]);
            start_dt_empty_flag = 1;
            PRINTF("Datetime set to: %02d-%02d-%02d %02d:%02d\n",
                   dt_start->year, dt_start->month, dt_start->day,
                   dt_start->hour, dt_start->min);
        }
        if (is_datetime_empty(dt_end)) {
            PRINTF("GETRECORDINDEXES: end datetime is empty, setting to first record datetime\n");
            arrayToDatetime(dt_end, &flash_records_ptr[0]);
            end_dt_empty_flag = 1;
            PRINTF("Datetime set to: %02d-%02d-%02d %02d:%02d\n",
                   dt_end->year, dt_end->month, dt_end->day,
                   dt_end->hour, dt_end->min);
        }

        PRINTF("LOAD PROFILE CONTENT:\n");
        for (uint32_t i = 0; i < FLASH_LOAD_PROFILE_RECORD_AREA_SIZE; i += FLASH_RECORD_SIZE) {
            // if current index is empty, continue
            if (flash_records_ptr[i] == 0xFF || flash_records_ptr[i] == 0x00) {
                continue;
            }

            datetime_t temp_dt = {0};
            arrayToDatetime(&temp_dt, &flash_records_ptr[i]);
            PRINTF("Record at offset %ld: %02d-%02d-%02d %02d:%02d\n", i,
                   temp_dt.year, temp_dt.month, temp_dt.day,
                   temp_dt.hour, temp_dt.min);
        }

        for (uint32_t i = 0; i < FLASH_LOAD_PROFILE_RECORD_AREA_SIZE; i += FLASH_RECORD_SIZE) {

            // if current index is empty, continue
            if (flash_records_ptr[i] == 0xFF || flash_records_ptr[i] == 0x00) {
                continue;
            }

            // if current index is not empty, set datetime to current index record
            datetime_t recurrent_time = {0};
            arrayToDatetime(&recurrent_time, &flash_records_ptr[i]);

            if (start_dt_empty_flag) {
                if (datetimeComp(&recurrent_time, dt_start) <= 0) {
                    *start = i;
                    datetimeCopy(&recurrent_time, &start_dt_local);
                }
            } else {
                // if current record datetime  is bigger than start datetime and start index is not set, this is the start index
                if (datetimeComp(&recurrent_time, dt_start) >= 0) {
                    if (is_datetime_empty(&start_dt_local)) {
                        *start = i;
                        datetimeCopy(&recurrent_time, &start_dt_local);
                    } else if (*start == -1 || (datetimeComp(&recurrent_time, &start_dt_local) < 0)) {
                        *start = i;
                        datetimeCopy(&recurrent_time, &start_dt_local);
                    }
                }
            }

            if (end_dt_empty_flag) {
                if (datetimeComp(&recurrent_time, dt_end) >= 0) {
                    *end = i;
                    datetimeCopy(&recurrent_time, &end_dt_local);
                }
            } else {
                // if current record datetime is smaller than end datetime and end index is not set, this is the end index
                if (datetimeComp(&recurrent_time, dt_end) <= 0) {
                    if (is_datetime_empty(&end_dt_local)) {
                        *end = i;
                        datetimeCopy(&recurrent_time, &end_dt_local);
                    } else if (*end == -1 || (datetimeComp(&recurrent_time, &end_dt_local) > 0)) {
                        *end = i;
                        datetimeCopy(&recurrent_time, &end_dt_local);
                    }
                }
            }
        }

        xSemaphoreGive(xFlashMutex);
        return 1;
    } else {
        PRINTF("GETRECORDINDEXES: Could not take flash mutex!\n");
        led_blink_pattern(LED_ERROR_CODE_FLASH_MUTEX_NOT_TAKEN);
        return 0;
    }
}

bool check_datetime_format(datetime_t *dt) {
    if (dt->year < 0 || dt->year > 99) {
        return false;
    }

    if (dt->month < 0 || dt->month > 12) {
        return false;
    }

    if (dt->day < 0 || dt->day > 31) {
        return false;
    }

    if (dt->hour < 0 || dt->hour > 23) {
        return false;
    }

    if (dt->min < 0 || dt->min > 59) {
        return false;
    }

    return true;
}

void add_date_to_buffer(char *start_ptr, char *end_ptr, uint8_t *date_arr) {
    size_t date_length = end_ptr - start_ptr - 1;
    uint8_t date_idx = 0;

    if (date_length < 10 && date_length > 14) {
        PRINTF("Date length is wrong: %zu\n", date_length);
        return;
    }

    start_ptr++;

    PRINTF("Date length: %zu\n", date_length);
    PRINTF("Date string: %.*s\n", (int)date_length, start_ptr);
    PRINTF("\n");

    for (uint8_t i = 0; i < date_length; i++) {
        if (start_ptr[i] == '-' || start_ptr[i] == ',' || start_ptr[i] == ':') {
            continue;
        }
        date_arr[date_idx++] = start_ptr[i];
    }

    date_arr[date_idx] = '\0';
}

uint8_t parse_load_profile_dates(uint8_t *buf, datetime_t *dt_start, datetime_t *dt_end) {
    char *lp_start_ptr = NULL;
    char *lp_end_ptr = NULL;
    char *lp_date_seperator_ptr = NULL;
    uint8_t start_date_arr[16];
    uint8_t end_date_arr[16];

    memset(start_date_arr, 0, sizeof(start_date_arr));
    memset(end_date_arr, 0, sizeof(end_date_arr));

    lp_start_ptr = strchr((char *)buf, '(');
    lp_end_ptr = strchr((char *)buf, ')');
    lp_date_seperator_ptr = strchr((char *)buf, ';');

    if (lp_start_ptr == NULL || lp_end_ptr == NULL || lp_date_seperator_ptr == NULL || lp_end_ptr <= lp_start_ptr) {
        return 0;
    } else if (lp_start_ptr + 1 == lp_date_seperator_ptr && lp_date_seperator_ptr + 1 == lp_end_ptr) {
        return 1;
    } else if (lp_start_ptr + 1 == lp_date_seperator_ptr && lp_date_seperator_ptr + 1 != lp_end_ptr) {
        add_date_to_buffer(lp_date_seperator_ptr, lp_end_ptr, end_date_arr);
        arrayToDatetime(dt_end, end_date_arr);
        return 2;
    } else if (lp_start_ptr + 1 != lp_date_seperator_ptr && lp_date_seperator_ptr + 1 == lp_end_ptr) {
        add_date_to_buffer(lp_start_ptr, lp_date_seperator_ptr, start_date_arr);
        arrayToDatetime(dt_start, start_date_arr);

        return 3;
    }

    add_date_to_buffer(lp_start_ptr, lp_date_seperator_ptr, start_date_arr);
    add_date_to_buffer(lp_date_seperator_ptr, lp_end_ptr, end_date_arr);

    arrayToDatetime(dt_start, start_date_arr);
    arrayToDatetime(dt_end, end_date_arr);

    PRINTF("Parsed Start Date: %s\n", start_date_arr);
    PRINTF("Parsed End Date: %s\n", end_date_arr);

    PRINTF("DATE START DATETIME:\n");
    PRINTF("Year: %d, Month: %d, Day: %d, Hour: %d, Min: %d\n",
           dt_start->year, dt_start->month, dt_start->day, dt_start->hour,
           dt_start->min);
    PRINTF("\n");
    PRINTF("DATE END DATETIME:\n");
    PRINTF("Year: %d, Month: %d, Day: %d, Hour: %d, Min: %d\n",
           dt_end->year, dt_end->month, dt_end->day, dt_end->hour, dt_end->min);
    PRINTF("\n");

    return 4;
}

// This function searches the requested data in flash by starting from flash
// record beginning offset, collects data from flash and sends it to UART to
// show load profile content
void send_load_profile_records(uint8_t *buf) {
    datetime_t dt_start;
    datetime_t dt_end;
    int64_t start_index = -1;
    int64_t end_index = -1;
    char load_profile_line[48] = {0};
    uint8_t *flash_start_content = (uint8_t *)(XIP_BASE + FLASH_LOAD_PROFILE_RECORD_ADDR);

    char year[3] = {0};
    char month[3] = {0};
    char day[3] = {0};
    char hour[3] = {0};
    char minute[3] = {0};
    uint8_t max = 0;
    uint8_t max_dec = 0;
    uint8_t min = 0;
    uint8_t min_dec = 0;
    uint8_t mean = 0;
    uint8_t mean_dec = 0;

    memset(&dt_start, 0, sizeof(datetime_t));
    memset(&dt_end, 0, sizeof(datetime_t));

    uint8_t parse_result = parse_load_profile_dates(buf, &dt_start, &dt_end);

    if (!check_datetime_format(&dt_start) || !check_datetime_format(&dt_end) || parse_result == 0) {
        PRINTF("SEARCHDATAINFLASH: Date format is wrong.\n");
        sendErrorMessage((char *)"LPDATEFORMAT");
        return;
    }

    uint8_t result = get_record_indexes(&start_index, &end_index, &dt_start, &dt_end);

    if (result == 0 || (start_index == -1 && end_index == -1)) {
        PRINTF("SEARCHDATAINFLASH: Error occurred while getting record indexes.\n");
        sendErrorMessage((char *)"LPGETRECIDXERR");
        return;
    }
    PRINTF("SEARCHDATAINFLASH: Start index is: %lld\n", start_index);
    PRINTF("SEARCHDATAINFLASH: End index is: %lld\n", end_index);

    // if there start and end index are set, there are records between these
    // times so send them to UART
    if (start_index >= 0 && end_index >= 0) {
        PRINTF("SEARCHDATAINFLASH: Generating messages...\n");

        // initialize the variables
        uint8_t xor_result = 0x00;
        uint32_t start_addr = start_index;
        uint32_t end_addr =
            start_index <= end_index
                ? end_index
                : (int32_t)(FLASH_LOAD_PROFILE_RECORD_AREA_SIZE -
                            FLASH_RECORD_SIZE);
        int result;

        // send STX character
        uart_putc(UART0_ID, STX);

        while (start_addr <= end_addr) {
            if (xSemaphoreTake(xFlashMutex, pdMS_TO_TICKS(250)) == pdTRUE) {
                if (flash_start_content[start_addr] == 0xFF ||
                    flash_start_content[start_addr] == 0x00) {
                    start_addr += FLASH_RECORD_SIZE;
                    xSemaphoreGive(xFlashMutex);
                    continue;
                }
                PRINTF("SEARCHDATAINFLASH: set data mutex received\n");

                // set char arrays to initialize a string for record
                snprintf(year, sizeof(year), "%c%c",
                         flash_start_content[start_addr],
                         flash_start_content[start_addr + 1]);
                snprintf(month, sizeof(month), "%c%c",
                         flash_start_content[start_addr + 2],
                         flash_start_content[start_addr + 3]);
                snprintf(day, sizeof(day), "%c%c",
                         flash_start_content[start_addr + 4],
                         flash_start_content[start_addr + 5]);
                snprintf(hour, sizeof(hour), "%c%c",
                         flash_start_content[start_addr + 6],
                         flash_start_content[start_addr + 7]);
                snprintf(minute, sizeof(minute), "%c%c",
                         flash_start_content[start_addr + 8],
                         flash_start_content[start_addr + 9]);
                max = flash_start_content[start_addr + 10];
                max_dec = flash_start_content[start_addr + 11];
                min = flash_start_content[start_addr + 12];
                min_dec = flash_start_content[start_addr + 13];
                mean = flash_start_content[start_addr + 14];
                mean_dec = flash_start_content[start_addr + 15];

                xSemaphoreGive(xFlashMutex);
            } else {
                led_blink_pattern(LED_ERROR_CODE_FLASH_MUTEX_NOT_TAKEN);
                continue;
            }

            result = snprintf(load_profile_line, sizeof(load_profile_line),
                              "(%s-%s-%s,%s:%s)(%03d.%d,%03d.%d,%03d.%d)\r\n",
                              year, month, day, hour, minute, min, min_dec, max,
                              max_dec, mean, mean_dec);
            bccGenerate((uint8_t *)load_profile_line, 40, &xor_result);
            PRINTF("SEARCHDATAINFLASH: message to send: %s\n",
                   load_profile_line);

            // send the record to UART and wait
            PRINTF("SEARCHDATAINFLASH: Record to send as bytearray: \n");
            printBufferHex((uint8_t *)load_profile_line, 41);
            PRINTF("\n");

            if (result >= (int)sizeof(load_profile_line)) {
                PRINTF("SEARCHDATAINFLASH: Buffer Overflow! Sending NACK.\n");
                sendErrorMessage((char *)"LPBUFFEROVERFLOW");
            } else {
                uart_puts(UART0_ID, load_profile_line);
            }

            // if start address equals to end address, it means there is just
            // one record to send or this record is the last record to send
            if (start_addr == end_addr) {
                if (start_index > end_index &&
                    start_addr == FLASH_LOAD_PROFILE_RECORD_AREA_SIZE -
                                      FLASH_RECORD_SIZE) {
                    start_addr = 0;
                    end_addr = end_index;
                } else {
                    result = snprintf(load_profile_line,
                                      sizeof(load_profile_line), "\r%c", ETX);
                    bccGenerate((uint8_t *)load_profile_line, result,
                                &xor_result);

                    uart_puts(UART0_ID, load_profile_line);
                    PRINTF("SEARCHDATAINFLASH: lp data block xor is: %02X\n",
                           xor_result);
                    uart_putc(UART0_ID, xor_result);
                }
            }

            vTaskDelay(pdMS_TO_TICKS(15));
            start_addr += FLASH_RECORD_SIZE;
        }

        return;
    }

    PRINTF("SEARCHDATAINFLASH: data not found.\n");
    sendErrorMessage((char *)"NODATAFOUND");
}

void __not_in_flash_func(checkSectorContent)() {
    uint16_t *flash_sector_content =
        (uint16_t *)(XIP_BASE + FLASH_LOAD_PROFILE_LAST_SECTOR_DATA_ADDR);

    if (flash_sector_content[0] == 0xFFFF) {
        PRINTF("CHECKSECTORCONTENT: sector area is empty. Sector content is "
               "going to set 0.\n");
        uint16_t sector_buffer[FLASH_PAGE_SIZE / sizeof(uint16_t)] = {0};

        uint32_t ints = save_and_disable_interrupts();
        flash_range_erase(FLASH_LOAD_PROFILE_LAST_SECTOR_DATA_ADDR,
                          FLASH_LOAD_PROFILE_LAST_SECTOR_DATA_SIZE);
        flash_range_program(FLASH_LOAD_PROFILE_LAST_SECTOR_DATA_ADDR,
                            (uint8_t *)sector_buffer, FLASH_PAGE_SIZE);
        restore_interrupts(ints);
    }
}

void __not_in_flash_func(checkThresholdContent)() {
    uint16_t *th_ptr = (uint16_t *)(XIP_BASE + FLASH_THRESHOLD_PARAMETERS_ADDR);

    // Threshold value control
    if (th_ptr[0] == 0xFFFF) {
        PRINTF("threshold value is empty, setting to 5 as default...\n");

        uint16_t th_arr[256 / sizeof(uint16_t)] = {0};
        th_arr[0] = vrms_threshold;
        th_arr[1] = th_ptr[1];

        uint32_t ints = save_and_disable_interrupts();
        flash_range_erase(FLASH_THRESHOLD_PARAMETERS_ADDR,
                          FLASH_THRESHOLD_PARAMETERS_SIZE);
        flash_range_program(FLASH_THRESHOLD_PARAMETERS_ADDR, (uint8_t *)th_arr,
                            FLASH_PAGE_SIZE);
        restore_interrupts(ints);
    }
    // Threshold Records Sector control
    if (th_ptr[1] == 0xFFFF) {
        PRINTF("threshold record's sector value is empty, setting to 0 as "
               "default...\n");

        uint16_t th_arr[256 / sizeof(uint16_t)] = {0};
        th_arr[0] = th_ptr[0];
        th_arr[1] = 0;

        uint32_t ints = save_and_disable_interrupts();
        flash_range_erase(FLASH_THRESHOLD_PARAMETERS_ADDR,
                          FLASH_THRESHOLD_PARAMETERS_SIZE);
        flash_range_program(FLASH_THRESHOLD_PARAMETERS_ADDR, (uint8_t *)th_arr,
                            FLASH_PAGE_SIZE);
        restore_interrupts(ints);
    }
}

void __not_in_flash_func(updateThresholdSector)(uint16_t sector_val) {
    uint16_t th_arr[256 / sizeof(uint16_t)] = {0};

    th_arr[0] = getVRMSThresholdValue();
    th_arr[1] = sector_val;

    if (xSemaphoreTake(xFlashMutex, pdMS_TO_TICKS(250)) == pdTRUE) {
        PRINTF("UPDATETHRESHOLDSECTOR: write flash mutex received\n");
        uint32_t ints = save_and_disable_interrupts();
        flash_range_erase(FLASH_THRESHOLD_PARAMETERS_ADDR,
                          FLASH_THRESHOLD_PARAMETERS_SIZE);
        flash_range_program(FLASH_THRESHOLD_PARAMETERS_ADDR, (uint8_t *)th_arr,
                            FLASH_PAGE_SIZE);
        restore_interrupts(ints);
        xSemaphoreGive(xFlashMutex);
    } else {
        PRINTF("MUTEX CANNOT RECEIVED!\n");
        led_blink_pattern(LED_ERROR_CODE_FLASH_MUTEX_NOT_TAKEN);
    }
}

#if WITHOUT_BOOTLOADER
// This function adds serial number to flash area
void __not_in_flash_func(addSerialNumber)() {
    PRINTF("ADDSERIALNUMBER: entered addserialnumber function.\n");

    uint8_t *snumber = (uint8_t *)(XIP_BASE + FLASH_SERIAL_NUMBER_ADDR);

    if (snumber[0] == 0xFF) {
        PRINTF("ADDSERIALNUMBER: serial number is going to be added.\n");

        uint32_t ints = save_and_disable_interrupts();
        flash_range_erase(FLASH_SERIAL_NUMBER_ADDR,
                          FLASH_SERIAL_NUMBER_AREA_SIZE);
        flash_range_program(FLASH_SERIAL_NUMBER_ADDR, (const uint8_t *)s_number,
                            FLASH_PAGE_SIZE);
        restore_interrupts(ints);
    }
}
#endif

void __not_in_flash_func(setProgramStartDate)(datetime_t *ct) {
    uint8_t *flash_reset_count_offset =
        (uint8_t *)(XIP_BASE + FLASH_RESET_DATES_ADDR);
    uint8_t current_time_buffer[16] = {0};
    uint8_t flash_reset_count_buffer[FLASH_SECTOR_SIZE] = {0};
    uint16_t offset = 0;

    memcpy(flash_reset_count_buffer, flash_reset_count_offset,
           FLASH_SECTOR_SIZE);

    setDateToCharArray(ct->year, (char *)current_time_buffer);
    setDateToCharArray(ct->month, (char *)current_time_buffer + 2);
    setDateToCharArray(ct->day, (char *)current_time_buffer + 4);
    setDateToCharArray(ct->hour, (char *)current_time_buffer + 6);
    setDateToCharArray(ct->min, (char *)current_time_buffer + 8);
    setDateToCharArray(ct->sec, (char *)current_time_buffer + 10);

    current_time_buffer[12] = 0x7F;
    current_time_buffer[13] = 0x7F;
    current_time_buffer[14] = 0x7F;
    current_time_buffer[15] = 0x7F;

    PRINTF("SETPROGRAMSTARTDATE: Program start date is set to: ");
    printBufferHex(current_time_buffer, 10);
    PRINTF("\n");

    for (uint16_t i = 0; i < FLASH_SECTOR_SIZE; i += 16) {
        if (flash_reset_count_buffer[offset] == 0xFF ||
            flash_reset_count_offset[offset] == 0x00) {
            break;
        }

        offset += 16;
    }

    if (offset >= FLASH_SECTOR_SIZE) {
        memset(flash_reset_count_buffer, 0, FLASH_SECTOR_SIZE);
        offset = 0;
    }

    memcpy(flash_reset_count_buffer + offset, current_time_buffer,
           sizeof(current_time_buffer));

    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_RESET_DATES_ADDR, FLASH_RESET_DATES_AREA_SIZE);
    flash_range_program(FLASH_RESET_DATES_ADDR, flash_reset_count_buffer,
                        FLASH_SECTOR_SIZE);
    restore_interrupts(ints);

    printBufferHex(flash_reset_count_offset, FLASH_PAGE_SIZE);
}