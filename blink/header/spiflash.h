#ifndef SPIFLASH_H
#define SPIFLASH_H

#include <stdint.h>
#include <stddef.h>
#include "pico/util/datetime.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "header/project_globals.h"

uint8_t read_sr(uint8_t sr_index);
void __not_in_flash_func(send_write_enable_command)();
void __not_in_flash_func(send_write_protect_command)();
void read_flash_status_registers();
// This function gets the contents like sector data, last records contents from flash and sets them to variables.
void getFlashContents();
// This function writes current sector data to flash.
void __not_in_flash_func(setSectorData)(uint16_t sector_value);
// This function converts the datetime value to char array as characters to write flash correctly
void setDateToCharArray(int value, char *array);
// this function converts a float value's floating value to uint8_t value.
uint8_t floatDecimalDigitToUint8t(float float_value);
// This function gets a buffer which includes VRMS values, and calculate the max, min and mean values of this buffer and sets the variables.
VRMS_VALUES_RECORD vrmsSetMinMaxMean(float *buffer, uint16_t size);
float convertVRMSValueToFloat(uint8_t value, uint8_t value_dec);
// This function sets the current time values which are 16 bytes total and calculated VRMS values to flash
void setFlashData(VRMS_VALUES_RECORD *vrms_values);
// This function writes flash_data content to flash area
void __not_in_flash_func(SPIWriteToFlash)(VRMS_VALUES_RECORD *vrms_values);
// this function converts an array to datetime value
void arrayToDatetime(datetime_t *dt, uint8_t *arr);
// this function converts an array to datetime value
void arrayToDatetimeWithSecond(datetime_t *dt, uint8_t *arr);
// This functon compares two datetime values and return an int value
int datetimeComp(datetime_t *dt1, datetime_t *dt2);
// This function copies a datetime value to another datetime value
void datetimeCopy(datetime_t *src, datetime_t *dst);
void getAllRecords(int32_t *st_idx, int32_t *end_idx, datetime_t *start, datetime_t *end, size_t offset, size_t size, uint16_t record_size, enum ListeningStates state);
void getSelectedRecords(int32_t *st_idx, int32_t *end_idx, datetime_t *start, datetime_t *end, datetime_t *dt_start, datetime_t *dt_end, uint8_t *reading_state_start_time, uint8_t *reading_state_end_time, size_t offset, size_t size, uint16_t record_size, enum ListeningStates state);
// This function searches the requested data in flash by starting from flash record beginning offset, collects data from flash and sends it to UART to show load profile content
void searchDataInFlash(uint8_t *buf, uint8_t *reading_state_start_time, uint8_t *reading_state_end_time, enum ListeningStates state);
// This function resets records and sector data and set sector data to 0 (UNUSUED FUNCTION)
void __not_in_flash_func(resetFlashSettings)();
void __not_in_flash_func(checkSectorContent)();
void __not_in_flash_func(checkThresholdContent)();
void __not_in_flash_func(updateThresholdSector)(uint16_t sector_val);
#if WITHOUT_BOOTLOADER
// This function adds serial number to flash area
void __not_in_flash_func(addSerialNumber)();
#endif
void __not_in_flash_func(setProgramStartDate)(datetime_t *ct);
void __not_in_flash_func(writeSuddenAmplitudeChangeRecordToFlash)(struct AmplitudeChangeTimerCallbackParameters *ac_params);

#endif
