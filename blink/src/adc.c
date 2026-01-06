
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/flash.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

#include "header/project_globals.h"
#include "header/print.h"
#include "header/spiflash.h"
#include "header/adc.h"
#include "header/bcc.h"

uint16_t calculateVariance(uint16_t *buffer, uint16_t size)
{
    uint32_t total = 0;
    uint32_t mean;
    uint32_t variance_total = 0;

    for (uint16_t i = 0; i < size; i++)
    {
        total += buffer[i];
    }

    mean = total / size;

    for (uint16_t i = 0; i < size; i++)
    {
        int32_t mult = buffer[i] - mean;
        variance_total += mult * mult;
    }

    return (uint16_t)(variance_total / (size - 1));
}

float calculateVRMS(uint16_t *buffer, size_t size, float bias_voltage)
{
    float total = 0.0;
    float vrms = 0.0;
    float conversion_factor = (3.28f / (1 << 12));

    bias_voltage = bias_voltage * conversion_factor;

    for (size_t i = 0; i < size; i++)
    {
        float adjusted_sample = (float)((buffer[i] * conversion_factor) - bias_voltage);
        total += adjusted_sample * adjusted_sample;
    }

    vrms = sqrt(total / size);
    return vrms * VRMS_MULTIPLICATION_VALUE;
}

float getMean(uint16_t *buffer, size_t size)
{
    float total = 0;

    for (size_t i = 0; i < size; i++)
        total += (float)buffer[i];

    if (size == 0)
    {
        return 0;
    }
    else
    {
        return (total / size);
    }
}

// Write threshold data to flash
void __not_in_flash_func(writeThresholdRecord)(float vrms, uint16_t variance)
{
    PRINTF("writing threshold record\r\n");

    // initialize the variables
    struct ThresholdData data;
    uint8_t *flash_threshold_recs = (uint8_t *)(XIP_BASE + FLASH_THRESHOLD_RECORDS_ADDR + (th_sector_data * FLASH_SECTOR_SIZE));
    uint16_t offset = 0;

    if (xSemaphoreTake(xFlashMutex, pdMS_TO_TICKS(250)) == pdTRUE)
    {
        PRINTF("WRITETHRESHOLDRECORD: memcpy mutex received\r\n");
        memcpy(th_flash_buf, flash_threshold_recs, FLASH_SECTOR_SIZE);
        xSemaphoreGive(xFlashMutex);
    }
    else
    {
        PRINTF("WRITETHRESHOLDRECORD: memcpy mutex error\r\n");
        led_blink_pattern(LED_ERROR_CODE_FLASH_MUTEX_NOT_TAKEN);
        return;
    }

    // set struct data parameters
    setDateToCharArray(current_time.year, data.year);
    setDateToCharArray(current_time.month, data.month);
    setDateToCharArray(current_time.day, data.day);
    setDateToCharArray(current_time.hour, data.hour);
    setDateToCharArray(current_time.min, data.min);
    setDateToCharArray(current_time.sec, data.sec);
    data.vrms = vrms;
    data.variance = variance;

    if (xSemaphoreTake(xFlashMutex, pdMS_TO_TICKS(250)) == pdTRUE)
    {
        PRINTF("WRITETHRESHOLDRECORD: offset loop mutex received\r\n");
        // find the last offset of flash records and write current values to last offset of flash_data buffer
        for (offset = 0; offset < FLASH_SECTOR_SIZE; offset += FLASH_RECORD_SIZE)
        {
            if (flash_threshold_recs[offset] == 0x00 || flash_threshold_recs[offset] == 0xFF)
            {
                if (offset == 0)
                {
                    PRINTF("WRITETHRESHOLDRECORD: last record is not found.\r\n");
                }
                else
                {
                    PRINTF("WRITETHRESHOLDRECORD: last record is start in %d offset\r\n", offset - 16);
                }

                th_flash_buf[offset / FLASH_RECORD_SIZE] = data;

                PRINTF("WRITETHRESHOLDRECORD: record saved to offset: %d. used %d/%d of sector.\r\n", offset, offset + 16, FLASH_SECTOR_SIZE);

                break;
            }
        }

        xSemaphoreGive(xFlashMutex);
    }
    else
    {
        PRINTF("WRITETHRESHOLDRECORD: offset loop mutex error\r\n");
        led_blink_pattern(LED_ERROR_CODE_FLASH_MUTEX_NOT_TAKEN);
        return;
    }

    // if offset value is equals or bigger than FLASH_SECTOR_SIZE, (4096 bytes) it means current sector is full and program should write new values to next sector
    if (offset >= FLASH_SECTOR_SIZE)
    {
        PRINTF("WRITETHRESHOLDRECORD: offset value is equals to sector size. Current sector data is: %d. Sector is changing...\r\n", th_sector_data);

        // if current sector is last sector of flash, sector data will be 0 and the program will start to write new records to beginning of the flash record offset
        if (th_sector_data == 3)
        {
            th_sector_data = 0;
        }
        else
        {
            th_sector_data++;
        }

        PRINTF("WRITETHRESHOLDRECORD: new sector value is: %d\r\n", th_sector_data);

        // reset variables and call setSectorData()
        memset(th_flash_buf, 0, FLASH_SECTOR_SIZE);
        th_flash_buf[0] = data;
        updateThresholdSector(th_sector_data);

        PRINTF("WRITETHRESHOLDRECORD: Sector changing written to flash.\r\n");
    }

    // write buffer in flash
    if (xSemaphoreTake(xFlashMutex, pdMS_TO_TICKS(250)) == pdTRUE)
    {
        PRINTF("WRITETHRESHOLDRECORD: write flash mutex received\r\n");
        uint32_t ints = save_and_disable_interrupts();
        flash_range_erase(FLASH_THRESHOLD_RECORDS_ADDR + (th_sector_data * FLASH_SECTOR_SIZE), FLASH_SECTOR_SIZE);
        flash_range_program(FLASH_THRESHOLD_RECORDS_ADDR + (th_sector_data * FLASH_SECTOR_SIZE), (uint8_t *)th_flash_buf, FLASH_SECTOR_SIZE);
        restore_interrupts(ints);
        PRINTF("WRITETHRESHOLDDATA: threshold record written to flash.\r\n");
        xSemaphoreGive(xFlashMutex);
    }
    else
    {
        PRINTF("MUTEX CANNOT RECEIVED!\r\n");
        led_blink_pattern(LED_ERROR_CODE_FLASH_MUTEX_NOT_TAKEN);
        return;
    }
}

uint8_t detectSuddenAmplitudeChangeWithDerivative(float *sample_buf, size_t buffer_size)
{
    for (uint16_t i = 1; i < buffer_size; i++)
    {
        float derivative = sample_buf[i] - sample_buf[i - 1];

        if (fabs(derivative) > AMPLITUDE_THRESHOLD)
        {
            PRINTF("Sudden amplitude change detected at index %d: %f\r\n", i, fabs(derivative));
            return 1;
        }
    }

    return 0;
}

void calculateVRMSValuesPerSecond(float *vrms_buffer, uint16_t *sample_buf, size_t buffer_size, size_t sample_size_per_vrms_calc, float bias_voltage)
{
    for (uint16_t i = 0; i < buffer_size; i += sample_size_per_vrms_calc)
    {
        float vrms = calculateVRMS(sample_buf + i, sample_size_per_vrms_calc, bias_voltage);
        vrms_buffer[i / sample_size_per_vrms_calc] = vrms;
    }

    PRINTF("VRMS VALUES PER SECOND:");
    printBufferFloat(vrms_buffer, buffer_size / sample_size_per_vrms_calc);
}

void setAmplitudeChangeParameters(struct AmplitudeChangeTimerCallbackParameters *ac_data, float *vrms_values_buffer, uint16_t variance, size_t adc_fifo_size, size_t vrms_values_buffer_size_bytes)
{
    memcpy(ac_data->vrms_values_buffer, vrms_values_buffer, vrms_values_buffer_size_bytes);
    ac_data->vrms_values_buffer_size_bytes = vrms_values_buffer_size_bytes;
    ac_data->variance = variance;
    ac_data->adc_fifo_size = adc_fifo_size;
}
