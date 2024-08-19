#ifndef ADC_H
#define ADC_H

void adcCapture(uint16_t *buf, size_t count)
{
    // Set ADC FIFO and get the samples with adc_run()
    adc_fifo_setup(true, false, 0, false, false);
    adc_run(true);

    // Get FIFO contents and copy them to buffer
    for (size_t i = 0; i < count; i++)
        buf[i] = adc_fifo_get_blocking();

    // End sampling and drain the FIFO
    adc_run(false);
    adc_fifo_drain();
}

uint16_t calculateVariance(uint16_t *buffer, uint16_t size)
{
    int total = 0;
    int mean;
    int variance_total = 0;

    for (uint16_t i = 0; i < size; i++)
    {
        total += buffer[i];
    }

    mean = total / size;

    for (uint16_t i = 0; i < size; i++)
    {
        int mult = buffer[i] - mean;
        variance_total += mult * mult;
    }

    // PRINTF("\ntotal of samples is: %d\n", total);
    // PRINTF("\nmean of samples is: %d\n", mean);
    // PRINTF("\nvariance total of samples is: %d\n", variance_total);
    // PRINTF("variance of samples is: %d\n", (variance_total / (size - 1)));

    return (uint16_t)(variance_total / (size - 1));
}

float calculateVRMS(uint16_t *buffer, size_t size, float bias_voltage)
{
    // Initialize the variables for VRMS calculation
    float vrms = 0.0;
    float vrms_accumulator = 0.0;
    const float conversion_factor = 1000 * (3.3f / (1 << 12));

    // #if DEBUG
    //     char deneme[40] = {0};
    // #endif

    // #if DEBUG
    //     deneme[32] = '\0';
    //     snprintf(deneme, 31, "CALCULATEVRMS: mean: %f\n", mean);
    //     PRINTF("%s", deneme);
    // #endif

    float mean = bias_voltage * conversion_factor / 1000;

    for (uint16_t i = 0; i < size; i++)
    {
        float production = (float)(buffer[i] * conversion_factor) / 1000;
        vrms_accumulator += (float)pow((production - mean), 2);
    }

    // #if DEBUG
    //     snprintf(deneme, 36, "CALCULATEVRMS: vrmsAc: %f\n", vrms_accumulator);
    //     deneme[37] = '\0';
    //     PRINTF("%s", deneme);
    // #endif

    vrms = sqrt(vrms_accumulator / size);
    vrms = vrms * VRMS_MULTIPLICATION_VALUE;

    return vrms;
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
    uint8_t *flash_threshold_recs = (uint8_t *)(XIP_BASE + FLASH_THRESHOLD_OFFSET + (th_sector_data * FLASH_SECTOR_SIZE));
    // uint8_t *flash_threshold_recs_start = (uint8_t *)(XIP_BASE + FLASH_THRESHOLD_OFFSET);
    // uint8_t *flash_threshold_recs_end = (uint8_t *)(XIP_BASE + FLASH_THRESHOLD_OFFSET + (3 * FLASH_SECTOR_SIZE) + 14 * FLASH_PAGE_SIZE);
    uint16_t offset = 0;

    if (xSemaphoreTake(xFlashMutex, portMAX_DELAY) == pdTRUE)
    {
        PRINTF("WRITETHRESHOLDRECORD: memcpy mutex received\r\n");
        memcpy(th_flash_buf, flash_threshold_recs, FLASH_SECTOR_SIZE);
        xSemaphoreGive(xFlashMutex);
    }
    else
    {
        PRINTF("WRITETHRESHOLDRECORD: memcpy mutex error\r\n");
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

    if (xSemaphoreTake(xFlashMutex, portMAX_DELAY) == pdTRUE)
    {
        PRINTF("WRITETHRESHOLDRECORD: offset loop mutex received\r\n");
        // find the last offset of flash records and write current values to last offset of flash_data buffer
        for (offset = 0; offset < FLASH_SECTOR_SIZE; offset += FLASH_RECORD_SIZE)
        {
            if (flash_threshold_recs[offset] == 0x00 || flash_threshold_recs[offset] == 0xFF)
            {
                if (offset == 0)
                {
                    PRINTF("SETFLASHDATA: last record is not found.\r\n");
                }
                else
                {
                    PRINTF("SETFLASHDATA: last record is start in %d offset\r\n", offset - 16);
                }

                th_flash_buf[offset / FLASH_RECORD_SIZE] = data;

                PRINTF("SETFLASHDATA: record saved to offset: %d. used %d/%d of sector.\r\n", offset, offset + 16, FLASH_SECTOR_SIZE);

                break;
            }
        }

        xSemaphoreGive(xFlashMutex);
    }
    else
    {
        PRINTF("WRITETHRESHOLDRECORD: offset loop mutex error\r\n");
    }

    // if offset value is equals or bigger than FLASH_SECTOR_SIZE, (4096 bytes) it means current sector is full and program should write new values to next sector
    if (offset >= FLASH_SECTOR_SIZE)
    {
        PRINTF("SETFLASHDATA: offset value is equals to sector size. Current sector data is: %d. Sector is changing...\r\n", th_sector_data);

        // if current sector is last sector of flash, sector data will be 0 and the program will start to write new records to beginning of the flash record offset
        if (th_sector_data == 3)
        {
            th_sector_data = 0;
        }
        else
        {
            th_sector_data++;
        }

        PRINTF("SETFLASHDATA: new sector value is: %d\r\n", th_sector_data);

        // reset variables and call setSectorData()
        memset(th_flash_buf, 0, FLASH_SECTOR_SIZE);
        th_flash_buf[0] = data;
        updateThresholdSector(th_sector_data);

        PRINTF("SETFLASHDATA: Sector changing written to flash.\r\n");
    }

    // PRINTF("WRITETHRESHOLDDATA: flash_th_buf as hexadecimal: \n");
    // printBufferHex((uint8_t *)th_flash_buf, FLASH_PAGE_SIZE);

    // write buffer in flash
    if (xSemaphoreTake(xFlashMutex, portMAX_DELAY) == pdTRUE)
    {
        PRINTF("WRITETHRESHOLDRECORD: write flash mutex received\r\n");
        flash_range_erase(FLASH_THRESHOLD_OFFSET + (th_sector_data * FLASH_SECTOR_SIZE), FLASH_SECTOR_SIZE);
        flash_range_program(FLASH_THRESHOLD_OFFSET + (th_sector_data * FLASH_SECTOR_SIZE), (uint8_t *)th_flash_buf, FLASH_SECTOR_SIZE);
        PRINTF("WRITETHRESHOLDDATA: threshold record written to flash.\r\n");
        xSemaphoreGive(xFlashMutex);
    }
    else
    {
        PRINTF("MUTEX CANNOT RECEIVED!\r\n");
    }
    // PRINTF("WRITETHRESHOLDDATA: threshold records start area: \n");
    // printBufferHex(flash_threshold_recs_start, FLASH_PAGE_SIZE);

    // PRINTF("WRITETHRESHOLDDATA: threshold records end area: \n");
    // printBufferHex(flash_threshold_recs_end, 3 * FLASH_PAGE_SIZE);
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

#endif
