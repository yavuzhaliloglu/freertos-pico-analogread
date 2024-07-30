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

uint16_t calculateVariance(double *buffer, size_t size)
{
    double total = 0;
    double mean;
    double variance_total = 0;

    for (size_t i = 0; i < size; i++)
    {
        total += buffer[i];
    }

    mean = total / size;

    for (size_t i = 0; i < size; i++)
    {
        double mult = buffer[i] - mean;
        variance_total += mult * mult;
    }

    PRINTF("\ntotal of samples is: %f\n", total);
    PRINTF("\nmean of samples is: %f\n", mean);
    PRINTF("\nvariance total of samples is: %f\n", variance_total);
    PRINTF("\nvariance of samples is: %f\n", (variance_total / (size - 1)));

    if (size <= 1)
    {
        PRINTF("variance of samples is 1 or 0\n");
        return 0;
    }
    else
    {
        PRINTF("variance of samples is bigger than 1\n");
        return (uint16_t)(variance_total / (size - 1));
    }
}

double calculateVRMS(double bias)
{
    // Initialize the variables for VRMS calculation
    double vrms = 0.0;
    double vrms_accumulator = 0.0;
    const float conversion_factor = 1000 * (3.3f / (1 << 12));
    // Get samples
    adcCapture(sample_buffer, VRMS_SAMPLE);

#if DEBUG
    char deneme[40] = {0};
#endif

    double mean = bias * conversion_factor / 1000;

#if DEBUG
    deneme[32] = '\0';
    snprintf(deneme, 31, "CALCULATEVRMS: mean: %f\n", mean);
    PRINTF("%s", deneme);
#endif

    for (uint16_t i = 0; i < VRMS_SAMPLE; i++)
    {
        double production = (double)(sample_buffer[i] * conversion_factor) / 1000;
        vrms_accumulator += pow((production - mean), 2);
    }

#if DEBUG
    snprintf(deneme, 36, "CALCULATEVRMS: vrmsAc: %f\n", vrms_accumulator);
    deneme[37] = '\0';
    PRINTF("%s", deneme);
#endif

    vrms = sqrt(vrms_accumulator / VRMS_SAMPLE);
    vrms = vrms * VRMS_MULTIPLICATION_VALUE;

    return vrms;
}

double getMean(uint16_t *buffer, size_t size)
{
    double total = 0;

    for (size_t i = 0; i < size; i++)
        total += (double)buffer[i];

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
void writeThresholdRecord(double vrms, uint16_t variance)
{
    PRINTF("writing threshold record\n");
    // initialize the variables
    struct ThresholdData data;
    uint8_t *flash_threshold_recs = (uint8_t *)(XIP_BASE + FLASH_THRESHOLD_OFFSET + (th_sector_data * FLASH_SECTOR_SIZE));
    uint8_t *flash_threshold_recs_start = (uint8_t *)(XIP_BASE + FLASH_THRESHOLD_OFFSET);
    uint8_t *flash_threshold_recs_end = (uint8_t *)(XIP_BASE + FLASH_THRESHOLD_OFFSET + (3 * FLASH_SECTOR_SIZE) + 14 * FLASH_PAGE_SIZE);
    uint16_t offset;

    memcpy(th_flash_buf, flash_threshold_recs, FLASH_SECTOR_SIZE);

    // set struct data parameters
    setDateToCharArray(current_time.year, data.year);
    setDateToCharArray(current_time.month, data.month);
    setDateToCharArray(current_time.day, data.day);
    setDateToCharArray(current_time.hour, data.hour);
    setDateToCharArray(current_time.min, data.min);
    data.vrms = vrms;
    data.variance = variance;
    data.padding[0] = 0x7F;
    data.padding[1] = 0x7F;

    // find the last offset of flash records and write current values to last offset of flash_data buffer
    for (offset = 0; offset < FLASH_SECTOR_SIZE; offset += FLASH_RECORD_SIZE)
    {
        if (flash_threshold_recs[offset] == 0x00 || flash_threshold_recs[offset] == 0xFF)
        {
            if (offset == 0)
            {
                PRINTF("SETFLASHDATA: last record is not found.\n");
            }
            else
            {
                PRINTF("SETFLASHDATA: last record is start in %d offset\n", offset - 16);
            }

            th_flash_buf[offset / FLASH_RECORD_SIZE] = data;

            PRINTF("SETFLASHDATA: record saved to offset: %d. used %d/%d of sector.\n", offset, offset + 16, FLASH_SECTOR_SIZE);

            break;
        }
    }

    // if offset value is equals or bigger than FLASH_SECTOR_SIZE, (4096 bytes) it means current sector is full and program should write new values to next sector
    if (offset >= FLASH_SECTOR_SIZE)
    {
        PRINTF("SETFLASHDATA: offset value is equals to sector size. Current sector data is: %d. Sector is changing...\n", th_sector_data);

        // if current sector is last sector of flash, sector data will be 0 and the program will start to write new records to beginning of the flash record offset
        if (th_sector_data == 3)
        {
            th_sector_data = 0;
        }
        else
        {
            th_sector_data++;
        }

        PRINTF("SETFLASHDATA: new sector value is: %d\n", th_sector_data);

        // reset variables and call setSectorData()
        memset(th_flash_buf, 0, FLASH_SECTOR_SIZE);
        th_flash_buf[0] = data;
        updateThresholdSector(th_sector_data);

        PRINTF("SETFLASHDATA: Sector changing written to flash.\n");
    }

    PRINTF("WRITETHRESHOLDDATA: flash_th_buf as hexadecimal: \n");
    printBufferHex((uint8_t *)th_flash_buf, FLASH_PAGE_SIZE);

    // write buffer in flash
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_THRESHOLD_OFFSET + (th_sector_data * FLASH_SECTOR_SIZE), FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_THRESHOLD_OFFSET + (th_sector_data * FLASH_SECTOR_SIZE), (uint8_t *)th_flash_buf, FLASH_SECTOR_SIZE);
    restore_interrupts(ints);

    PRINTF("WRITETHRESHOLDDATA: threshold records start area: \n");
    printBufferHex(flash_threshold_recs_start, FLASH_PAGE_SIZE);

    PRINTF("WRITETHRESHOLDDATA: threshold records end area: \n");
    printBufferHex(flash_threshold_recs_end, 3 * FLASH_PAGE_SIZE);
}

double getMeanVarianceVRMSValues(double *buffer, uint8_t size)
{
    double mean_vrms = 0;
    double total = 0;
    uint16_t variance = 0;

    // mean vrms calculation
    for (uint8_t i = 0; i < size; i++)
    {
        total += buffer[i];
    }

    if (size == 0)
    {
        mean_vrms = 0;
    }
    else
    {
        mean_vrms = total / size;
    }

    // if calculated vrms is bigger than threshold value, set a record to flash memory
    if (mean_vrms >= (double)vrms_threshold)
    {
        if (!threshold_set_before)
        {
            // put THRESHOLD PIN 1 value
            gpio_put(THRESHOLD_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            // set flag to not put 1 until command comes
            threshold_set_before = 1;
        }

        variance = calculateVariance(buffer, size);
        writeThresholdRecord(mean_vrms, variance);
    }

    PRINTF("GETMEANVARIANCEVRMSVALUES: calculated vrms value from vrms_values array is: %f\n", mean_vrms);

    return mean_vrms;
}
#endif
