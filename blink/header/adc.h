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
    PRINTF("\nvariance of samples as uint32_t is: %ld\n", (uint32_t)(variance_total / (size - 1)));

    return (uint16_t)(variance_total / (size - 1));
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

    return (total / size);
}

void writeThresholdRecord(double vrms, uint16_t variance)
{
    // set datetime variables
    struct ThresholdData data;
    uint8_t *threshold_recs = (uint8_t *)(XIP_BASE + (FLASH_THRESHOLD_OFFSET + (th_sector_data * FLASH_SECTOR_SIZE)));
    unsigned int l_offset;
    uint8_t record_count = 0;

    // set struct data parameters
    setDateToCharArray(current_time.year, data.year);
    setDateToCharArray(current_time.month, data.month);
    setDateToCharArray(current_time.day, data.day);
    setDateToCharArray(current_time.hour, data.hour);
    setDateToCharArray(current_time.min, data.min);
    data.vrms = vrms;
    data.variance = variance;
    data.padding[0] = 0x00;
    data.padding[1] = 0x00;

    // // format the variables to create a record
    // snprintf(threshold_buffer, 17, "%02d%02d%02d%02d%02d%03d%03d", year, month, day, hour, min, (uint16_t)vrms, variance);

    // find last record offset of the sector
    for (l_offset = 0; l_offset < FLASH_SECTOR_SIZE; l_offset += 16)
    {
        if (threshold_recs[l_offset] == 0xFF || threshold_recs[l_offset] == 0x00)
        {
            break;
        }
        record_count++;
    }

    // if offset is equal to flash sector size, it means sector is full, so update sector and write data to the next sector
    if (l_offset == FLASH_SECTOR_SIZE)
    {
        if (th_sector_data == 4)
        {
            th_sector_data = 0;
            updateThresholdSector(th_sector_data);
        }
        else
        {
            th_sector_data++;
            updateThresholdSector(th_sector_data);
        }
    }

    // if l_offset is equals to flash sector size and sector data is 4, it means the area for threshold records is full, so clean the first sector and write record there
    if (l_offset == FLASH_SECTOR_SIZE && th_sector_data == 4)
    {
        l_offset = 0;
    }

    // copy the records in a sector in the buffer
    memcpy(flash_th_buf, threshold_recs, l_offset);
    // copy next record after the buffer's last record
    memcpy((flash_th_buf + record_count), (void *)&data, 16);

    PRINTF("WRITETHRESHOLDDATA: flash_th_buf as hexadecimal: \n");
    printBufferHex((uint8_t *)flash_th_buf, FLASH_PAGE_SIZE);

    // write buffer in flash
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_THRESHOLD_OFFSET + (th_sector_data * FLASH_SECTOR_SIZE), FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_THRESHOLD_OFFSET + (th_sector_data * FLASH_SECTOR_SIZE), (uint8_t *)flash_th_buf, FLASH_SECTOR_SIZE);
    restore_interrupts(ints);

    PRINTF("WRITETHRESHOLDDATA: threshold records as hexadecimal: \n");
    printBufferHex(threshold_recs, FLASH_PAGE_SIZE);
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

    mean_vrms = total / size;

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
