#ifndef ADC_H
#define ADC_H

void __not_in_flash_func(adcCapture)(uint16_t *buf, size_t count)
{
    // Set ADC FIFO and get the samples with adc_run()
    adc_fifo_setup(true, false, 0, false, false);
    adc_run(true);

    // Get FIFO contents and copy them to buffer
    for (int i = 0; i < count; i = i + 1)
        buf[i] = adc_fifo_get_blocking();

    // End sampling and drain the FIFO
    adc_run(false);
    adc_fifo_drain();
}

uint32_t calculateVariance(uint16_t *buffer, size_t size)
{
    uint32_t total = 0;
    uint32_t mean;
    uint32_t variance_total = 0;

    for (int i = 0; i < size; i++)
    {
        total += buffer[i];
    }

    mean = total / size;

    for (int i = 0; i < size; i++)
    {
        uint32_t mult = buffer[i] - mean;
        variance_total += mult * mult;
    }

#if DEBUG
    printf("\ntotal of samples is: %ld\n", total);
    printf("\nmean of samples is: %ld\n", mean);
    printf("\nvariance total of samples is: %ld\n", variance_total);
    printf("\nvariance of samples is: %ld\n", (variance_total / (size - 1)));
#endif

    return variance_total / (size - 1);
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

    float mean = bias * conversion_factor / 1000;

#if DEBUG
    deneme[32] = '\0';
    snprintf(deneme, 31, "CALCULATEVRMS: mean: %f\n", mean);
    printf("%s", deneme);
#endif

    for (uint16_t i = 0; i < VRMS_SAMPLE; i++)
    {
        double production = (double)(sample_buffer[i] * conversion_factor) / 1000;
        vrms_accumulator += pow((production - mean), 2);
    }

#if DEBUG
    snprintf(deneme, 36, "CALCULATEVRMS: vrmsAc: %f\n", vrms_accumulator);
    deneme[37] = '\0';
    printf("%s", deneme);
#endif

    vrms = sqrt(vrms_accumulator / VRMS_SAMPLE);
    vrms = vrms * 150;

    return vrms;
}

double getMean(uint16_t *buffer, size_t size)
{
    double total = 0;

    for (size_t i = 0; i < size; i++)
        total += buffer[i];

    return (total / size);
}

void checkVRMSThreshold(double vrms)
{
    vrms = (uint16_t)vrms;
#if DEBUG
    printf("vrms value as uint16_t is: %d\n", vrms);
#endif
    if (vrms >= vrms_threshold)
    {
        int8_t day = current_time.day;
        int8_t month = current_time.month;
        int8_t year = current_time.year;
        int8_t hour = current_time.hour;
        int8_t min = current_time.min;
        uint8_t *threshold_recs = (uint8_t *)(XIP_BASE + FLASH_THRESHOLD_OFFSET);
        int l_offset;
        uint8_t sector_count = 0;

        uint8_t threshold_buffer[16] = {0};
        snprintf(threshold_buffer, 16, "%02d%02d%02d%02d%02d%03d", year, month, day, hour, min, (uint16_t)vrms);

#if DEBUG
        printf("threshold buffer as hexadecimal: \n");
        printBufferHex(threshold_buffer, 16);
#endif

        for (l_offset = 16; l_offset < 4 * FLASH_SECTOR_SIZE; l_offset += 16)
        {
            if (l_offset == FLASH_SECTOR_SIZE)
            {
                sector_count++;
            }

            if (threshold_recs[l_offset] == 0xFF || threshold_recs[l_offset] == 0x00)
            {
                break;
            }
        }

        if (sector_count == 4 * FLASH_SECTOR_SIZE)
        {
            sector_count = 0;
            l_offset = 16;
        }

        uint8_t *threshold_sector_recs = (uint8_t *)(XIP_BASE + FLASH_THRESHOLD_OFFSET + (sector_count * FLASH_SECTOR_SIZE));
        memcpy(flash_th_buf, threshold_sector_recs, l_offset - (sector_count * FLASH_SECTOR_SIZE));
        memcpy(flash_th_buf + l_offset - (sector_count * FLASH_SECTOR_SIZE), threshold_buffer, 16);

#if DEBUG
        printf("flash_th_buf as hexadecimal: \n");
        printBufferHex(flash_th_buf, FLASH_PAGE_SIZE);
#endif
        uint32_t ints = save_and_disable_interrupts();

        flash_range_erase(FLASH_THRESHOLD_OFFSET + (sector_count * FLASH_SECTOR_SIZE), FLASH_SECTOR_SIZE);
        flash_range_program(FLASH_THRESHOLD_OFFSET, flash_th_buf, FLASH_SECTOR_SIZE);

        restore_interrupts(ints);
    }
}
#endif
