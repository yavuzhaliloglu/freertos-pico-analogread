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

        uint8_t threshold_buffer[32];
        snprintf(threshold_buffer, 31, "%cT.R.1(%02d-%02d-%02d,%02d:%02d)(%03d)\r\n%c", 0x02, year, month, day, hour, min, (uint16_t)vrms, 0x03);
        uint8_t bcc = bccGenerate(threshold_buffer, 30, 0x02);
        threshold_buffer[30] = bcc;
        threshold_buffer[31] = '\0';
#if DEBUG
        printf("threshold buffer as hexadecimal: \n");
        printBufferHex(threshold_buffer, 32);
#endif
        uint8_t flash_th_buf[FLASH_PAGE_SIZE] = {0};
        concatenateAndPrintHex(vrms_threshold, threshold_buffer, 32, flash_th_buf);
#if DEBUG
        printf("flash_th_buf as hexadecimal: \n");
        printBufferHex(flash_th_buf, FLASH_PAGE_SIZE);
#endif
        uint32_t ints = save_and_disable_interrupts();

        flash_range_erase(FLASH_THRESHOLD_OFFSET, FLASH_SECTOR_SIZE);
        flash_range_program(FLASH_THRESHOLD_OFFSET, flash_th_buf, FLASH_PAGE_SIZE);

        restore_interrupts(ints);
    }
}
#endif
