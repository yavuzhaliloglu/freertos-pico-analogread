#ifndef ADC_H
#define ADC_H

// ADC FUNCTIONS
void __not_in_flash_func(adcCapture)(uint16_t *buf, size_t count)
{
    adc_fifo_setup(true, false, 0, false, false);
    adc_run(true);

    for (int i = 0; i < count; i = i + 1)
        buf[i] = adc_fifo_get_blocking();

    adc_run(false);
    adc_fifo_drain();
}

double calculateVRMS(double bias)
{
    double vrms = 0.0;
    double vrms_accumulator = 0.0;
    const float conversion_factor = 1000 * (3.3f / (1 << 12));

    adcCapture(sample_buffer, VRMS_SAMPLE);

#if DEBUG
    char deneme[40] = {0};
    for (uint8_t i = 0; i < 150; i++)
    {
        snprintf(deneme, 20, "sample: %d\n", sample_buffer[i]);
        deneme[21] = '\0';

        printf("%s", deneme);
        vTaskDelay(1);
    }
    printf("\n");
#endif

    float mean = bias * conversion_factor / 1000;

#if DEBUG
    snprintf(deneme, 30, "mean: %f\n", mean);
    deneme[31] = '\0';
    printf("%s", deneme);
#endif

    for (uint16_t i = 0; i < VRMS_SAMPLE; i++)
    {
        double production = (double)(sample_buffer[i] * conversion_factor) / 1000;
        vrms_accumulator += pow((production - mean), 2);
    }

#if DEBUG
    snprintf(deneme, 34, "vrmsAc: %f\n", vrms_accumulator);
    deneme[35] = '\0';
    printf("%s", deneme);
#endif
    vrms = sqrt(vrms_accumulator / VRMS_SAMPLE);
    vrms = vrms * 75;
#if DEBUG
    printf("vrms: %d\n", (uint8_t)vrms);
#endif

    return vrms;
}

double getMean(uint16_t *buffer, size_t size)
{
    double total = 0;

    for (size_t i = 0; i < size; i++)
        total += buffer[i];

    printf("total: %lf\n", total);
    return (total / size);
}

#endif
