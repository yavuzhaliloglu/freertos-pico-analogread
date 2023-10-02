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

#endif
