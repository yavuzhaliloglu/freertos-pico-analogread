#ifndef ADC_H
#define ADC_H

#include <stdint.h>
#include <stddef.h>

void adcCapture(uint16_t *buf, size_t count);
uint16_t calculateVariance(uint16_t *buffer, uint16_t size);
float calculateVRMS(uint16_t *buffer, size_t size, float bias_voltage);
float getMean(uint16_t *buffer, size_t size);
// Write threshold data to flash
void __not_in_flash_func(writeThresholdRecord)(float vrms, uint16_t variance);
uint8_t detectSuddenAmplitudeChangeWithDerivative(float *sample_buf, size_t buffer_size);
void calculateVRMSValuesPerSecond(float *vrms_buffer, uint16_t *sample_buf, size_t buffer_size, size_t sample_size_per_vrms_calc, float bias_voltage);
// void setAmplitudeChangeParameters(struct AmplitudeChangeTimerCallbackParameters *ac_data, float *vrms_values_buffer, uint16_t variance, size_t adc_fifo_size, size_t vrms_values_buffer_size_bytes);
uint16_t getMeanOfSamples(uint16_t *buffer, uint16_t size);
float detectFrequency(uint16_t *adc_samples_buffer, float bias_voltage, size_t adc_samples_size);

#endif
