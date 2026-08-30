#ifndef ADC_H
#define ADC_H

#include <stdint.h>
#include <stddef.h>

struct ThresholdData;

uint16_t calculateVariance(uint16_t *buffer, uint16_t size);
float calculateVRMS(uint16_t *buffer, size_t size, float bias_voltage);
float getMean(uint16_t *buffer, size_t size);
void calculateVRMSValuesPerSecond(float *vrms_buffer, uint16_t *sample_buf, size_t buffer_size, size_t sample_size_per_vrms_calc, float bias_voltage);
// Halkadaki bir sonraki yazma konumu (mutlak slot indeksi, 0..TH_RECORD_SLOT_COUNT-1)
uint16_t getThresholdWriteIndex(void);
void __not_in_flash_func(writeThresholdRecord)(const struct ThresholdData *record);
#if CONF_SUDDEN_AMPLITUDE_CHANGE_ENABLED
uint8_t detectSuddenAmplitudeChangeWithDerivative(float *sample_buf, size_t buffer_size);
#endif

#endif
