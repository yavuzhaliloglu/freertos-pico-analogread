#ifndef MUTEX_H
#define MUTEX_H

#include <stdint.h>

uint8_t setMutexes();
uint16_t getVRMSThresholdValue();
void setVRMSThresholdValue(uint16_t value);
#if CONF_THRESHOLD_PIN_ENABLED
uint8_t getThresholdSetBeforeFlag();
void setThresholdSetBeforeFlag(uint8_t value);
#endif

#endif // MUTEX_H