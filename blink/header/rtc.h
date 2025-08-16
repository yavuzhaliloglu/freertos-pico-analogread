#ifndef RTC_H
#define RTC_H

#include <stdint.h>
#include "hardware/i2c.h"
#include "hardware/rtc.h"
#include "pico/util/datetime.h"

uint8_t initI2C();
// This function converts decimal value to BCD value
uint8_t decimalToBCD(uint8_t decimalValue);
// This function converts BCD value to decimal value
uint8_t bcd_to_decimal(uint8_t bcd);
// This function sets the PT7C4338's Real Time
uint8_t setTimePt7c4338(struct i2c_inst *i2c, uint8_t address, uint8_t seconds, uint8_t minutes, uint8_t hours, uint8_t day, uint8_t date, uint8_t month, uint8_t year);
// This function gets the PT7C4338's Real Time and sets it to datetime object
uint8_t getTimePt7c4338(datetime_t *dt);
#endif
