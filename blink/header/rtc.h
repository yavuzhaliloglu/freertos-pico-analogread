#ifndef RTC_H
#define RTC_H

#include "defines.h"
#include "variables.h"

uint8_t decimalToBCD(uint8_t decimalValue)
{
    return ((decimalValue / 10) << 4) | (decimalValue % 10);
}

uint8_t bcd_to_decimal(uint8_t bcd)
{
    return bcd - 6 * (bcd >> 4);
}

void setTimePt7c4338(struct i2c_inst *i2c, uint8_t address, uint8_t seconds, uint8_t minutes, uint8_t hours, uint8_t day, uint8_t date, uint8_t month, uint8_t year)
{
    uint8_t buf[8];
    buf[0] = PT7C4338_REG_SECONDS;
    buf[1] = decimalToBCD(seconds);
    buf[2] = decimalToBCD(minutes);
    buf[3] = decimalToBCD(hours);
    buf[4] = decimalToBCD(day);
    buf[5] = decimalToBCD(date);
    buf[6] = decimalToBCD(month);
    buf[7] = decimalToBCD(year);

    i2c_write_blocking(i2c, address, buf, 8, false);
}

datetime_t getTimePt7c4338(datetime_t *dt)
{
    uint8_t buffer[7] = {PT7C4338_REG_SECONDS};

    i2c_write_blocking(I2C_PORT, I2C_ADDRESS, buffer, 1, 1);
    i2c_read_blocking(I2C_PORT, I2C_ADDRESS, buffer, 7, 0);

    dt->year = bcd_to_decimal(buffer[6]);
    dt->month = bcd_to_decimal(buffer[5]);
    dt->day = bcd_to_decimal(buffer[4]);
    dt->dotw = buffer[3];
    dt->hour = bcd_to_decimal(buffer[2]);
    dt->min = bcd_to_decimal(buffer[1]);
    dt->sec = bcd_to_decimal(buffer[0]);
}

#endif
