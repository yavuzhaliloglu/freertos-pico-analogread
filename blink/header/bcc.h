#ifndef BCC_H
#define BCC_H

#include "defines.h"
#include "variables.h"

uint8_t bccCreate(uint8_t *data_buffer, uint8_t size, uint8_t xor)
{
    for (uint8_t i = 0; i < size; i++)
        xor ^= data_buffer[i];

    return xor;
}

bool bccControl(uint8_t *buffer, uint8_t size)
{
    uint8_t xor_result = 0x01;

    for (uint8_t i = 0; i < size; i++)
        xor_result ^= buffer[i];

    return xor_result == buffer[size - 1];
}

void setBCC(uint8_t *buffer, uint8_t size, uint8_t xor)
{
    for (int i = 0; i < size; i++)
        xor ^= buffer[i];

    printf("xor result in function is: %02X", xor);

    buffer[size - 1] = xor;
}


#endif
