#ifndef BCC_H
#define BCC_H

#include "defines.h"
#include "variables.h"

// Generate BCC for a buffer.
uint8_t bccCreate(uint8_t *data_buffer, uint8_t size, uint8_t xor)
{
    for (uint8_t i = 0; i < size; i++)
        xor ^= data_buffer[i];

    return xor;
}

// Generate a BCC for a buffer and control.
bool bccControl(uint8_t *buffer, uint8_t size)
{
    uint8_t xor_result = 0x01;

    for (uint8_t i = 0; i < size; i++)
        xor_result ^= buffer[i];

    return xor_result == buffer[size - 1];
}

// Generate a BCC and add it to end of a buffer.n
void setBCC(uint8_t *buffer, uint8_t size, uint8_t xor)
{
    for (int i = 0; i < size; i++)
        xor ^= buffer[i];

    buffer[size - 1] = xor;
}

#endif
