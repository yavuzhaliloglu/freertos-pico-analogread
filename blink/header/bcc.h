#ifndef BCC_H
#define BCC_H

// Generate BCC for a buffer.
uint8_t bccGenerate(uint8_t *data_buffer, uint8_t size, uint8_t xor)
{
    for (uint8_t i = 0; i < size; i++)
        xor ^= data_buffer[i];

    PRINTF("BCCGENERATE: returned xor result is: %02X\n", xor);

    return xor;
}

// Generate a BCC for a buffer and control.
bool bccControl(uint8_t *buffer, uint8_t size)
{
    uint8_t xor_result = 0x01;

    for (uint8_t i = 0; i < size - 1; i++)
        xor_result ^= buffer[i];

    PRINTF("BCCCONTROL: generated xor result is: %02X\n", xor_result);
    PRINTF("BCCCONTROL: xor result in coming message is: %02X\n", buffer[size - 1]);

    return xor_result == buffer[size - 1];
}

// Generate a BCC and add it to end of a buffer.n
void setBCC(uint8_t *buffer, uint8_t size, uint8_t xor)
{
    for (int i = 0; i < size; i++)
        xor ^= buffer[i];

    PRINTF("SETBCC: generated xor result is: %02X\n", xor);

    buffer[size] = xor;
}

#endif
