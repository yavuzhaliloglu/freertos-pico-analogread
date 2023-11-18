#ifndef BCC_H
#define BCC_H

// Generate BCC for a buffer.
uint8_t bccGenerate(uint8_t *data_buffer, uint8_t size, uint8_t xor)
{
    for (uint8_t i = 0; i < size; i++)
        xor ^= data_buffer[i];

#if DEBUG
    printf("BCCGENERATE: returned xor result is: %02X\n", xor);
#endif
    return xor;
}

// Generate a BCC for a buffer and control.
bool bccControl(uint8_t *buffer, uint8_t size)
{
    uint8_t xor_result = 0x01;

    for (uint8_t i = 0; i < size - 1; i++)
        xor_result ^= buffer[i];

#if DEBUG
    printf("BCCCONTROL: generated xor result is: %02X\n", xor_result);
    printf("BCCCONTROL: xor result in coming message is: %02X\n", buffer[size - 1]);
#endif
    return xor_result == buffer[size - 1];
}

// Generate a BCC and add it to end of a buffer.n
void setBCC(uint8_t *buffer, uint8_t size, uint8_t xor)
{
    for (int i = 0; i < size; i++)
        xor ^= buffer[i];

#if DEBUG
    printf("SETBCC: generated xor result is: %02X\n", xor);
#endif
    buffer[size] = xor;
}

#endif
