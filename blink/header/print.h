#ifndef PRINT_H
#define PRINT_H

// This function prints a 1 byte value as binary
void printBinaryHex(uint8_t value)
{
    printf("Hexadecimal Value: %02X\t Binary Value: ", value);

    for (int i = 7; i >= 0; i--)
    {
        uint8_t binstr[2] = {0};
        snprintf(binstr, 2, "%d", (value & (1 << i)) ? 1 : 0);
        printf("%s", binstr);
    }
    printf("\n");
}

// This function prints a buffer as hexadecimal values
void printBufferHex(uint8_t *buf, size_t len)
{
    for (size_t i = 0; i < len; ++i)
    {
        printf("%02X", buf[i]);
        if (i % 16 == 15)
            printf("\n");
        else
            printf(" ");

        if (i % 256 == 0 && i != 0)
        {
            printf("\n\n");
            printf("page %d\n", i / 256);
        }
    }
}

#endif