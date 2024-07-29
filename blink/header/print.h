#ifndef PRINT_H
#define PRINT_H

// This function prints a 1 byte value as binary
void printBinaryHex(uint8_t value)
{
    PRINTF("Hexadecimal Value: %02X\t Binary Value: ", value);

    for (int i = 7; i >= 0; i--)
    {
        uint8_t binstr[2] = {0};
        snprintf((char *)binstr, 2, "%d", (value & (1 << i)) ? 1 : 0);
        PRINTF("%s", binstr);
    }
    PRINTF("\n");
}

// This function prints a buffer as hexadecimal values
void printBufferHex(uint8_t *buf, size_t len)
{
#if DEBUG
    for (size_t i = 0; i < len; ++i)
    {
        printf("%02X", buf[i]);

        if (i % 16 == 15)
        {
            printf("\n");
        }
        else
        {
            printf(" ");
        }

        if (i % 256 == 255)
        {
            printf("page %d\n\n", i / 256);
        }
    }
#else
    (void)buf;
    (void)len;
#endif
}

#endif
