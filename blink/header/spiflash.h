#ifndef SPIFLASH_H
#define SPIFLASH_H

#include "defines.h"
#include "variables.h"
#include "bcc.h"

void writeBlock(uint8_t *buffer, uint8_t size)
{
    uint8_t lsb_byte = buffer[size - 2];

    for (uint8_t i = 0; i < size - 2; i++)
    {
        buffer[i] = (buffer[i] << 1);
        uint8_t lsb = lsb_byte & 0x01;
        buffer[i] += lsb;
        lsb_byte = (lsb_byte >> 1);
    }

    memcpy(rpb + (rpb_len), buffer, size - 2);
    rpb_len += 7;

    if (rpb_len == FLASH_RPB_BLOCK_SIZE || reprogram_size == 0)
    {
        flash_range_program(FLASH_REPROGRAM_OFFSET + (ota_block_count * FLASH_RPB_BLOCK_SIZE), rpb, FLASH_RPB_BLOCK_SIZE);
        ota_block_count++;
        memset(rpb, 0, FLASH_RPB_BLOCK_SIZE);
        rpb_len = 0;
    }
}

bool writeProgramToFlash(uint8_t chr)
{
    data_pck[data_cnt++] = chr;
    reprogram_size--;

    if (data_cnt == 9 || reprogram_size == 0)
    {
        // DEBUG
        printf("Reprogram Size: %d\n", reprogram_size);

        uint8_t bcc_received = data_pck[data_cnt - 1];
        uint8_t bcc = bccCreate(data_pck, data_cnt - 1, 0x00) & 0x7F;

        if (bcc == bcc_received)
        {
            uart_putc(UART0_ID, 0x06);
            writeBlock(data_pck, data_cnt);
        }
        else
        {
            uart_putc(UART0_ID, 0x15);
        }

        data_cnt = 0;
        memset(data_pck, 0, 9);
    }

    if (reprogram_size == 0)
        return true;
    else
        return false;
}

#endif