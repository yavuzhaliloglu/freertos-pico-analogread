#ifndef SPIFLASH_H
#define SPIFLASH_H

#include "defines.h"
#include "variables.h"
#include "bcc.h"

void printBinary(uint8_t value)
{
    uint8_t debug2[4] = {0};
    snprintf(debug2, 4, "%02X ", value);
    printf("%s", debug2);

    for (int i = 7; i >= 0; i--)
    {
        uint8_t debug[2] = {0};
        snprintf(debug, 2, "%d", (value & (1 << i)) ? 1 : 0);
        printf("%s", debug);
    }
    printf("\n");
}

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

void convertTo8Bit(uint8_t *buffer, uint8_t len)
{
    uint8_t lsb_byte = buffer[len - 1];

    for (uint8_t i = 0; i < len - 1; i++)
    {
        buffer[i] = (buffer[i] << 1);
        uint8_t lsb = lsb_byte & 0x01;
        buffer[i] += lsb;
        lsb_byte = (lsb_byte >> 1);
    }
}

void writeBlock(uint8_t *buffer, uint8_t size)
{
    convertTo8Bit(buffer, size);

    if (data_cnt != 0)
        memcpy(rpb + (rpb_len), buffer, size - 1);

    rpb_len += 7;

    if (rpb_len == FLASH_RPB_BLOCK_SIZE || is_program_end)
    {
        printf("rpb len is equal to block size. Programming...\n");
        flash_range_program(FLASH_REPROGRAM_OFFSET + (ota_block_count * FLASH_RPB_BLOCK_SIZE), rpb, FLASH_RPB_BLOCK_SIZE);
        ota_block_count++;
        memset(rpb, 0, FLASH_RPB_BLOCK_SIZE);
        rpb_len = 0;
    }
}

void writeProgramToFlash(uint8_t chr)
{
    if (!is_program_end)
        data_pck[data_cnt++] = chr;

    if (data_cnt == 8 || is_program_end)
    {
        writeBlock(data_pck, data_cnt);

        memset(data_pck, 0, 8);
        data_cnt = 0;
    }
}

void __not_in_flash_func(getFlashContents)()
{
    uint32_t ints = save_and_disable_interrupts();
    sector_data = *(uint8_t *)flash_sector_content;
    printf("sector data is: %d\n", sector_data);
    uint8_t *flash_target_contents = (uint8_t *)(XIP_BASE + FLASH_DATA_OFFSET + (sector_data * FLASH_SECTOR_SIZE));
    memcpy(flash_data, flash_target_contents, FLASH_SECTOR_SIZE);
    restore_interrupts(ints);
}

void __not_in_flash_func(setSectorData)()
{
    uint8_t sector_buffer[FLASH_PAGE_SIZE / sizeof(uint8_t)] = {0};
    sector_buffer[0] = sector_data;

    flash_range_erase(FLASH_SECTOR_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_SECTOR_OFFSET, (uint8_t *)sector_buffer, FLASH_SECTOR_SIZE);
}

void setDateToCharArray(int value, char *array)
{
    if (value < 10)
    {
        array[0] = '0';
        array[1] = value + '0';
    }
    else
    {
        array[0] = value / 10 + '0';
        array[1] = value % 10 + '0';
    }
}

void vrmsSetMinMaxMean(uint8_t *buffer, uint8_t size)
{
    uint8_t buffer_max = buffer[0];
    uint8_t buffer_min = buffer[0];
    uint16_t buffer_sum = buffer[0];
    uint8_t buffer_size = size;

    for (uint8_t i = 1; i < size; i++)
    {
        if (buffer[i] > buffer_max)
            buffer_max = buffer[i];

        if (buffer[i] < buffer_min)
            buffer_min = buffer[i];

        buffer_sum += buffer[i];
    }

    vrms_max = buffer_max;
    vrms_min = buffer_min;
    vrms_mean = (uint8_t)(buffer_sum / buffer_size);
}

void setFlashData()
{
    struct FlashData data;
    uint8_t *flash_target_contents = (uint8_t *)(XIP_BASE + FLASH_DATA_OFFSET + (sector_data * FLASH_SECTOR_SIZE));
    uint16_t offset;

    setDateToCharArray(current_time.year, data.year);
    setDateToCharArray(current_time.month, data.month);
    setDateToCharArray(current_time.day, data.day);
    setDateToCharArray(current_time.hour, data.hour);
    setDateToCharArray(current_time.min, data.min);
    setDateToCharArray(current_time.sec, data.sec);
    data.max_volt = vrms_max;
    data.min_volt = vrms_min;
    data.mean_volt = vrms_mean;
    data.eod_character = 0x04;

    vrms_max = 0;
    vrms_min = 0;
    vrms_mean = 0;

    // TO-DO:
    // flasha yazma işleminde yazamadan önce silinip güç kesildiğinde tüm bitler ff kalıyordu ve yazma işlemi yapmıyordu.
    // şuan yazma işlemi yapıyor fakat sektörü silip en başından yapıyor ve veri kaybına sebep oluyor.

    for (offset = 0; offset < FLASH_SECTOR_SIZE; offset += FLASH_RECORD_SIZE)
    {
        if (flash_target_contents[offset] == '\0' || flash_target_contents[offset] == 0xff)
        {
            flash_data[offset / FLASH_RECORD_SIZE] = data;
            break;
        }
    }

    if (offset >= FLASH_SECTOR_SIZE)
    {
        if (sector_data == FLASH_TOTAL_SECTORS)
            sector_data = 0;
        else
            sector_data++;

        memset(flash_data, 0, FLASH_SECTOR_SIZE);
        flash_data[0] = data;
        setSectorData();
    }
}

void __not_in_flash_func(SPIWriteToFlash)()
{
    setFlashData();
    // setSectorData();

    flash_range_erase(FLASH_DATA_OFFSET + (sector_data * FLASH_SECTOR_SIZE), FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_DATA_OFFSET + (sector_data * FLASH_SECTOR_SIZE), (uint8_t *)flash_data, FLASH_SECTOR_SIZE);
}

void arrayToDatetime(datetime_t *dt, uint8_t *arr)
{
    dt->year = (arr[0] - '0') * 10 + (arr[1] - '0');
    dt->month = (arr[2] - '0') * 10 + (arr[3] - '0');
    dt->day = (arr[4] - '0') * 10 + (arr[5] - '0');
    dt->hour = (arr[6] - '0') * 10 + (arr[7] - '0');
    dt->min = (arr[8] - '0') * 10 + (arr[9] - '0');
}

int datetimeComp(datetime_t *dt1, datetime_t *dt2)
{
    if (dt1->year - dt2->year != 0)
        return dt1->year - dt2->year;

    else if (dt1->month - dt2->month != 0)
        return dt1->month - dt2->month;

    else if (dt1->day - dt2->day != 0)
        return dt1->day - dt2->day;

    else if (dt1->hour - dt2->hour != 0)
        return dt1->hour - dt2->hour;

    else if (dt1->min - dt2->min != 0)
        return dt1->min - dt2->min;

    else if (dt1->sec - dt2->sec != 0)
        return dt1->sec - dt2->sec;

    return 0;
}

void datetimeCopy(datetime_t *src, datetime_t *dst)
{
    dst->year = src->year;
    dst->month = src->month;
    dst->day = src->day;
    dst->dotw = src->dotw;
    dst->hour = src->hour;
    dst->min = src->min;
    dst->sec = src->sec;
}

void searchDataInFlash()
{
    datetime_t start = {0};
    datetime_t end = {0};
    datetime_t dt_start = {0};
    datetime_t dt_end = {0};
    int32_t start_index = -1;
    int32_t end_index = -1;
    char load_profile_line[36] = {0};
    uint8_t *flash_start_content = (uint8_t *)(XIP_BASE + FLASH_DATA_OFFSET);

#if DEBUG
    printf("rx buffer len: %d\n", rx_buffer_len);
#endif

    if (rx_buffer_len == 14)
    {
        for (int i = 0; i < FLASH_TOTAL_RECORDS; i += 16)
        {
            if ((start_index != -1) && (flash_start_content[i] == 0xFF || flash_start_content[i] == 0x00))
            {
#if DEBUG
                printf("end index entered.\n");
#endif
                arrayToDatetime(&end, &flash_start_content[i - 16]);
                end_index = i - 16;
                break;
            }

            if (flash_start_content[i] == 0xFF || flash_start_content[i] == 0x00)
            {
#if DEBUG
                printf("empty rec entered.\n");
#endif
                continue;
            }

            if (start_index == -1)
            {
#if DEBUG
                printf("start index entered.\n");
#endif
                arrayToDatetime(&start, &flash_start_content[i]);
                start_index = i;
            }
        }
    }
    else
    {
        arrayToDatetime(&start, reading_state_start_time);
        arrayToDatetime(&end, reading_state_end_time);

        if (datetimeComp(&start, &end) > 0)
            return;

        for (uint32_t i = 0; i < FLASH_TOTAL_RECORDS; i += 16)
        {
            datetime_t recurrent_time = {0};

            if (flash_start_content[i] == 0xFF || flash_start_content[i] == 0x00)
                continue;

            arrayToDatetime(&recurrent_time, &flash_start_content[i]);

            if (datetimeComp(&recurrent_time, &start) >= 0)
            {
                if (start_index == -1 || (datetimeComp(&recurrent_time, &dt_start) < 0))
                {
                    start_index = i;
                    datetimeCopy(&recurrent_time, &dt_start);
                }
            }

            if (datetimeComp(&recurrent_time, &end) <= 0)
            {
                if (end_index == -1 || datetimeComp(&recurrent_time, &dt_end) > 0)
                {
                    end_index = i;
                    datetimeCopy(&recurrent_time, &dt_end);
                }
            }
        }
    }

    if (start_index >= 0 && end_index >= 0)
    {
        uint8_t xor_result = 0x02;
        uint32_t start_addr = start_index;
        uint8_t first_flag = 0;
        uint32_t end_addr = start_index <= end_index ? end_index : 1572864;

        for (; start_addr <= end_addr;)
        {
            char year[3] = {flash_start_content[start_addr], flash_start_content[start_addr + 1], 0x00};
            char month[3] = {flash_start_content[start_addr + 2], flash_start_content[start_addr + 3], 0x00};
            char day[3] = {flash_start_content[start_addr + 4], flash_start_content[start_addr + 5], 0x00};
            char hour[3] = {flash_start_content[start_addr + 6], flash_start_content[start_addr + 7], 0x00};
            char minute[3] = {flash_start_content[start_addr + 8], flash_start_content[start_addr + 9], 0x00};
            uint8_t min = flash_start_content[start_addr + 13];
            uint8_t max = flash_start_content[start_addr + 12];
            uint8_t mean = flash_start_content[start_addr + 14];

            if (start_addr == end_addr)
            {
                if (!first_flag)
                {
                    snprintf(load_profile_line, 35, "%c(%s-%s-%s,%s:%s)(%03d,%03d,%03d)\r\n\r%c", 0x02, year, month, day, hour, minute, min, max, mean, 0x03);
                    first_flag = 1;
                    xor_result = bccCreate(load_profile_line, 36, xor_result);
                    load_profile_line[34] = xor_result;
#if DEBUG
                    for (int i = 0; i < 32; i++)
                    {
                        printf("%02X ", load_profile_line[i]);
                    }
                    printf("\n");
#endif
                }
                else
                {
                    snprintf(load_profile_line, 34, "(%s-%s-%s,%s:%s)(%03d,%03d,%03d)\r\n\r%c", year, month, day, hour, minute, min, max, mean, 0x03);
                    xor_result = bccCreate(load_profile_line, 36, xor_result);
                    load_profile_line[33] = xor_result;
                }
            }
            else
            {
                if (!first_flag)
                {
                    snprintf(load_profile_line, 33, "%c(%s-%s-%s,%s:%s)(%03d,%03d,%03d)\r\n", 0x02, year, month, day, hour, minute, min, max, mean);
                    xor_result = bccCreate(load_profile_line, 33, xor_result);
                    first_flag = 1;
                }
                else
                {
                    snprintf(load_profile_line, 32, "(%s-%s-%s,%s:%s)(%03d,%03d,%03d)\r\n", year, month, day, hour, minute, min, max, mean);
                    xor_result = bccCreate(load_profile_line, 32, xor_result);
                }
            }
#if DEBUG
            printf("data sent\n");
#endif
            uart_puts(UART0_ID, load_profile_line);

            sleep_ms(15);

            if (start_index > end_index && start_addr == 1572848)
            {
                start_addr = 0;
                end_addr = end_index;
            }
            else
                start_addr += 16;
        }
    }
    else
    {
#if DEBUG
        printf("data not found\n");
#endif
        uart_putc(UART0_ID, 0x15);
    }
    memset(reading_state_start_time, 0, 10);
    memset(reading_state_end_time, 0, 10);
}

void resetFlashSettings()
{
    uint8_t reset_flash[256] = {0};

    flash_range_erase(FLASH_SECTOR_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_SECTOR_OFFSET, (uint8_t *)reset_flash, FLASH_PAGE_SIZE);

    flash_range_erase(FLASH_DATA_OFFSET, 1024 * 1024);
}

#endif
