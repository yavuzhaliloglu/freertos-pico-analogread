#ifndef SPIFLASH_H
#define SPIFLASH_H

// this functions gets a buffer and adds last element's bits to rest of buffer's elements
void convertTo8Bit(uint8_t *buffer, uint8_t len)
{
    // get last element of the buffer
    uint8_t lsb_byte = buffer[len - 1];

    for (uint8_t i = 0; i < len - 1; i++)
    {
        // shift buffer's element to left
        buffer[i] = (buffer[i] << 1);
        // get the last element's LSB
        uint8_t lsb = lsb_byte & 0x01;
        // add the LSB to buffer element
        buffer[i] += lsb;
        // shift the last element to right to get next bit to add next buffer element
        lsb_byte = (lsb_byte >> 1);
    }
}

// this function writes 7-bit values rpb buffer and if buffer size is equals to block of rpb data size, (7 * 256 bytes) the buffer is written to flash.
void writeBlock(uint8_t *buffer, uint8_t size)
{
    // Convert 7-bit values to 8-bit
    convertTo8Bit(buffer, size);

    // if data_cnt value is 0 and this function runs, it means program is over and there are program data in rpb buffer which is not written to flash, and there is no data in data_pck buffer.
    // So there is no need to copy data_pck values to rpb buffer.
    if (data_cnt != 0)
        memcpy(rpb + (rpb_len), buffer, size - 1);

    // increase the rpb_len value to write next 7-bytes area
    rpb_len += 7;

    // is_program_end flag is set when there is no values coming in 5 seconds from UART, and it means program data is over or program data transfer broke.
    if (rpb_len == FLASH_RPB_BLOCK_SIZE || is_program_end)
    {
#if DEBUG
        printf("WRITEBLOCK: rpb len is equal to block size. Programming...\n");
#endif
        // write the buffer to flash
        flash_range_program(FLASH_REPROGRAM_OFFSET + (ota_block_count * FLASH_RPB_BLOCK_SIZE), rpb, FLASH_RPB_BLOCK_SIZE);
        // jump to next block offset to write data automatically
        ota_block_count++;
        // reset buffer and length value of rpb.
        memset(rpb, 0, FLASH_RPB_BLOCK_SIZE);
        rpb_len = 0;
    }
}

// this functiın gets the character coming from UART and adds it to data_pck buffer and copies it to rpb buffer.
void writeProgramToFlash(uint8_t chr)
{
    // copy character to data_pack value. If is_program_end flag is set it means there is no character coming from UART the no need to copy a NULL value to data_pack buffer.
    if (!is_program_end)
        data_pck[data_cnt++] = chr;

    // if data_pck buffer is full or is_program_end flag is set the buffer can be copied to rpb buffer
    if (data_cnt == 8 || is_program_end)
    {
        // copy the data_pck buffer to rpb buffer
        writeBlock(data_pck, data_cnt);

        // reset the data_pck buffer and its length value
        memset(data_pck, 0, 8);
        data_cnt = 0;
    }
}

// This function gets the contents like sector data, last records contents from flash and sets them to variables.
void __not_in_flash_func(getFlashContents)()
{
    // disable interrupts and get the contents
    uint32_t ints = save_and_disable_interrupts();
    sector_data = *(uint16_t *)flash_sector_content;
    uint8_t *flash_target_contents = (uint8_t *)(XIP_BASE + FLASH_DATA_OFFSET + (sector_data * FLASH_SECTOR_SIZE));
    memcpy(flash_data, flash_target_contents, FLASH_SECTOR_SIZE);
    // enable interrupts
    restore_interrupts(ints);
}

// This function writes current sector data to flash.
void __not_in_flash_func(setSectorData)()
{
    uint16_t sector_buffer[FLASH_PAGE_SIZE / sizeof(uint16_t)] = {0};
    sector_buffer[0] = sector_data;
#if DEBUG
    printf("SETSECTORDATA: sector data which is going to be written: %d\n", sector_data);
#endif
    flash_range_erase(FLASH_SECTOR_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_SECTOR_OFFSET, (uint8_t *)sector_buffer, FLASH_SECTOR_SIZE);
}

// This function converts the datetime value to char array as characters to write flash correctly
void setDateToCharArray(int value, char *array)
{
    // if value is smaller than 10, second character of array will be always 0, first element of array will be value
    if (value < 10)
    {
        array[0] = '0';
        array[1] = value + '0';
    }
    // if value is bigger than 10, two character will be written to char array, first element of array keeps ones place, second element of array keeps tens place
    else
    {
        array[0] = value / 10 + '0';
        array[1] = value % 10 + '0';
    }
}

// this function converts a double value's floating value to uint8_t value.
uint8_t doubleFloatingToUint8t(double double_value)
{
    // get floating value of double value and multiply it with 10, so we can get the first digit. We set that value an uint8_t value because we don't want to get rest of the floating digits.
    uint8_t floating_value = (double_value - floor(double_value)) * 10;
#if DEBUG
    printf("double value after subtraction: %lf\n", double_value);
    printf("floating value after floor and uint8_t: %d\n\n", floating_value);
#endif
    return floating_value;
}

// This function gets a buffer which includes VRMS values, and calculate the max, min and mean values of this buffer and sets the variables.
void vrmsSetMinMaxMean(double *buffer, uint8_t size)
{
    double buffer_max = buffer[0];
    double buffer_min = buffer[0];
    double buffer_sum = buffer[0];

    for (uint8_t i = 1; i < size; i++)
    {
        if (buffer[i] > buffer_max)
            buffer_max = buffer[i];

        if (buffer[i] < buffer_min)
            buffer_min = buffer[i];

        buffer_sum += buffer[i];
    }

    vrms_max = (uint8_t)floor(buffer_max);
    vrms_min = (uint8_t)floor(buffer_min);
    vrms_mean = (uint8_t)(buffer_sum / size);
    vrms_max_dec = doubleFloatingToUint8t(buffer_max);
    vrms_min_dec = doubleFloatingToUint8t(buffer_min);
    vrms_mean_dec = doubleFloatingToUint8t(buffer_sum / size);

#if DEBUG
    printf("buffer max: %lf,vrms_max: %d,vrms_max_dec: %d\n", buffer_max, vrms_max, vrms_max_dec);
    printf("buffer min: %lf,vrms_min: %d,vrms_min_dec: %d\n", buffer_min, vrms_min, vrms_min_dec);
    printf("buffer mean: %lf,vrms_mean: %d,vrms_mean_dec: %d\n", buffer_sum / size, vrms_mean, vrms_mean_dec);
#endif
}

// This function sets the current time values which are 16 bytes total and calculated VRMS values to flash
void setFlashData()
{
    // initialize the variables
    struct FlashData data;
    uint8_t *flash_target_contents = (uint8_t *)(XIP_BASE + FLASH_DATA_OFFSET + (sector_data * FLASH_SECTOR_SIZE));
    uint16_t offset;

    // set date values and VRMS values to FlashData struct variable
    setDateToCharArray(current_time.year, data.year);
    setDateToCharArray(current_time.month, data.month);
    setDateToCharArray(current_time.day, data.day);
    setDateToCharArray(current_time.hour, data.hour);
    setDateToCharArray(current_time.min, data.min);
    data.max_volt = vrms_max;
    data.min_volt = vrms_min;
    data.mean_volt = vrms_mean;
    data.max_volt_dec = vrms_max_dec;
    data.min_volt_dec = vrms_min_dec;
    data.mean_volt_dec = vrms_mean_dec;

    // reset VRMS values
    vrms_max = 0;
    vrms_min = 0;
    vrms_mean = 0;
    vrms_max_dec = 0;
    vrms_min_dec = 0;
    vrms_mean_dec = 0;

    // find the last offset of flash records and write current values to last offset of flash_data buffer
    for (offset = 0; offset < FLASH_SECTOR_SIZE; offset += FLASH_RECORD_SIZE)
    {
        if (flash_target_contents[offset] == '\0' || flash_target_contents[offset] == 0xff)
        {
#if DEBUG
            if (offset == 0)
                printf("SETFLASHDATA: last record is not found.\n");
            else
                printf("SETFLASHDATA: last record is start in %d offset\n", offset - 16);
#endif
            flash_data[offset / FLASH_RECORD_SIZE] = data;
#if DEBUG
            printf("SETFLASHDATA: record saved to offset: %d. used %d/%d of sector.\n", offset, offset + 16, FLASH_SECTOR_SIZE);
#endif
            break;
        }
    }

    // if offset value is equals or bigger than FLASH_SECTOR_SIZE, (4096 bytes) it means current sector is full and program should write new values to next sector
    if (offset >= FLASH_SECTOR_SIZE)
    {
#if DEBUG
        printf("SETFLASHDATA: offset value is equals to sector size.current secor data is: %d. Sector is changing...\n", sector_data);
#endif
        // if current sector is last sector of flash, sector data will be 0 and the program will start to write new records to beginning of the flash record offset
        if (sector_data == FLASH_TOTAL_SECTORS)
            sector_data = 0;
        else
            sector_data++;
#if DEBUG
        printf("SETFLASHDATA: new sector value is: %d\n", sector_data);
#endif
        // reset variables and call setSectorData()
        memset(flash_data, 0, FLASH_SECTOR_SIZE);
        flash_data[0] = data;
        setSectorData();
#if DEBUG
        printf("SETFLASHDATA: Sector changing written to flash.\n");
#endif
    }
}

// This function writes flash_data content to flash area
void __not_in_flash_func(SPIWriteToFlash)()
{
#if DEBUG
    printf("SPIWRITETOFLASH: Setting flash data...\n");
#endif
    setFlashData();
    // setSectorData();

    flash_range_erase(FLASH_DATA_OFFSET + (sector_data * FLASH_SECTOR_SIZE), FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_DATA_OFFSET + (sector_data * FLASH_SECTOR_SIZE), (uint8_t *)flash_data, FLASH_SECTOR_SIZE);
}

// this function converts an array to datetime value
void arrayToDatetime(datetime_t *dt, uint8_t *arr)
{
    dt->year = (arr[0] - '0') * 10 + (arr[1] - '0');
    dt->month = (arr[2] - '0') * 10 + (arr[3] - '0');
    dt->day = (arr[4] - '0') * 10 + (arr[5] - '0');
    dt->hour = (arr[6] - '0') * 10 + (arr[7] - '0');
    dt->min = (arr[8] - '0') * 10 + (arr[9] - '0');
}

// This functon compares two datetime values and return an int value
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

// This function copies a datetime value to another datetime value
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

void getAllRecords(int32_t *st_idx, int32_t *end_idx, datetime_t *start, datetime_t *end)
{
    uint8_t *flash_start_content = (uint8_t *)(XIP_BASE + FLASH_DATA_OFFSET);

    for (int i = 0; i < FLASH_TOTAL_RECORDS; i += 16)
    {
        uint8_t *flash_start_content = (uint8_t *)(XIP_BASE + FLASH_DATA_OFFSET);

        // this is the end index control. if start index occurs and current index starts with FF or current index is NULL which means this is the last index of records
        if ((*st_idx != -1) && (flash_start_content[i] == 0xFF || flash_start_content[i] == 0x00))
        {
            arrayToDatetime(end, &flash_start_content[i - 16]);
            *end_idx = i - 16;
            break;
        }

        // if current index is 0xFF or 0x00, this is an empty record and no need to look for if it is start index, continue the loop
        if (flash_start_content[i] == 0xFF || flash_start_content[i] == 0x00)
            continue;

        // if current record is not empty, this is start index of records.
        if (*st_idx == -1)
        {
            arrayToDatetime(start, &flash_start_content[i]);
            *st_idx = i;
        }
    }
}

void getSelectedRecords(int32_t *st_idx, int32_t *end_idx, datetime_t *start, datetime_t *end, datetime_t *dt_start, datetime_t *dt_end)
{
    uint8_t *flash_start_content = (uint8_t *)(XIP_BASE + FLASH_DATA_OFFSET);

    // convert date and time values to datetime value
    arrayToDatetime(start, reading_state_start_time);
    arrayToDatetime(end, reading_state_end_time);

    // if start date is bigger than end date, it means dates are wrong so function returns
    if (datetimeComp(start, end) > 0)
        return;

    for (uint32_t i = 0; i < FLASH_TOTAL_RECORDS; i += 16)
    {
        // initialize the current datetime
        datetime_t recurrent_time = {0};

        // if current index is empty, continue
        if (flash_start_content[i] == 0xFF || flash_start_content[i] == 0x00)
            continue;

        // if current index is not empty, set datetime to current index record
        arrayToDatetime(&recurrent_time, &flash_start_content[i]);

        // if current record datetime  is bigger than start datetime and start index is not set, this is the start index
        if (datetimeComp(&recurrent_time, start) >= 0)
        {
            if (*st_idx == -1 || (datetimeComp(&recurrent_time, dt_start) < 0))
            {
                *st_idx = i;
                datetimeCopy(&recurrent_time, dt_start);
            }
        }

        // if current record datetime is smaller than end datetime and end index is not set, this is the end index
        if (datetimeComp(&recurrent_time, end) <= 0)
        {
            if (*end_idx == -1 || datetimeComp(&recurrent_time, dt_end) > 0)
            {
                *end_idx = i;
                datetimeCopy(&recurrent_time, dt_end);
            }
        }
    }
}

// This function searches the requested data in flash by starting from flash record beginning offset, collects data from flash and sends it to UART to show load profile content
void searchDataInFlash()
{
    // initialize the variables
    datetime_t start = {0};
    datetime_t end = {0};
    datetime_t dt_start = {0};
    datetime_t dt_end = {0};
    int32_t start_index = -1;
    int32_t end_index = -1;
    char load_profile_line[42] = {0};
    uint8_t *flash_start_content = (uint8_t *)(XIP_BASE + FLASH_DATA_OFFSET);
    uint8_t *date_start = strchr(rx_buffer, '(');
    uint8_t *date_end = strchr(rx_buffer, ')');

    // if there are just ; character between parentheses and this function executes, it means load profile request got without dates so it means all records will be showed in load profile request
    if (date_end - date_start == 2)
    {
#if DEBUG
        printf("SEARCHDATAINFLASH: all records are going to send\n");
#endif
        getAllRecords(&start_index, &end_index, &start, &end);
    }
    // if rx_buffer_len is not 14, request got with dates and records will be showed between those dates.
    else
    {
#if DEBUG
        printf("SEARCHDATAINFLASH: selected records are going to send\n");
#endif
        getSelectedRecords(&start_index, &end_index, &start, &end, &dt_start, &dt_end);
    }
#if DEBUG
    printf("SEARCHDATAINFLASH: Start index is: %ld\n", start_index);
    printf("SEARCHDATAINFLASH: End index is: %ld\n", end_index);
#endif
    // if start index is bigger than end index, swap the values
    if (start_index > end_index)
    {
        uint32_t temp = start_index;
        start_index = end_index;
        end_index = temp;
    }

    // if there start and end index are set, there are records between these times so send them to UART
    if (start_index >= 0 && end_index >= 0)
    {
#if DEBUG
        printf("SEARCHDATAINFLASH: Generating messages...\n");
#endif
        // initialize the variables. XOR result is 0x02 because message to send UART starts with 0x02 (STX) so in BCC calculation, that byte will be ignored
        uint8_t xor_result = 0x02;
        uint32_t start_addr = start_index;
        uint8_t first_flag = 0;
        uint32_t end_addr = start_index <= end_index ? end_index : 1572864;

        for (; start_addr <= end_addr;)
        {
            // set char arrays to initialize a string for record
            char year[3] = {flash_start_content[start_addr], flash_start_content[start_addr + 1], 0x00};
            char month[3] = {flash_start_content[start_addr + 2], flash_start_content[start_addr + 3], 0x00};
            char day[3] = {flash_start_content[start_addr + 4], flash_start_content[start_addr + 5], 0x00};
            char hour[3] = {flash_start_content[start_addr + 6], flash_start_content[start_addr + 7], 0x00};
            char minute[3] = {flash_start_content[start_addr + 8], flash_start_content[start_addr + 9], 0x00};
            uint8_t max = flash_start_content[start_addr + 10];
            uint8_t max_dec = flash_start_content[start_addr + 11];
            uint8_t min = flash_start_content[start_addr + 12];
            uint8_t min_dec = flash_start_content[start_addr + 13];
            uint8_t mean = flash_start_content[start_addr + 14];
            uint8_t mean_dec = flash_start_content[start_addr + 15];

            // if start address equals to end address, it means there is just one record to send or this record is the last record to send
            if (start_addr == end_addr)
            {
                // if this is the first record, it means there is just one record to send so thsi string include STX and BCC together
                if (!first_flag)
                { // 17               19                  4
                    snprintf(load_profile_line, 41, "%c(%s-%s-%s,%s:%s)(%03d.%d,%03d.%d,%03d.%d)\r\n\r%c", 0x02, year, month, day, hour, minute, min, min_dec, max, max_dec, mean, mean_dec, 0x03);
                    first_flag = 1;
                    xor_result = bccGenerate(load_profile_line, 40, xor_result);
                }
                // if this is the not first record, it measn this is the last record to send
                else
                { // 16               19               4
                    snprintf(load_profile_line, 40, "(%s-%s-%s,%s:%s)(%03d.%d,%03d.%d,%03d.%d)\r\n\r%c", year, month, day, hour, minute, min, min_dec, max, max_dec, mean, mean_dec, 0x03);
                    xor_result = bccGenerate(load_profile_line, 39, xor_result);
                }
            }
            // if start address not equals to end address, it means this is the start record or normal record
            else
            {
                // if this record is start record, this string includes STX character
                if (!first_flag)
                { // 17               19               2
                    snprintf(load_profile_line, 39, "%c(%s-%s-%s,%s:%s)(%03d.%d,%03d.%d,%03d.%d)\r\n", 0x02, year, month, day, hour, minute, min, min_dec, max, max_dec, mean, mean_dec);
                    xor_result = bccGenerate(load_profile_line, 38, xor_result);
                    first_flag = 1;
                }
                // if this is not start record, this is the normal record
                else
                { // 16                   19               2
                    snprintf(load_profile_line, 38, "(%s-%s-%s,%s:%s)(%03d.%d,%03d.%d,%03d.%d)\r\n", year, month, day, hour, minute, min, min_dec, max, max_dec, mean, mean_dec);
                    xor_result = bccGenerate(load_profile_line, 37, xor_result);
                }
            }
#if DEBUG
            printf("SEARCHDATAINFLASH: message to send: %s\n", load_profile_line);
#endif
            // send the record to UART and wait
            uart_puts(UART0_ID, load_profile_line);
            if (start_addr == end_addr)
                uart_putc(UART0_ID, xor_result);

            sleep_ms(15);

            // last sector and record control
            if (start_index > end_index && start_addr == 1572848)
            {
                start_addr = 0;
                end_addr = end_index;
            }
            // jump to next record
            else
                start_addr += 16;
        }
    }
    // if start record or end record does not exist, send NACK message
    else
    {
#if DEBUG
        printf("SEARCHDATAINFLASH: data not found.\n");
#endif

        uart_putc(UART0_ID, 0x15);
    }

    // reset time buffers
    memset(reading_state_start_time, 0, 10);
    memset(reading_state_end_time, 0, 10);
#if DEBUG
    printf("SEARCHDATAINFLASH: time values are deleted.\n");
#endif
}

// This function resets records and sector data and set sector data to 0
void resetFlashSettings()
{
#if DEBUG
    printf("RESETFLASHSETTINGS: resetting records anad sector values\n");
#endif

    uint16_t reset_flash[256] = {0};

    flash_range_erase(FLASH_SECTOR_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_SECTOR_OFFSET, (uint8_t *)reset_flash, FLASH_PAGE_SIZE);

    flash_range_erase(FLASH_DATA_OFFSET, 1024 * 1024);
#if DEBUG
    printf("RESETFLASHSETTINGS: erasing is successful.\n");
#endif
}

void checkSectorContent()
{
    uint8_t *flash_sector_content = (uint8_t *)(XIP_BASE + FLASH_SECTOR_OFFSET);
    uint16_t ff_count = 0;

    for (uint16_t i = 0; i < 256; i++)
    {
        if (flash_sector_content[i] == 0xFF)
            ff_count++;
    }

    if (ff_count >= 255)
    {
        #if DEBUG
                printf("CHECKSECTORCONTENT: sector content is going to set 0.\n");
        #endif
        uint16_t sector_buffer[256] = {0};
        flash_range_erase(FLASH_SECTOR_OFFSET, FLASH_SECTOR_SIZE);
        flash_range_program(FLASH_SECTOR_OFFSET, (uint8_t *)sector_buffer, FLASH_PAGE_SIZE);
    }
}

#endif
