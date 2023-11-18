#ifndef UART_H
#define UART_H

// UART Receiver function
void UARTReceive()
{
    configASSERT(xTaskToNotify_UART == NULL);
    xTaskToNotify_UART = xTaskGetCurrentTaskHandle();
    uart_set_irq_enables(UART0_ID, true, false);
}

// UART Interrupt Service
void UARTIsr()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uart_set_irq_enables(UART0_ID, false, false);

    if (xTaskToNotify_UART != NULL)
    {
        vTaskNotifyGiveFromISR(xTaskToNotify_UART, &xHigherPriorityTaskWoken);
        xTaskToNotify_UART = NULL;
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// UART Initialization
void initUART()
{
    uart_set_format(UART0_ID, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART0_ID, true);
    int UART_IRQ = UART0_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(UART_IRQ, UARTIsr);
    irq_set_enabled(UART_IRQ, true);
    uart_set_translate_crlf(UART0_ID, true);
}

void sendFlashContents()
{
    uint8_t *flash_sector_content = (uint8_t *)(XIP_BASE + FLASH_SECTOR_OFFSET);
    uint8_t *flash_serial_number_content = (uint8_t *)(XIP_BASE + FLASH_SERIAL_OFFSET);
    uint8_t *flash_records = (uint8_t *)(XIP_BASE + FLASH_DATA_OFFSET);
    int offset;

    char debug_uart_buffer[64] = {0};
    sprintf(debug_uart_buffer, "sector content is: %d\r\n\0", flash_sector_content[0]);
    uart_puts(UART0_ID, debug_uart_buffer);

    sprintf(debug_uart_buffer, "serial number of this device is: %s\r\n\0", flash_serial_number_content);
    uart_puts(UART0_ID, debug_uart_buffer);

    for (offset = 0;; offset += 16)
    {
        if (flash_records[offset] == 0xFF)
            break;

        char year[3] = {flash_records[offset], flash_records[offset + 1], 0x00};
        char month[3] = {flash_records[offset + 2], flash_records[offset + 3], 0x00};
        char day[3] = {flash_records[offset + 4], flash_records[offset + 5], 0x00};
        char hour[3] = {flash_records[offset + 6], flash_records[offset + 7], 0x00};
        char minute[3] = {flash_records[offset + 8], flash_records[offset + 9], 0x00};
        char sec[3] = {flash_records[offset + 10], flash_records[offset + 11], 0x00};
        uint8_t max = flash_records[offset + 12];
        uint8_t min = flash_records[offset + 13];
        uint8_t mean = flash_records[offset + 14];

        sprintf(debug_uart_buffer, "%s-%s-%s;%s:%s:%s;%d,%d,%d\r\n\0", year, month, day, hour, minute, sec, max, min, mean);
        uart_puts(UART0_ID, debug_uart_buffer);
    }

    sprintf(debug_uart_buffer, "usage of flash is: %d/%ld bytes\r\n\0", offset, PICO_FLASH_SIZE_BYTES);
    uart_puts(UART0_ID, debug_uart_buffer);
}

// This function check the data which comes when State is Listening, and compares the message to defined strings, and returns a ListeningState value to process the request
enum ListeningStates checkListeningData(uint8_t *data_buffer, uint8_t size)
{
    // define the process strings
    uint8_t reprogram[4] = {0x21, 0x21, 0x21, 0x21};
    uint8_t password[4] = {0x01, 0x50, 0x31, 0x02};
    uint8_t reading_control[4] = {0x01, 0x52, 0x32, 0x02};
    uint8_t writing_control[4] = {0x01, 0x57, 0x32, 0x02};

    char reading_str[] = "P.01";
    char timeset_str[] = "0.9.1";
    char dateset_str[] = "0.9.2";
    char production_str[] = "96.1.3";

    // ReProgram Control (!!!!)
    if (strncmp(reprogram, data_buffer, 4) == 0)
        return ReProgram;

    // if BCC control for this message is false, it means message transfer is not correct so it returns Error
    if (!(bccControl(data_buffer, size)))
    {
#if DEBUG
        printf("CHECKLISTENINGDATA: bcc control error.\n");
#endif
        return DataError;
    }

    // Password Control ([SOH]P1[STX])
    if (strncmp(data_buffer, password, 4) == 0)
    {
#if DEBUG
        printf("CHECKLISTENINGDATA: Password state is accepted in checklisteningdata\n");
#endif
        return Password;
    }

    // Reading Control ([SOH]R2[STX]) or Writing Control ([SOH]W2[STX])
    if (strncmp(data_buffer, reading_control, 4) == 0 || strncmp(data_buffer, writing_control, 4) == 0)
    {
#if DEBUG
        printf("CHECKLISTENINGDATA: default control is accepted in checklisteningdata\n");
#endif
        // Reading Control (P.01)
        if (strstr(data_buffer, reading_str) != NULL)
        {
#if DEBUG
            printf("CHECKLISTENINGDATA: Reading state is accepted in checklisteningdata\n");
#endif
            return Reading;
        }

        // Time Set Control (0.9.1)
        if (strstr(data_buffer, timeset_str) != NULL)
        {
#if DEBUG
            printf("CHECKLISTENINGDATA: Timeset state is accepted in checklisteningdata\n");
#endif
            return TimeSet;
        }

        // Date Set Control (0.9.2)
        if (strstr(data_buffer, dateset_str) != NULL)
        {
#if DEBUG
            printf("CHECKLISTENINGDATA: Dateset state is accepted in checklisteningdata\n");
#endif
            return DateSet;
        }

        // Production Date Control (96.1.3)
        if (strstr(data_buffer, production_str) != NULL)
        {
#if DEBUG
            printf("CHECKLISTENINGDATA: Productioninfo state is accepted in checklisteningdata\n");
#endif
            return ProductionInfo;
        }
    }
#if DEBUG
    printf("CHECKLISTENINGDATA: dataerror state is accepted in checklisteningdata\n");
#endif
    return DataError;
}

// This function deletes a character from a given string
void deleteChar(uint8_t *str, uint8_t len, char chr)
{
    uint8_t i, j;
    for (i = j = 0; i < len; i++)
    {
        if (str[i] != chr)
            str[j++] = str[i];
    }
    str[j] = '\0';
}

// This function gets the load profile data and finds the date characters and add them to time arrays
void parseReadingData(uint8_t *buffer)
{
    for (uint8_t i = 0; buffer[i] != '\0'; i++)
    {
        // if character is "(" then next character will be start of the start date
        if (buffer[i] == 0x28)
        {
            uint8_t k;

            // add start time to array
            for (k = i + 1; buffer[k] != 0x3B; k++)
            {
                reading_state_start_time[k - (i + 1)] = buffer[k];
            }

            // Delete the characters from start date array
            if (rx_buffer_len == 41)
            {
                deleteChar(reading_state_start_time, strlen(reading_state_start_time), '-');
                deleteChar(reading_state_start_time, strlen(reading_state_start_time), ',');
                deleteChar(reading_state_start_time, strlen(reading_state_start_time), ':');
            }

            // add end time to array
            for (uint8_t l = k + 1; buffer[l] != 0x29; l++)
            {
                reading_state_end_time[l - (k + 1)] = buffer[l];
            }
            if (rx_buffer_len == 41)
            {
                // Delete the characters from end date array
                deleteChar(reading_state_end_time, strlen(reading_state_end_time), '-');
                deleteChar(reading_state_end_time, strlen(reading_state_end_time), ',');
                deleteChar(reading_state_end_time, strlen(reading_state_end_time), ':');
            }

            break;
        }
    }
#if DEBUG
    printf("PARSEREADINGDATA: parsed start time: %s\n", reading_state_start_time);
    printf("PARSEREADINGDATA: parsed end time: %s\n", reading_state_end_time);
#endif
}

// This function gets the baud rate number like 1,2,3,4,5
uint8_t getProgramBaudRate(uint16_t b_rate)
{
    switch (b_rate)
    {
    case 300:
        return 0;
    case 600:
        return 1;
    case 1200:
        return 2;
    case 2400:
        return 3;
    case 4800:
        return 4;
    case 9600:
        return 5;
    }
}

// This function sets the device's baud rate according to given number like 1,2,3,4,5
void setProgramBaudRate(uint8_t b_rate)
{
    uint selected_baud_rate;

    switch (b_rate)
    {
    case 0:
        selected_baud_rate = 300;
        break;
    case 1:
        selected_baud_rate = 600;
        break;
    case 2:
        selected_baud_rate = 1200;
        break;
    case 3:
        selected_baud_rate = 2400;
        break;
    case 4:
        selected_baud_rate = 4800;
        break;
    case 5:
        selected_baud_rate = 9600;
        break;
    }
    // set UART's baud rate
    uart_set_baudrate(UART0_ID, selected_baud_rate);
}

// This function reboots this device. This function checks new program area and get the new program area contents and compares if the program written true.
// If program is written correctly, device will be rebooted but if it's not, new program area will be deleted and device won't be rebooted
void __not_in_flash_func(rebootDevice)()
{
    // get contents of new program area
    uint32_t epoch_new = *((uint32_t *)(XIP_BASE + FLASH_REPROGRAM_OFFSET));
    uint32_t epoch_current = *((uint32_t *)(XIP_BASE + FLASH_PROGRAM_OFFSET));
    uint32_t program_len = *((uint32_t *)(XIP_BASE + FLASH_REPROGRAM_OFFSET + 4));
    uint8_t *md5_offset = (uint8_t *)(XIP_BASE + FLASH_REPROGRAM_OFFSET + 8);
    uint8_t *program = (uint8_t *)(XIP_BASE + FLASH_REPROGRAM_OFFSET + FLASH_PAGE_SIZE);

    // calculate MD5 checksum for new program area
    unsigned char md5_local[MD5_DIGEST_LENGTH];
    calculateMD5(program, program_len, md5_local);

    // check if MD5 is true and program's epochs. If epoch is bigger it means the program is newer
    if (!(strncmp(md5_offset, md5_local, 16) == 0) || !(epoch_new > epoch_current))
    {
#if DEBUG
        printf("REBOOTDEVICE: md5 check is false or new program's epoch is smaller or equal.\n");
#endif
        flash_range_erase(FLASH_REPROGRAM_OFFSET, (256 * 1024) - FLASH_SECTOR_SIZE);
    }

    hw_clear_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_ENABLE_BITS);
    watchdog_hw->scratch[5] = ENTRY_MAGIC;
    watchdog_hw->scratch[6] = ~ENTRY_MAGIC;
    watchdog_reboot(0, 0, 0);
}

// This function resets rx_buffer content
void resetRxBuffer()
{
    memset(rx_buffer, 0, 256);
    rx_buffer_len = 0;
#if DEBUG
    printf("reset Buffer func!\n");
#endif
}

// This function sets state to Greeting and resets rx_buffer and it's len. Also it sets the baud rate to 300, which is initial baud rate.
void resetState()
{
    state = Greeting;
    memset(rx_buffer, 0, 256);
    setProgramBaudRate(0);
    rx_buffer_len = 0;
#if DEBUG
    printf("reset state func!\n");
#endif
}

// This function handles in Greeting state. It checks the handshake request message (/? or /?ALP) and if check is true,
void greetingStateHandler(uint8_t *buffer, uint8_t size)
{
    // initialize variables,
    uint8_t *serial_num;
    uint8_t greeting_head[2] = {0x2F, 0x3F};
    uint8_t greeting_head_new[5] = {0x2F, 0x3F, 0x41, 0x4C, 0x50};
    uint8_t greeting_tail[3] = {0x21, 0x0D, 0x0A};
    uint8_t *buffer_tail = strchr(buffer, 0x21);
    bool greeting_head_check = strncmp(greeting_head, buffer, 2) == 0 ? true : false;
    bool greeting_head_new_check = strncmp(greeting_head_new, buffer, 5) == 0 ? true : false;
    bool greeting_tail_check = strncmp(greeting_tail, buffer_tail, 3) == 0 ? true : false;
    uint8_t program_baud_rate = getProgramBaudRate(max_baud_rate);
    char greeting_uart_buffer[20] = {0};

    // check greeting head, greeting head new strings and greeting tail. If these checks are true, the message format is true and this block will be executed
    if ((greeting_head_check || greeting_head_new_check) && greeting_tail_check)
    {
#if DEBUG
        printf("GREETINGSTATEHANDLER: greeting default control passed.\n");
#endif
        // if greeting head does not include [ALP] characters, then serial number beginning offset is just after 0x3F character
        if (greeting_head_check)
            serial_num = strchr(buffer, 0x3F) + 1;

        // if greeting head includes [ALP] characters, then serial number beginning offset is just after 0x50 character
        if (greeting_head_new_check)
            serial_num = strchr(buffer, 0x50) + 1;

        if (serial_num[0] != 0x21)
        {
#if DEBUG
            printf("GREETINGSTATEHANDLER: request came with serial number.\n");
#endif
            // is serial num 8 character and is it the correct serial number
            if (strncmp(serial_num, serial_number, 8) == 0 && (buffer_tail - serial_num == 8))
            {
#if DEBUG
                printf("GREETINGSTATEHANDLER: serial number is true.\n");
#endif
                // send handshake message
                snprintf(greeting_uart_buffer, 20, "/ALP%d<1>MAVIALPV2\r\n", program_baud_rate);
                uart_puts(UART0_ID, greeting_uart_buffer);

                state = Setting;
#if DEBUG
                printf("GREETINGSTATEHANDLER: handshake message sent.\n");
#endif
            }
            // if the message is not correct, do nothing
            else
            {
#if DEBUG
                printf("GREETINGSTATEHANDLER: serial number is false.\n");
#endif
                return;
            }
        }
        else
        {
#if DEBUG
            printf("GREETINGSTATEHANDLER: request came without serial number.\n");
#endif
            // send handshake message
            snprintf(greeting_uart_buffer, 20, "/ALP%d<1>MAVIALPV2\r\n", program_baud_rate);
            uart_puts(UART0_ID, greeting_uart_buffer);

            state = Setting;
#if DEBUG
            printf("GREETINGSTATEHANDLER: handshake message sent.\n");
#endif
        }
    }
}

// This function handles in Setting State. It sets the baud rate and if the message is requested to readout data, it gives readout message
void settingStateHandler(uint8_t *buffer, uint8_t size)
{
    // initialize request strings, can be load profile and meter read, also there is default control which is always in the beginning of the request
    uint8_t load_profile[3] = {0x31, 0x0D, 0x0A};
    uint8_t meter_read[3] = {0x30, 0x0D, 0x0A};
    uint8_t debug_mode[3] = {0x36, 0x0D, 0x0A};
    uint8_t default_control[2] = {0x06, 0x30};

    // if default control is true and size of message is 6, it means the message format is true.
    if ((strncmp(buffer, default_control, 2) == 0) && (size == 6))
    {
#if DEBUG
        printf("SETTINGSTATEHANDLER: default control is passed.\n");
#endif
        uint8_t modem_baud_rate;
        uint8_t i;
        uint8_t program_baud_rate = getProgramBaudRate(max_baud_rate);

        for (i = 0; i < size; i++)
        {
            // if there is a 0 character in message, it means next character is baud rate which modem sends
            if (buffer[i] == '0')
            {
                modem_baud_rate = buffer[i + 1] - '0';
#if DEBUG
                printf("SETTINGSTATEHANDLER: modem's baud rate is: %d.\n", modem_baud_rate);
#endif
                // if modem's baud rate bigger than 5 or smaller than 0, it means modem's baud rate not
                if (modem_baud_rate > 5 || modem_baud_rate < 0)
                {
                    i = size;
                    break;
                }
                if (modem_baud_rate > max_baud_rate)
                {
#if DEBUG
                    printf("SETTINGSTATEHANDLER: modem's baud rate is not acceptable value.\n");
#endif
                    uart_putc(UART0_ID, 0x15);
                }
                else
                {
#if DEBUG
                    printf("SETTINGSTATEHANDLER: modem's baud rate is acceptable value.\n");
#endif
                    setProgramBaudRate(modem_baud_rate);
                }
                break;
            }
        }
        if (i == size)
        {
#if DEBUG
            printf("SETTINGSTATEHANDLER: no baud rate value.\n");
#endif
            uart_putc(UART0_ID, 0x15);
        }

        // Load Profile ([ACK]0Z1[CR][LF]) -> buffer + 3 is beginning of 0[CR][LF] or 1[CR][LF]
        if (strncmp(load_profile, (buffer + 3), 3) == 0)
        {
#if DEBUG
            printf("SETTINGSTATEHANDLER: programming mode accepted.\n");
#endif
            // Generate the message to send UART
            uint8_t ack_buff[17] = {0};
            snprintf(ack_buff, 16, "%cP0%c(%s)%c", 0x01, 0x02, serial_number, 0x03);
            // Set BCC to add the message
            setBCC(ack_buff, 15, 0x01);
#if DEBUG
            printf("SETTINGSTATEHANDLER: ack message to send:\n");
            printBufferHex(ack_buff, 17);
            printf("\n");
#endif
            // SEND Message
            uart_puts(UART0_ID, ack_buff);

            // Change state
            state = Listening;
        }

        // Read Out ([ACK]0Z0[CR][LF]) -> buffer + 3 is beginning of 0[CR][LF] or 1[CR][LF]
        if (strncmp(meter_read, (buffer + 3), 3) == 0)
        {
#if DEBUG
            printf("SETTINGSTATEHANDLER: readout request accepted.\n");
#endif
            // Generate the message to send UART
            uint8_t mread_data_buff[58] = {0};
            snprintf(mread_data_buff, 57, "%c0.0.0(%s)\r\n0.9.1(%02d:%02d:%02d)\r\n0.9.2(%02d-%02d-%02d)\r\n!\r\n%c", 0x02, serial_number, current_time.hour, current_time.min, current_time.sec, current_time.year, current_time.month, current_time.day, 0x03);
            setBCC(mread_data_buff, 56, 0x02);
#if DEBUG
            printf("SETTINGSTATEHANDLER: readout message to send:\n");
            printBufferHex(mread_data_buff, 58);
            printf("\n");
#endif
            // Send the readout data
            uart_puts(UART0_ID, mread_data_buff);
        }

        // Debug Mode ([ACK]0Z6[CR][LF])-> buffer + 3 is beginning of 0[CR][LF] or 1[CR][LF] OR 6[CR][LF]
        if (strncmp(debug_mode, (buffer + 3), 3) == 0)
        {
#if DEBUG
            printf("SETTINGSTATEHANDLER: debug mode accepted.\n");
#endif
            sendFlashContents();
        }
    }
}

// This function sets time via UART
void setTimeFromUART(uint8_t *buffer)
{
    // initialize variables
    uint8_t time_buffer[9] = {0};
    uint8_t hour;
    uint8_t min;
    uint8_t sec;

    // get pointers to get time data between () characters
    char *start_ptr = strchr(buffer, '(');
    start_ptr++;
    char *end_ptr = strchr(buffer, ')');

    // copy time data to buffer and delete the ":" character. First two characters of the array is hour info, next 2 character is minute info, last 2 character is the second info. Also there are two ":" characters to seperate informations.
    strncpy(time_buffer, start_ptr, end_ptr - start_ptr);
    // deleteChar(time_buffer, strlen(time_buffer), ':');

    // set hour,min and sec variables to change time.
    hour = (time_buffer[0] - '0') * 10 + (time_buffer[1] - '0');
    min = (time_buffer[2] - '0') * 10 + (time_buffer[3] - '0');
    sec = (time_buffer[4] - '0') * 10 + (time_buffer[5] - '0');

    // Get the current time and set chip's Time value according to variables and current date values
    setTimePt7c4338(I2C_PORT, I2C_ADDRESS, sec, min, hour, (uint8_t)current_time.dotw, (uint8_t)current_time.day, (uint8_t)current_time.month, (uint8_t)current_time.year);
    // Get new current time from RTC Chip and set to RP2040's RTC module
    getTimePt7c4338(&current_time);
    rtc_set_datetime(&current_time);
    // Set time_change_flag value. This flag provides to align ADCReadTask's period. When time changed, the task that aligned to beginning of the minute will broke, so this flag controls to align task itself beginning of the minute again
    time_change_flag = 1;
}

// This function sets date via UART
void setDateFromUART(uint8_t *buffer)
{
    // initialize variables
    uint8_t date_buffer[9] = {0};
    uint8_t year;
    uint8_t month;
    uint8_t day;

    // get pointers to get date data between () characters
    char *start_ptr = strchr(buffer, '(');
    start_ptr++;
    char *end_ptr = strchr(buffer, ')');

    // copy date data to buffer and delete the "-" character. First two characters of the array is year info, next 2 character is month info, last 2 character is the day info. Also there are two "-" characters to seperate informations.
    strncpy(date_buffer, start_ptr, end_ptr - start_ptr);
    // deleteChar(date_buffer, strlen(date_buffer), '-');

    // set year,month and day variables to change date.
    year = (date_buffer[0] - '0') * 10 + (date_buffer[1] - '0');
    month = (date_buffer[2] - '0') * 10 + (date_buffer[3] - '0');
    day = (date_buffer[4] - '0') * 10 + (date_buffer[5] - '0');

    // Get the current time and set chip's date value according to variables and current time values
    setTimePt7c4338(I2C_PORT, I2C_ADDRESS, (uint8_t)current_time.sec, (uint8_t)current_time.min, (uint8_t)current_time.hour, (uint8_t)current_time.dotw, day, month, year);
    // Get new current time from RTC Chip and set to RP2040's RTC module
    getTimePt7c4338(&current_time);
    rtc_set_datetime(&current_time);
}

//  This function controls message coming from UART. If coming message is provides the formats that described below, this message is accepted to precessing.
bool controlRXBuffer(uint8_t *buffer, uint8_t len)
{
    // message formats like password request, reprogram request, reading (load profile) request etc.
    uint8_t password[4] = {0x01, 0x50, 0x31, 0x02};
    uint8_t reprogram[4] = {0x21, 0x21, 0x21, 0x21};
    uint8_t reading[8] = {0x01, 0x52, 0x32, 0x02, 0x50, 0x2E, 0x30, 0x31};
    uint8_t time[9] = {0x01, 0x57, 0x32, 0x02, 0x30, 0x2E, 0x39, 0x2E, 0x31};
    uint8_t date[9] = {0x01, 0x57, 0x32, 0x02, 0x30, 0x2E, 0x39, 0x2E, 0x32};
    uint8_t production[10] = {0x01, 0x52, 0x32, 0x02, 0x39, 0x36, 0x2E, 0x31, 0x2E, 0x33};
    uint8_t reading_all[11] = {0x01, 0x52, 0x32, 0x02, 0x50, 0x2E, 0x30, 0x31, 0x28, 0x3B, 0x29};

    // length of mesasge that should be
    uint8_t time_len = 21;
    uint8_t date_len = 21;
    uint8_t reading_len = 41;
    uint8_t reading_len2 = 33;
    uint8_t reprogram_len = 4;
    uint8_t password_len = 16;
    uint8_t production_len = 14;
    uint8_t reading_all_len = 13;

    // controls for the message
    if ((len == password_len) && (strncmp(buffer, password, 4) == 0))
    {
#if DEBUG
        printf("CONTROLRXBUFFER: incoming message is password request.\n");
#endif
        return true;
    }
    if ((len == reprogram_len) && (strncmp(buffer, reprogram, 4) == 0))
    {
#if DEBUG
        printf("CONTROLRXBUFFER: incoming message is reprogram request.\n");
#endif
        return true;
    }
    if (((len == reading_len) && (strncmp(buffer, reading, 8) == 0)) || ((len == reading_all_len) && (strncmp(buffer, reading_all, 11) == 0)) || ((len == reading_len2) && (strncmp(buffer, reading, 8) == 0) && (strchr(buffer, 0x2D) == NULL)))
    {
#if DEBUG
        printf("CONTROLRXBUFFER: incoming message is reading or reading all request.\n");
#endif
        return true;
    }
    if ((len == time_len) && (strncmp(buffer, time, 9) == 0))
    {
#if DEBUG
        printf("CONTROLRXBUFFER: incoming message is time change request.\n");
#endif
        return true;
    }
    if ((len == date_len) && (strncmp(buffer, date, 9) == 0))
    {
#if DEBUG
        printf("CONTROLRXBUFFER: incoming message is date change request.\n");
#endif
        return true;
    }
    if ((len == production_len) && (strncmp(buffer, production, 10) == 0))
    {
#if DEBUG
        printf("CONTROLRXBUFFER: incoming message is production info request.\n");
#endif
        return true;
    }

    return false;
}

// This function generates a product,on info message and sends it to UART
void sendProductionInfo()
{
    // initialize the buffer and get the current time
    char production_obis[22] = {0};

    // generate the message and add BCC to the message
    snprintf(production_obis, 21, "%c96.1.3(23-10-05)\r\n%c", 0x02, current_time.year, current_time.month, current_time.day, 0x03);
    setBCC(production_obis, 21, 0x02);
#if DEBUG
    printf("SENDPRODUCTIONINFO: production info message to send: %s.\n", production_obis);
#endif
    // send the message to UART
    uart_puts(UART0_ID, production_obis);
}

// This function gets a password and controls the password, if password is true, device sends an ACK message, if not, device sends NACK message
void passwordHandler(uint8_t *buffer, uint8_t len)
{
    char *ptr = strchr(buffer, '(');
    ptr++;

    if (strncmp(ptr, DEVICE_PASSWORD, 8) == 0)
    {
        uart_putc(UART0_ID, 0x06);
        password_correct_flag = true;
    }
    else
        uart_putc(UART0_ID, 0x15);
}

// This functiın handles to request for reprogramming
void ReProgramHandler(uint8_t *buffer, uint8_t len)
{
    // send ACK message to UART
    uart_putc(UART0_ID, 0x06);
#if DEBUG
    printf("REPROGRAMHANDLER: ACK send from reprogram handler.\n");
#endif
    // change the state to reprogram
    state = WriteProgram;
#if DEBUG
    printf("REPROGRAMHANDLER: state changed from reprogram handler.\n");
#endif
    // delete ADCRead Task. This task also reaches flash so due to prevent conflict, this task is deleted
    vTaskDelete(xADCHandle);
#if DEBUG
    printf("REPROGRAMHANDLER: adcread task deleted from reprogram handler.\n");
#endif
    // delete the reprogram area to write new program
    flash_range_erase(FLASH_REPROGRAM_OFFSET, FLASH_REPROGRAM_SIZE);
#if DEBUG
    printf("REPROGRAMHANDLER: new program area cleaned from reprogram handler.\n");
#endif
    return;
}

// This function handles to reboot device. This function executes when there is no coming character in 5 second in WriteProgram state.
void RebootHandler()
{
#if DEBUG
    printf("REBOOTHANDLER: reboot Handler entered.\n");
#endif
    // if there is any content in buffers, is_program_end flag is set
    if ((rpb_len > 0) || (data_cnt > 0))
        is_program_end = true;

    // write program execution with null variable and reboot device
    writeProgramToFlash(0x00);
    rebootDevice();
}

#endif
