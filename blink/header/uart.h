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

void sendResetDates()
{
    uint8_t *reset_dates_flash = (uint8_t *)(XIP_BASE + FLASH_RESET_COUNT_OFFSET);
    char date_buffer[20] = {0};
    uint16_t date_offset;
    int result;

    for (date_offset = 0; date_offset < FLASH_SECTOR_SIZE; date_offset += 16)
    {
        if (reset_dates_flash[date_offset] == 0xFF || reset_dates_flash[date_offset] == 0x00)
        {
            break;
        }

        char year[3] = {reset_dates_flash[date_offset], reset_dates_flash[date_offset + 1], 0x00};
        char month[3] = {reset_dates_flash[date_offset + 2], reset_dates_flash[date_offset + 3], 0x00};
        char day[3] = {reset_dates_flash[date_offset + 4], reset_dates_flash[date_offset + 5], 0x00};
        char hour[3] = {reset_dates_flash[date_offset + 6], reset_dates_flash[date_offset + 7], 0x00};
        char min[3] = {reset_dates_flash[date_offset + 8], reset_dates_flash[date_offset + 9], 0x00};
        char sec[3] = {reset_dates_flash[date_offset + 10], reset_dates_flash[date_offset + 11], 0x00};

        result = snprintf(date_buffer, sizeof(date_buffer), "%s-%s-%s,%s:%s:%s\r\n", year, month, day, hour, min, sec);

        if (result >= (int)sizeof(date_buffer))
        {
            sendErrorMessage((char *)"DATEBUFFERSMALL");
            continue;
        }

        uart_puts(UART0_ID, date_buffer);
    }
}

void sendDeviceInfo()
{
    uint8_t *flash_records = (uint8_t *)(XIP_BASE + FLASH_DATA_OFFSET);
    int offset;
    uint16_t record_count = 0;

    char debug_uart_buffer[45] = {0};
    sprintf(debug_uart_buffer, "sector count is: %d\r\n", flash_sector_content[0]);
    uart_puts(UART0_ID, debug_uart_buffer);

    uart_puts(UART0_ID, "System Time is: ");
    uart_puts(UART0_ID, datetime_str);
    uart_puts(UART0_ID, "\r\n");

    sprintf(debug_uart_buffer, "serial number of this device is: %s\r\n", serial_number);
    uart_puts(UART0_ID, debug_uart_buffer);

    for (offset = 0; offset < 1556480; offset += 16)
    {
        if (flash_records[offset] == 0xFF || flash_records[offset] == 0x00)
        {
            continue;
        }

        char year[3] = {flash_records[offset], flash_records[offset + 1], 0x00};
        char month[3] = {flash_records[offset + 2], flash_records[offset + 3], 0x00};
        char day[3] = {flash_records[offset + 4], flash_records[offset + 5], 0x00};
        char hour[3] = {flash_records[offset + 6], flash_records[offset + 7], 0x00};
        char minute[3] = {flash_records[offset + 8], flash_records[offset + 9], 0x00};
        uint8_t max = flash_records[offset + 10];
        uint8_t max_dec = flash_records[offset + 11];
        uint8_t min = flash_records[offset + 12];
        uint8_t min_dec = flash_records[offset + 13];
        uint8_t mean = flash_records[offset + 14];
        uint8_t mean_dec = flash_records[offset + 15];

        sprintf(debug_uart_buffer, "%s-%s-%s;%s:%s;%d.%d,%d.%d,%d.%d\r\n", year, month, day, hour, minute, max, max_dec, min, min_dec, mean, mean_dec);
        uart_puts(UART0_ID, debug_uart_buffer);
        record_count++;
    }

    sprintf(debug_uart_buffer, "usage of flash is: %d/%d bytes\n", record_count * 16, (PICO_FLASH_SIZE_BYTES - FLASH_DATA_OFFSET));
    uart_puts(UART0_ID, debug_uart_buffer);

    sendResetDates();
}

// This function check the data which comes when State is Listening, and compares the message to defined strings, and returns a ListeningState value to process the request
enum ListeningStates checkListeningData(uint8_t *data_buffer, uint8_t size)
{
    // define the process strings
    uint8_t password[4] = {0x01, 0x50, 0x31, 0x02};            // [SOH]P1[STX]
    uint8_t reading_control[4] = {0x01, 0x52, 0x32, 0x02};     // [SOH]R2[STX]
    uint8_t reading_control_alt[4] = {0x01, 0x52, 0x35, 0x02}; // [SOH]R5[STX]
    uint8_t writing_control[4] = {0x01, 0x57, 0x32, 0x02};     // [SOH]W2[STX]

    char reading_str[] = "P.01";
    char timeset_str[] = "0.9.1";
    char dateset_str[] = "0.9.2";
    char production_str[] = "96.1.3";
    char reprogram_str[] = "!!!!";
    char threshold_str[] = "T.V.1";
    char get_threshold_str[] = "T.R.1";
    char threshold_pin[] = "T.P.1";

    // if BCC control for this message is false, it means message transfer is not correct so it returns Error
    if (!(bccControl(data_buffer, size)))
    {
        PRINTF("CHECKLISTENINGDATA: bcc control error.\n");
        return BCCError;
    }

    // Password Control ([SOH]P1[STX])
    if (strncmp((char *)data_buffer, (char *)password, sizeof(password)) == 0)
    {
        PRINTF("CHECKLISTENINGDATA: Password state is accepted in checklisteningdata\n");
        return Password;
    }

    // Reading Control ([SOH]R2[STX])([SOH]R5[STX]), or Writing Control ([SOH]W2[STX])
    if (strncmp((char *)data_buffer, (char *)reading_control, sizeof(reading_control)) == 0 || strncmp((char *)data_buffer, (char *)writing_control, sizeof(writing_control)) == 0 || strncmp((char *)data_buffer, (char *)reading_control_alt, sizeof(reading_control_alt)) == 0)
    {
        PRINTF("CHECKLISTENINGDATA: default control is accepted in checklisteningdata\n");

        // Reading Control (P.01)
        if (strstr((char *)data_buffer, reading_str) != NULL)
        {
            PRINTF("CHECKLISTENINGDATA: Reading state is accepted in checklisteningdata\n");
            return Reading;
        }

        // Time Set Control (0.9.1)
        else if (strstr((char *)data_buffer, timeset_str) != NULL)
        {
            PRINTF("CHECKLISTENINGDATA: Timeset state is accepted in checklisteningdata\n");
            return TimeSet;
        }

        // Date Set Control (0.9.2)
        else if (strstr((char *)data_buffer, dateset_str) != NULL)
        {
            PRINTF("CHECKLISTENINGDATA: Dateset state is accepted in checklisteningdata\n");
            return DateSet;
        }

        // Production Date Control (96.1.3)
        else if (strstr((char *)data_buffer, production_str) != NULL)
        {
            PRINTF("CHECKLISTENINGDATA: Productioninfo state is accepted in checklisteningdata\n");
            return ProductionInfo;
        }

        // ReProgram Control (!!!!)
        else if (strstr((char *)data_buffer, reprogram_str) != NULL)
        {
            PRINTF("CHECKLISTENINGDATA: Reprogram state is accepted in checklisteningdata.\n");
            return WriteProgram;
        }

        // Set Threshold control (T.V.1)
        else if (strstr((char *)data_buffer, threshold_str) != NULL)
        {
            PRINTF("CHECKLISTENINGDATA: Set threshold value is accepted in checklisteningdata.\n");
            return SetThreshold;
        }

        // Get Threshold control (T.R.1)
        else if (strstr((char *)data_buffer, get_threshold_str) != NULL)
        {
            PRINTF("CHECKLISTENINGDATA: Get threshold value is accepted in checklisteningdata.\n");
            return GetThreshold;
        }

        // Set Threshold PIN (T.P.1)
        else if (strstr((char *)data_buffer, threshold_pin) != NULL)
        {
            PRINTF("CHECKLISTENINGDATA: Threshold PIN value is accepted in checklisteningdata.\n");
            return ThresholdPin;
        }
    }

    PRINTF("CHECKLISTENINGDATA: dataerror state is accepted in checklisteningdata\n");
    return DataError;
}

// This function deletes a character from a given string
void deleteChar(uint8_t *str, uint8_t len, char chr)
{
    uint8_t i, j;
    for (i = j = 0; i < len; i++)
    {
        if (str[i] != chr)
        {
            str[j++] = str[i];
        }
    }
    str[j] = '\0';
}

// This function gets the load profile data and finds the date characters and add them to time arrays
void parseLoadProfileDates(uint8_t *buffer, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++)
    {
        if (buffer[i] == 0x28)
        {
            uint8_t k;

            for (k = i + 1; buffer[k] != 0x3B; k++)
            {
                if (k == len)
                {
                    break;
                }

                reading_state_start_time[k - (i + 1)] = buffer[k];
            }

            // Delete the characters from start date array
            if (len == 41)
            {
                deleteChar(reading_state_start_time, strlen((char *)reading_state_start_time), '-');
                deleteChar(reading_state_start_time, strlen((char *)reading_state_start_time), ',');
                deleteChar(reading_state_start_time, strlen((char *)reading_state_start_time), ':');
            }

            // add end time to array
            for (uint8_t l = k + 1; buffer[l] != 0x29; l++)
            {
                if (l == len)
                {
                    break;
                }

                reading_state_end_time[l - (k + 1)] = buffer[l];
            }

            if (len == 41)
            {
                // Delete the characters from end date array
                deleteChar(reading_state_end_time, strlen((char *)reading_state_end_time), '-');
                deleteChar(reading_state_end_time, strlen((char *)reading_state_end_time), ',');
                deleteChar(reading_state_end_time, strlen((char *)reading_state_end_time), ':');
            }

            break;
        }
    }

    PRINTF("PARSELOADPROFILEDATES: parsed start time: %s\n", reading_state_start_time);
    PRINTF("PARSELOADPROFILEDATES: parsed end time: %s\n", reading_state_end_time);
}

uint8_t is_end_connection_message(uint8_t *msg_buf)
{
    uint8_t end_connection_str[5] = {0x01, 0x42, 0x30, 0x03, 0x71}; // [SOH]B0[ETX]q

    return (strncmp((char *)msg_buf, (char *)end_connection_str, 5) == 0) ? 1 : 0;
}

// This function gets the baud rate number like 1-6
uint8_t getProgramBaudRate(uint16_t b_rate)
{
    uint8_t baudrate = 0;

    switch (b_rate)
    {
    case 300:
        baudrate = 0;
        break;
    case 600:
        baudrate = 1;
        break;
    case 1200:
        baudrate = 2;
        break;
    case 2400:
        baudrate = 3;
        break;
    case 4800:
        baudrate = 4;
        break;
    case 9600:
        baudrate = 5;
        break;
    case 19200:
        baudrate = 6;
        break;
    }

    return baudrate;
}

// This function sets the device's baud rate according to given number like 0,1,2,3,4,5,6
uint setProgramBaudRate(uint8_t b_rate)
{
    uint selected_baud_rate = 300;
    uint set_baud_rate = 0;

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
    case 6:
        selected_baud_rate = 19200;
        break;
    }
    // set UART's baud rate
    set_baud_rate = uart_set_baudrate(UART0_ID, selected_baud_rate);
    return set_baud_rate;
}

// This function reboots this device. This function checks new program area and get the new program area contents and compares if the program written true.
// If program is written correctly, device will be rebooted but if it's not, new program area will be deleted and device won't be rebooted
void rebootProgram()
{
    // get contents of new program area
    uint32_t epoch_new = *((uint32_t *)(XIP_BASE + FLASH_REPROGRAM_OFFSET));
    uint32_t epoch_current = *((uint32_t *)(XIP_BASE + FLASH_PROGRAM_OFFSET));
    uint32_t program_len = *((uint32_t *)(XIP_BASE + FLASH_REPROGRAM_OFFSET + 4));
    uint8_t *md5_offset = (uint8_t *)(XIP_BASE + FLASH_REPROGRAM_OFFSET + 8);
    uint8_t *program = (uint8_t *)(XIP_BASE + FLASH_REPROGRAM_OFFSET + FLASH_PAGE_SIZE);

    // calculate MD5 checksum for new program area
    unsigned char md5_local[MD5_DIGEST_LENGTH];
    calculateMD5((char *)program, program_len, md5_local);

    uint32_t ints = save_and_disable_interrupts();

    // check if MD5 is true and program's epochs. If epoch is bigger it means the program is newer
    if (!(strncmp((char *)md5_offset, (char *)md5_local, 16) == 0) || !(epoch_new > epoch_current))
    {
        PRINTF("REBOOTPROGRAM: md5 check is false or new program's epoch is smaller or equal.\n");
        flash_range_erase(FLASH_REPROGRAM_OFFSET, FLASH_REPROGRAM_SIZE);
    }
    restore_interrupts(ints);

    hw_clear_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_ENABLE_BITS);
    watchdog_hw->scratch[5] = ENTRY_MAGIC;
    watchdog_hw->scratch[6] = ~ENTRY_MAGIC;
    watchdog_reboot(0, 0, 0);
}

// This function resets rx_buffer content
void resetRxBuffer()
{
    memset(rx_buffer, 0, sizeof(rx_buffer));
    rx_buffer_len = 0;

    PRINTF("reset Buffer func!\n");
}

// This function sets state to Greeting and resets rx_buffer and it's len. Also it sets the baud rate to 300, which is initial baud rate.
void resetState()
{
    state = Greeting;
    setProgramBaudRate(0);
    resetRxBuffer();

    PRINTF("reset state func!\n");
}

// This function handles in Greeting state. It checks the handshake request message (/? or /?ALP) and if check is true,
void greetingStateHandler(uint8_t *buffer)
{
    // initialize variables,
    uint8_t *serial_num;
    uint8_t greeting_head[2] = {0x2F, 0x3F};                       // /?
    uint8_t greeting_head_new[5] = {0x2F, 0x3F, 0x41, 0x4C, 0x50}; // /?ALP
    uint8_t greeting_tail[3] = {0x21, 0x0D, 0x0A};                 // !\r\n
    uint8_t *buffer_tail = (uint8_t *)strchr((char *)buffer, 0x21);

    bool greeting_head_check = strncmp((char *)greeting_head, (char *)buffer, sizeof(greeting_head)) == 0 ? true : false;
    bool greeting_head_new_check = strncmp((char *)greeting_head_new, (char *)buffer, sizeof(greeting_head_new)) == 0 ? true : false;
    bool greeting_tail_check = strncmp((char *)greeting_tail, (char *)buffer_tail, sizeof(greeting_tail)) == 0 ? true : false;

    uint8_t program_baud_rate = getProgramBaudRate(max_baud_rate);
    char greeting_uart_buffer[20] = {0};

    // check greeting head, greeting head new strings and greeting tail. If these checks are true, the message format is true and this block will be executed
    if ((greeting_head_check || greeting_head_new_check) && greeting_tail_check)
    {
        PRINTF("GREETINGSTATEHANDLER: greeting default control passed.\n");

        // if greeting head does not include [ALP] characters, then serial number beginning offset is just after 0x3F character
        if (greeting_head_check)
        {
            serial_num = (uint8_t *)strchr((char *)buffer, 0x3F) + 1;
        }
        // if greeting head includes [ALP] characters, then serial number beginning offset is just after 0x50 character
        else if (greeting_head_new_check)
        {
            serial_num = (uint8_t *)strchr((char *)buffer, 0x50) + 1;
        }

        PRINTF("GREETINGSTATEHANDLER: serial number: %s\n", serial_num);

        // check if greeting message received with serial number
        if (serial_num[0] != 0x21)
        {
            PRINTF("GREETINGSTATEHANDLER: request came with serial number.\n");

            // if the message is not correct, do nothing
            if (strncmp((char *)serial_num, (char *)serial_number, SERIAL_NUMBER_SIZE) != 0 || (buffer_tail - serial_num != SERIAL_NUMBER_SIZE))
            {
                PRINTF("GREETINGSTATEHANDLER: serial number is false.\n");
                return;
            }
            // is serial num 9 character and is it the correct serial number
            else
            {
                PRINTF("GREETINGSTATEHANDLER: serial number is true.\n");
            }
        }
        else
        {
            PRINTF("GREETINGSTATEHANDLER: request came without serial number.\n");
        }

        // send handshake message
        int result = snprintf(greeting_uart_buffer, 20, "/ALP%d<2>MAVIALPV2\r\n", program_baud_rate);

        if (result >= (int)sizeof(greeting_uart_buffer))
        {
            PRINTF("GREETINGSTATEHANDLER: handshake message is too big.\n");
            sendErrorMessage((char *)"GREETINGBUFOVERFLOW");
            return;
        }

        PRINTF("GREETINGSTATEHANDLER: handshake message:\n");
        printBufferHex((uint8_t *)greeting_uart_buffer, 20);
        PRINTF("\n");

        uart_puts(UART0_ID, greeting_uart_buffer);
        state = Setting;

        PRINTF("GREETINGSTATEHANDLER: handshake message sent.\n");
    }
}

// This function handles in Setting State. It sets the baud rate and if the message is requested to readout data, it gives readout message
void settingStateHandler(uint8_t *buffer, uint8_t size)
{
    // initialize request strings, can be load profile and meter read, also there is default control which is always in the beginning of the request
    uint8_t short_read[3] = {0x36, 0x0D, 0x0A};       // 6\r\n
    uint8_t programming_mode[3] = {0x31, 0x0D, 0x0A}; // 1\r\n
    uint8_t readout[3] = {0x30, 0x0D, 0x0A};          // 0\r\n
    uint8_t debug_mode[3] = {0x34, 0x0D, 0x0A};       // 4\r\n
    uint8_t default_control[2] = {0x06, 0x30};        // [ACK]0

    // if default control is true and size of message is 6, it means the message format is true.
    if ((strncmp((char *)buffer, (char *)default_control, sizeof(default_control)) == 0) && (size == 6))
    {
        PRINTF("SETTINGSTATEHANDLER: default control is passed.\n");

        uint8_t program_baud_rate = getProgramBaudRate(max_baud_rate);
        uint8_t modem_baud_rate = buffer[2] - '0';

        PRINTF("SETTINGSTATEHANDLER: modem's baud rate is: %d.\n", modem_baud_rate);

        if (modem_baud_rate > 6)
        {
            PRINTF("SETTINGSTATEHANDLER: modem's baud rate is bigger than 6 or smaller than 0.\n");

            uart_putc(UART0_ID, NACK);
            return;
        }

        if (modem_baud_rate > program_baud_rate)
        {
            PRINTF("SETTINGSTATEHANDLER: modem's baud rate is bigger than baud rate that device can support.\n");

            uart_putc(UART0_ID, NACK);
            return;
        }

        PRINTF("SETTINGSTATEHANDLER: modem's baud rate is acceptable value.\n");
        uint selectedrate = setProgramBaudRate(modem_baud_rate);
        PRINTF("SETTINGSTATEHANDLER: selected baud rate is: %d.\n", selectedrate);

        // Load Profile ([ACK]0Z1[CR][LF])
        if (strncmp((char *)programming_mode, (char *)(buffer + 3), 3) == 0)
        {
            PRINTF("SETTINGSTATEHANDLER: programming mode accepted.\n");

            // Generate the message to send UART
            uint8_t ack_buff[18] = {0};
            uint8_t ack_bcc = SOH;

            int result = snprintf((char *)ack_buff, sizeof(ack_buff), "%cP0%c(%s)%c", SOH, STX, serial_number, ETX);
            if (result >= (int)sizeof(ack_buff))
            {
                PRINTF("SETTINGSTATEHANDLER: ack message is too big.\n");
                sendErrorMessage((char *)"ACKBUFOVERFLOW");
                return;
            }

            // Generate BCC for acknowledgement message
            bccGenerate(ack_buff, result, &ack_bcc);

            PRINTF("SETTINGSTATEHANDLER: ack message to send:\n");
            printBufferHex(ack_buff, 18);
            PRINTF("\n");

            // SEND Message
            uart_puts(UART0_ID, (char *)ack_buff);
            uart_putc(UART0_ID, ack_bcc);

            // Change state
            state = Listening;
        }

        // Read Out ([ACK]0Z0[CR][LF]) or ([ACK]0Z6[CR][LF])
        if (strncmp((char *)readout, (char *)(buffer + 3), 3) == 0 || strncmp((char *)short_read, (char *)(buffer + 3), 3) == 0)
        {
            PRINTF("SETTINGSTATEHANDLER: readout request accepted.\n");

            char mread_data_buff[21] = {0};
            int result = 0;
            uint8_t readout_xor = 0x00;

            PRINTF("SETTINGSTATEHANDLER: data buffer before starting: \n");
            printBufferHex((uint8_t *)mread_data_buff, 21);
            PRINTF("\n");

            uart_putc(UART0_ID, STX);

            result = snprintf(mread_data_buff, sizeof(mread_data_buff), "0.0.0(%s)\r\n", serial_number);
            bccGenerate((uint8_t *)mread_data_buff, result, &readout_xor);
            uart_puts(UART0_ID, mread_data_buff);
            printBufferHex((uint8_t *)mread_data_buff, 21);
            PRINTF("\n");

            result = snprintf(mread_data_buff, sizeof(mread_data_buff), "0.2.0(%s)\r\n", SOFTWARE_VERSION);
            bccGenerate((uint8_t *)mread_data_buff, result, &readout_xor);
            uart_puts(UART0_ID, mread_data_buff);
            printBufferHex((uint8_t *)mread_data_buff, 21);
            PRINTF("\n");

            result = snprintf(mread_data_buff, sizeof(mread_data_buff), "0.8.4(%d*min)\r\n", load_profile_record_period);
            bccGenerate((uint8_t *)mread_data_buff, result, &readout_xor);
            uart_puts(UART0_ID, mread_data_buff);
            printBufferHex((uint8_t *)mread_data_buff, 21);
            PRINTF("\n");

            result = snprintf(mread_data_buff, sizeof(mread_data_buff), "0.9.1(%02d:%02d:%02d)\r\n", current_time.hour, current_time.min, current_time.sec);
            bccGenerate((uint8_t *)mread_data_buff, result, &readout_xor);
            uart_puts(UART0_ID, mread_data_buff);
            printBufferHex((uint8_t *)mread_data_buff, 21);
            PRINTF("\n");

            result = snprintf(mread_data_buff, sizeof(mread_data_buff), "0.9.2(%02d-%02d-%02d)\r\n", current_time.year, current_time.month, current_time.day);
            bccGenerate((uint8_t *)mread_data_buff, result, &readout_xor);
            uart_puts(UART0_ID, mread_data_buff);
            printBufferHex((uint8_t *)mread_data_buff, 21);
            PRINTF("\n");

            result = snprintf(mread_data_buff, sizeof(mread_data_buff), "96.1.3(%s)\r\n", PRODUCTION_DATE);
            bccGenerate((uint8_t *)mread_data_buff, result, &readout_xor);
            uart_puts(UART0_ID, mread_data_buff);
            printBufferHex((uint8_t *)mread_data_buff, 21);
            PRINTF("\n");

            result = snprintf(mread_data_buff, sizeof(mread_data_buff), "32.7.0(%.2f)\r\n", vrms_max_last);
            bccGenerate((uint8_t *)mread_data_buff, result, &readout_xor);
            uart_puts(UART0_ID, mread_data_buff);
            printBufferHex((uint8_t *)mread_data_buff, 21);
            PRINTF("\n");

            result = snprintf(mread_data_buff, sizeof(mread_data_buff), "52.7.0(%.2f)\r\n", vrms_min_last);
            bccGenerate((uint8_t *)mread_data_buff, result, &readout_xor);
            uart_puts(UART0_ID, mread_data_buff);
            printBufferHex((uint8_t *)mread_data_buff, 21);
            PRINTF("\n");

            result = snprintf(mread_data_buff, sizeof(mread_data_buff), "72.7.0(%.2f)\r\n", vrms_mean_last);
            bccGenerate((uint8_t *)mread_data_buff, result, &readout_xor);
            uart_puts(UART0_ID, mread_data_buff);
            printBufferHex((uint8_t *)mread_data_buff, 21);
            PRINTF("\n");

            result = snprintf(mread_data_buff, sizeof(mread_data_buff), "!\r\n%c", ETX);
            bccGenerate((uint8_t *)mread_data_buff, result, &readout_xor);
            uart_puts(UART0_ID, mread_data_buff);
            printBufferHex((uint8_t *)mread_data_buff, 21);
            PRINTF("\n");

            PRINTF("SETTINGSTATEHANDLER: readout XOR is: %02X.\n", readout_xor);
            uart_putc(UART0_ID, readout_xor);
        }

        // Debug Mode ([ACK]0Z4[CR][LF])
        if (strncmp((char *)debug_mode, (char *)(buffer + 3), 3) == 0)
        {
            PRINTF("SETTINGSTATEHANDLER: debug mode accepted.\n");
            sendDeviceInfo();
        }
    }
}

uint8_t verifyHourMinSec(uint8_t hour, uint8_t min, uint8_t sec)
{
    if (hour > 23 || min > 59 || sec > 59)
    {
        return 0;
    }

    return 1;
}

uint8_t verifyYearMonthDay(uint8_t year, uint8_t month, uint8_t day)
{
    if (year > 99 || month > 12 || day > 31)
    {
        return 0;
    }

    return 1;
}

// This function sets time via UART
void setTimeFromUART(uint8_t *buffer)
{
    if (!password_correct_flag)
    {
        return;
    }

    // initialize variables
    uint8_t time_buffer[9] = {0};
    uint8_t hour;
    uint8_t min;
    uint8_t sec;

    // get pointers to get time data between () characters
    char *start_ptr = strchr((char *)buffer, '(');
    start_ptr++;
    char *end_ptr = strchr((char *)buffer, ')');

    if (start_ptr == NULL && end_ptr == NULL)
    {
        return;
    }

    // copy time data to buffer and delete the ":" character. First two characters of the array is hour info, next 2 character is minute info, last 2 character is the second info. Also there are two ":" characters to seperate informations.
    strncpy((char *)time_buffer, start_ptr, end_ptr - start_ptr);
    deleteChar(time_buffer, strlen((char *)time_buffer), ':');

    // set hour,min and sec variables to change time.
    hour = (time_buffer[0] - '0') * 10 + (time_buffer[1] - '0');
    min = (time_buffer[2] - '0') * 10 + (time_buffer[3] - '0');
    sec = (time_buffer[4] - '0') * 10 + (time_buffer[5] - '0');

    PRINTF("SETTIMEFROMUART: hour: %d, min: %d, sec: %d\n", hour, min, sec);

    if (verifyHourMinSec(hour, min, sec))
    {
        // Get the current time and set chip's Time value according to variables and current date values
        setTimePt7c4338(I2C_PORT, I2C_ADDRESS, sec, min, hour, (uint8_t)current_time.dotw, (uint8_t)current_time.day, (uint8_t)current_time.month, (uint8_t)current_time.year);
        // Get new current time from RTC Chip and set to RP2040's RTC module
        getTimePt7c4338(&current_time);

        // ??
        if (current_time.dotw < 0 || current_time.dotw > 6)
        {
            PRINTF("SETTIMEFROMUART: invalid day of the week: %d!\n", current_time.dotw);
            current_time.dotw = 2;
        }
        //

        bool is_rtc_set = rtc_set_datetime(&current_time);

        if (is_rtc_set)
        {
            PRINTF("SETTIMEFROMUART: time was set to: %d:%d:%d\n", current_time.hour, current_time.min, current_time.sec);
            uart_putc(UART0_ID, ACK);
        }
        else
        {
            PRINTF("SETTTIMEFROMUART: time was not set!\n");
            sendErrorMessage((char *)"TIMENOTSET");
        }
    }
    else
    {
        PRINTF("SETTIMEFROMUART: invalid tim values!\n");
        sendErrorMessage((char *)"INVALIDTIMEVAL");
    }
}

// This function sets date via UART
void setDateFromUART(uint8_t *buffer)
{
    if (!password_correct_flag)
    {
        return;
    }

    // initialize variables
    uint8_t date_buffer[9] = {0};
    uint8_t year;
    uint8_t month;
    uint8_t day;

    // get pointers to get date data between () characters
    char *start_ptr = strchr((char *)buffer, '(');
    start_ptr++;
    char *end_ptr = strchr((char *)buffer, ')');

    if (start_ptr == NULL && end_ptr == NULL)
    {
        return;
    }

    // copy date data to buffer and delete the "-" character. First two characters of the array is year info, next 2 character is month info, last 2 character is the day info. Also there are two "-" characters to seperate informations.
    strncpy((char *)date_buffer, start_ptr, end_ptr - start_ptr);
    deleteChar(date_buffer, strlen((char *)date_buffer), '-');

    // set year,month and day variables to change date.
    year = (date_buffer[0] - '0') * 10 + (date_buffer[1] - '0');
    month = (date_buffer[2] - '0') * 10 + (date_buffer[3] - '0');
    day = (date_buffer[4] - '0') * 10 + (date_buffer[5] - '0');

    PRINTF("SETDATEFROMUART: year: %d, month: %d, day: %d\n", year, month, day);

    if (verifyYearMonthDay(year, month, day))
    {
        // Get the current time and set chip's date value according to variables and current time values
        setTimePt7c4338(I2C_PORT, I2C_ADDRESS, (uint8_t)current_time.sec, (uint8_t)current_time.min, (uint8_t)current_time.hour, (uint8_t)current_time.dotw, day, month, year);
        // Get new current time from RTC Chip and set to RP2040's RTC module
        getTimePt7c4338(&current_time);

        if (current_time.dotw < 0 || current_time.dotw > 6)
        {
            PRINTF("SETDATEFROMUART: invalid day of the week: %d!\n", current_time.dotw);
            current_time.dotw = 2;
        }

        bool is_rtc_set = rtc_set_datetime(&current_time);

        if (is_rtc_set)
        {
            PRINTF("SETDATEFROMUART: date was set to: %d-%d-%d\n", current_time.year, current_time.month, current_time.day);
            uart_putc(UART0_ID, ACK);
        }
        else
        {
            PRINTF("SETDATEFROMUART: date was not set!\n");
            sendErrorMessage((char *)"DATENOTSET");
        }
    }
    else
    {
        PRINTF("SETDATEFROMUART: invalid date values!\n");
        sendErrorMessage((char *)"INVALIDDATEVAL");
    }
}

//  This function controls message coming from UART. If coming message is provides the formats that described below, this message is accepted to precessing.
bool controlRXBuffer(uint8_t *buffer, uint8_t len)
{
    // message formats like password request, reprogram request, reading (load profile) request etc.
    uint8_t password[4] = {0x01, 0x50, 0x31, 0x02};                                               // [SOH]P1[STX]
    uint8_t reprogram[9] = {0x01, 0x57, 0x32, 0x02, 0x21, 0x21, 0x21, 0x21, 0x03};                // [SOH]W2[STX]!!!![ETX]
    uint8_t reading[8] = {0x01, 0x52, 0x32, 0x02, 0x50, 0x2E, 0x30, 0x31};                        // [SOH]R2[STX]P.01
    uint8_t reading_alt[8] = {0x01, 0x52, 0x35, 0x02, 0x50, 0x2E, 0x30, 0x31};                    // [SOH]R5[STX]P.01
    uint8_t time[9] = {0x01, 0x57, 0x32, 0x02, 0x30, 0x2E, 0x39, 0x2E, 0x31};                     // [SOH]W2[STX]0.9.1
    uint8_t date[9] = {0x01, 0x57, 0x32, 0x02, 0x30, 0x2E, 0x39, 0x2E, 0x32};                     // [SOH]W2[STX]0.9.2
    uint8_t production[10] = {0x01, 0x52, 0x32, 0x02, 0x39, 0x36, 0x2E, 0x31, 0x2E, 0x33};        // [SOH]R2[STX]96.1.3
    uint8_t reading_all[11] = {0x01, 0x52, 0x32, 0x02, 0x50, 0x2E, 0x30, 0x31, 0x28, 0x3B, 0x29}; // [SOH]R2[STX]P.01(;)
    uint8_t set_threshold_val[9] = {0x01, 0x57, 0x32, 0x02, 0x54, 0x2E, 0x56, 0x2E, 0x31};        // [SOH]W2[STX]T.V.1
    uint8_t get_threshold_val[9] = {0x01, 0x52, 0x32, 0x02, 0x54, 0x2E, 0x52, 0x2E, 0x31};        // [SOH]R2[STX]T.R.1
    uint8_t set_threshold_pin[9] = {0x01, 0x57, 0x32, 0x02, 0x54, 0x2E, 0x50, 0x2E, 0x31};        // [SOH]W2[STX]T.P.1
    uint8_t end_connection_str[5] = {0x01, 0x42, 0x30, 0x03, 0x71};                               // [SOH]B0[ETX]q

    // length of message that should be
    uint8_t time_len = 21;
    uint8_t date_len = 21;
    uint8_t reading_len = 41;
    uint8_t reprogram_len = 10;
    uint8_t password_len = 16;
    uint8_t production_len = 14;
    uint8_t reading_all_len = 13;
    uint8_t set_threshold_len = 16;
    uint8_t get_threshold_len = 13;
    uint8_t set_threshold_pin_len = 13;
    uint8_t end_connection_str_len = 5;

    // controls for the message
    if ((len == password_len) && (strncmp((char *)buffer, (char *)password, sizeof(password)) == 0))
    {
        PRINTF("CONTROLRXBUFFER: incoming message is password request.\n");
        return true;
    }
    else if ((len == reprogram_len) && (strncmp((char *)buffer, (char *)reprogram, sizeof(reprogram)) == 0))
    {
        PRINTF("CONTROLRXBUFFER: incoming message is reprogram request.\n");
        return true;
    }
    else if (((len == reading_len) && ((strncmp((char *)buffer, (char *)reading, sizeof(reading)) == 0) || (strncmp((char *)buffer, (char *)reading_alt, sizeof(reading_alt)) == 0))) || ((len == reading_all_len) && (strncmp((char *)buffer, (char *)reading_all, sizeof(reading_all)) == 0)))
    {
        PRINTF("CONTROLRXBUFFER: incoming message is reading or reading all request.\n");
        return true;
    }
    else if ((len == time_len) && (strncmp((char *)buffer, (char *)time, sizeof(time)) == 0))
    {
        PRINTF("CONTROLRXBUFFER: incoming message is time change request.\n");
        return true;
    }
    else if ((len == date_len) && (strncmp((char *)buffer, (char *)date, sizeof(date)) == 0))
    {
        PRINTF("CONTROLRXBUFFER: incoming message is date change request.\n");
        return true;
    }
    else if ((len == production_len) && (strncmp((char *)buffer, (char *)production, sizeof(production)) == 0))
    {
        PRINTF("CONTROLRXBUFFER: incoming message is production info request.\n");
        return true;
    }
    else if ((len == set_threshold_len) && (strncmp((char *)buffer, (char *)set_threshold_val, sizeof(set_threshold_val)) == 0))
    {
        PRINTF("CONTROLRXBUFFER: incoming message is set threshold value.\n");
        return true;
    }
    else if ((len == get_threshold_len) && (strncmp((char *)buffer, (char *)get_threshold_val, sizeof(get_threshold_val)) == 0))
    {
        PRINTF("CONTROLRXBUFFER: incoming message is get threshold value.\n");
        return true;
    }
    else if ((len == set_threshold_pin_len) && (strncmp((char *)buffer, (char *)set_threshold_pin, sizeof(set_threshold_pin)) == 0))
    {
        PRINTF("CONTROLRXBUFFER: incoming message is set threshold pin value.\n");
        return true;
    }
    else if ((len == end_connection_str_len) && (strncmp((char *)buffer, (char *)end_connection_str, sizeof(end_connection_str)) == 0))
    {
        PRINTF("CONTROLRXBUFFER: incoming message is end connection request.\n");
        return true;
    }

    return false;
}

// This function generates a product,on info message and sends it to UART
void sendProductionInfo()
{
    // initialize the buffer and get the current time
    char production_obis_buffer[24] = {0};
    uint8_t production_bcc = STX;

    // generate the message and add BCC to the message
    int result = snprintf(production_obis_buffer, sizeof(production_obis_buffer), "%c96.1.3(%s)\r\n%c", STX, PRODUCTION_DATE, ETX);
    if (result >= (int)sizeof(production_obis_buffer))
    {
        PRINTF("SENDPRODUCTIONINFO: production buffer is too small.\n");
        uart_putc(UART0_ID, NACK);
        return;
    }

    bccGenerate((uint8_t *)production_obis_buffer, result, &production_bcc);
    PRINTF("SENDPRODUCTIONINFO: production info message to send: %s\n", production_obis_buffer);

    // send the message to UART
    uart_puts(UART0_ID, production_obis_buffer);
    uart_putc(UART0_ID, production_bcc);
}

// This function gets a password and controls the password, if password is true, device sends an ACK message, if not, device sends NACK message
void passwordHandler(uint8_t *buffer)
{
    char *ptr = strchr((char *)buffer, '(');
    ptr++;

    if (strncmp(ptr, DEVICE_PASSWORD, 8) == 0)
    {
        uart_putc(UART0_ID, ACK);
        password_correct_flag = true;
    }
    else
    {
        sendErrorMessage((char *)"PWNOTCORRECT");
    }
}

// This function handles to request for reprogramming
void ReProgramHandler()
{
    // send ACK message to UART
    uart_putc(UART0_ID, ACK);
    PRINTF("REPROGRAMHANDLER: ACK send from reprogram handler.\n");

    // change the state to reprogram
    state = ReProgram;
    PRINTF("REPROGRAMHANDLER: state changed from reprogram handler.\n");

    // delete ADCRead Task. This task also reaches flash so due to prevent conflict, this task is deleted
    vTaskDelete(xADCHandle);
    PRINTF("REPROGRAMHANDLER: adcread task deleted from reprogram handler.\n");

    // delete the reprogram area to write new program
    flash_range_erase(FLASH_REPROGRAM_OFFSET, FLASH_REPROGRAM_SIZE);
    PRINTF("REPROGRAMHANDLER: new program area cleaned from reprogram handler.\n");
}

// This function handles to reboot device. This function executes when there is no coming character in 5 second in WriteProgram state.
void RebootHandler()
{
    PRINTF("REBOOTHANDLER: reboot Handler entered.\n");

    // if there is any content in buffers, is_program_end flag is set
    if ((rpb_len > 0) || (data_cnt > 0))
    {
        is_program_end = true;
    }

    // write program execution with null variable and reboot device
    writeProgramToFlash(0x00);
    rebootProgram();
}

void setThresholdValue(uint8_t *data)
{
    PRINTF("threshold value before change is: %d\n", getVRMSThresholdValue());

    // get value start and end pointers
    char *start_ptr = strchr((char *)data, '(');
    char *end_ptr = strchr((char *)data, ')');
    // to keep string threshold value
    uint8_t threshold_val_str[3];
    // converted threshold value from string
    uint16_t threshold_val;
    // array and pointer to write updated values to flash memory
    uint16_t th_arr[256 / sizeof(uint16_t)] = {0};
    uint16_t *th_ptr = (uint16_t *)(XIP_BASE + FLASH_THRESHOLD_INFO_OFFSET);

    // inc start pointer
    start_ptr++;

    // get string threshold value
    strncpy((char *)threshold_val_str, start_ptr, end_ptr - start_ptr);

    // convert threshold value to uint16_t type
    threshold_val = atoi((char *)threshold_val_str);

    PRINTF("threshold value as string is: %s\n", threshold_val_str);
    PRINTF("threshold value as int is: %d\n", threshold_val);

    // set the threshold value to use in program
    setVRMSThresholdValue(threshold_val);

    // set array variables to updated values (just threshold changed)
    th_arr[0] = getVRMSThresholdValue();
    th_arr[1] = th_ptr[1];

    // write updated values to flash
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_THRESHOLD_INFO_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_THRESHOLD_INFO_OFFSET, (uint8_t *)th_arr, FLASH_PAGE_SIZE);
    restore_interrupts(ints);

    PRINTF("threshold info content is:  \n");
    printBufferHex((uint8_t *)th_ptr, FLASH_PAGE_SIZE);
    PRINTF("\n");

    uart_putc(UART0_ID, ACK);
    PRINTF("SETTHRESHOLDVALUE: ACK send from set threshold value.\n");
}

void getThresholdRecord()
{
    // record area offset pointer
    uint8_t *record_ptr = (uint8_t *)(XIP_BASE + FLASH_THRESHOLD_OFFSET);
    // xor result to send as bcc
    uint8_t xor_result = 0x00;
    // buffer to format
    uint8_t buffer[35] = {0};

    // send STX character
    uart_putc(UART0_ID, STX);

    // find the records and format it in a string buffer
    for (unsigned int i = 0; i < 4 * FLASH_SECTOR_SIZE; i += 16)
    {
        // if beginning offset of a record starts with FF or 00, it means it is an empty record, so finish the formatting process
        if (record_ptr[i] == 0xFF || record_ptr[i] == 0x00)
        {
            continue;
        }

        // copy the flash content in struct
        uint8_t year[3] = {record_ptr[i], record_ptr[i + 1], 0x00};
        uint8_t month[3] = {record_ptr[i + 2], record_ptr[i + 3], 0x00};
        uint8_t day[3] = {record_ptr[i + 4], record_ptr[i + 5], 0x00};
        uint8_t hour[3] = {record_ptr[i + 6], record_ptr[i + 7], 0x00};
        uint8_t min[3] = {record_ptr[i + 8], record_ptr[i + 9], 0x00};
        uint16_t vrms = record_ptr[i + 11];
        vrms = (vrms << 8);
        vrms += record_ptr[i + 10];
        uint16_t variance = record_ptr[i + 13];
        variance = (variance << 8);
        variance += record_ptr[i + 12];

        PRINTF("data.year is: %s\n", year);
        PRINTF("data.month is: %s\n", month);
        PRINTF("data.day is: %s\n", day);
        PRINTF("data.hour is: %s\n", hour);
        PRINTF("data.min is: %s\n", min);
        PRINTF("data.vrms is: %d\n", vrms);
        PRINTF("data.variance is: %d\n", variance);

        // format the variables in a buffer
        int result = snprintf((char *)buffer, sizeof(buffer), "(%s-%s-%s,%s:%s)(%03d,%05d)\r\n", year, month, day, hour, min, vrms, variance);

        // xor all bytes of formatted array
        bccGenerate(buffer, 29, &xor_result);

        PRINTF("threshold record to send is: \n");
        printBufferHex(buffer, 29);
        PRINTF("\n");

        if (result >= (int)sizeof(buffer))
        {
            PRINTF("GETTHRESHOLDRECORD: Buffer Overflow! Sending NACK.\n");
            sendErrorMessage((char *)"THBUFFEROVERFLOW");
        }
        else
        {
            // Send the readout data
            uart_puts(UART0_ID, (char *)buffer);
        }
    }

    // xor last bits of data
    xor_result ^= '\r';
    xor_result ^= ETX;

    // send last \r character
    uart_putc(UART0_ID, '\r');
    // send ETX character
    uart_putc(UART0_ID, ETX);
    // send BCC character
    PRINTF("GETTHRESHOLDRECORD: threshold record xor is: %02X\n", xor_result);
    uart_putc(UART0_ID, xor_result);
}

void setThresholdPIN()
{
    if (!password_correct_flag)
    {
        return;
    }

    if (getThresholdSetBeforeFlag())
    {
        PRINTF("SETTHRESHOLDPIN: Threshold PIN set before, resetting pin...\n");

        gpio_put(THRESHOLD_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        setThresholdSetBeforeFlag(0);

        PRINTF("SETTHRESHOLDPIN: Threshold PIN reset\n");
    }

    uart_putc(UART0_ID, ACK);
    PRINTF("SETTHRESHOLDPIN: ACK send from set threshold pin.\n");
}

#endif
