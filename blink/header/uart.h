#ifndef UART_H
#define UART_H

#include "defines.h"
#include "variables.h"
#include "bcc.h"

// UART FUNCTIONS

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
        printf("bcc control error.");
#endif
        return DataError;
    }

    // Password Control ([SOH]P1[STX])
    if (strncmp(data_buffer, password, 4) == 0)
        return Password;

    // Reading Control ([SOH]R2[STX]) or Writing Control ([SOH]W2[STX])
    if (strncmp(data_buffer, reading_control, 4) == 0 || strncmp(data_buffer, writing_control, 4) == 0)
    {
        // Reading Control (P.01)
        if (strstr(data_buffer, reading_str) != NULL)
            return Reading;

        // Time Set Control (0.9.1)
        if (strstr(data_buffer, timeset_str) != NULL)
            return TimeSet;

        // Date Set Control (0.9.2)
        if (strstr(data_buffer, dateset_str) != NULL)
            return DateSet;

        // Production Date Control (96.1.3)
        if (strstr(data_buffer, production_str) != NULL)
            return ProductionInfo;
    }

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
            deleteChar(reading_state_start_time, strlen(reading_state_start_time), '-');
            deleteChar(reading_state_start_time, strlen(reading_state_start_time), ',');
            deleteChar(reading_state_start_time, strlen(reading_state_start_time), ':');

            // add end time to array
            for (uint8_t l = k + 1; buffer[l] != 0x29; l++)
            {
                reading_state_end_time[l - (k + 1)] = buffer[l];
            }

            // Delete the characters from end date array
            deleteChar(reading_state_end_time, strlen(reading_state_end_time), '-');
            deleteChar(reading_state_end_time, strlen(reading_state_end_time), ',');
            deleteChar(reading_state_end_time, strlen(reading_state_end_time), ':');

            break;
        }
    }

    printf("parsed start time: %s\n",reading_state_start_time);
    printf("parsed end time: %s\n",reading_state_end_time);
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
    if ((strncmp(md5_offset, md5_local, 16) == 0) && (epoch_new > epoch_current))
    {
        printf("md5 check is true and new program's epoch is bigger.\n");
        hw_clear_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_ENABLE_BITS);
        watchdog_hw->scratch[5] = ENTRY_MAGIC;
        watchdog_hw->scratch[6] = ~ENTRY_MAGIC;
        watchdog_reboot(0, 0, 0);
    }
    else
    {
        printf("md5 check is false or new program's epoch is smaller or equal.\n");
        flash_range_erase(FLASH_REPROGRAM_OFFSET, (256 * 1024) - FLASH_SECTOR_SIZE);
    }
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

    // check greeting head, greeting head new strings and greeting tail. If these checks are true, the message format is true and this block will be executed
    if ((greeting_head_check || greeting_head_new_check) && greeting_tail_check)
    {
#if DEBUG
        printf("greeting entered.\n");
#endif
        // if greeting head does not include [ALP] characters, then serial number beginning offset is just after 0x3F character
        if (greeting_head_check)
            serial_num = strchr(buffer, 0x3F) + 1;

        // if greeting head includes ALP characters, then serial number beginning offset is just after 0x50 character
        if (greeting_head_new_check)
            serial_num = strchr(buffer, 0x50) + 1;

        // is serial num 8 character and is it the correct serial number
        if (strncmp(serial_num, serial_number, 8) == 0 && (buffer_tail - serial_num == 8))
        {
            uint8_t program_baud_rate = getProgramBaudRate(max_baud_rate);
            char greeting_uart_buffer[21] = {0};

            // send handshake message
            snprintf(greeting_uart_buffer, 20, "/ALP%d<2>MAVIALPV2\r\n", program_baud_rate);
            uart_puts(UART0_ID, greeting_uart_buffer);

            state = Setting;
        }
        // if the message is not correct, do nothing
        else
            return;

#if DEBUG
        printf("state is setting.\n");
#endif
    }
}

// This function handles in Setting State. It sets the baud rate and if the message is requested to readout data, it gives readout message
void settingStateHandler(uint8_t *buffer, uint8_t size)
{
#if DEBUG
    printf("setting state entered.\n");
#endif

    // initialize request strings, can be load profile and meter read, also there is default control which is always in the beginning of the request
    uint8_t load_profile[3] = {0x31, 0x0D, 0x0A};
    uint8_t meter_read[3] = {0x30, 0x0D, 0x0A};
    uint8_t default_control[2] = {0x06, 0x30};

    // if default control is true and size of message is 6, it means the message format is true.
    if ((strncmp(buffer, default_control, 2) == 0) && (size == 6))
    {
#if DEBUG
        printf("default entered.\n");
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

                // if modem's baud rate bigger than 5 or smaller than 0, it means modem's baud rate not
                if (modem_baud_rate > 5 || modem_baud_rate < 0)
                {
                    i = size;
                    break;
                }
                if (modem_baud_rate > max_baud_rate)
                {
                    uart_putc(UART0_ID, 0x15);
                }
                else
                {
#if DEBUG
                    printf("modem baud rate = %d\n", modem_baud_rate);
#endif
                    setProgramBaudRate(modem_baud_rate);
                }
                break;
            }
        }
        if (i == size)
        {
            uart_putc(UART0_ID, 0x15);
        }

        // Load Profile ([ACK]0Z1[CR][LF])
        if (strncmp(load_profile, (buffer + 3), 3) == 0)
        {
            // Generate the message to send UART
            uint8_t ack_buff[17] = {0};
            snprintf(ack_buff, 16, "%cP0%c(%s)%c", 0x01, 0x02, serial_number, 0x03);
            // Set BCC to add the message
            setBCC(ack_buff, 16, 0x01);

            // SEND Message
            uart_puts(UART0_ID, ack_buff);
#if DEBUG
            printf("ack sent.\n");
            printf("state is listening.\n");
#endif
            // Change state
            state = Listening;
        }

        // Read Out ([ACK]0Z0[CR][LF])
        if (strncmp(meter_read, (buffer + 3), 3) == 0)
        {
#if DEBUG
            printf("meter read entered.\n");
#endif
            // Generate the message to send UART
            uint8_t mread_data_buff[55] = {0};
            snprintf(mread_data_buff, 55, "%c0.0.0(%s)\r\n0.9.1(%02d:%02d:%02d)\r\n0.9.2(%02d-%02d-%02d)\r\n!%c", 0x02, serial_number, current_time.hour, current_time.min, current_time.sec, current_time.year, current_time.month, current_time.day, 0X03);
            // Generate a BCC and add to end of the message
            uint8_t bcc = bccCreate(mread_data_buff, 55, 0x02);
            mread_data_buff[54] = bcc;

#if DEBUG
            for (int i = 0; i < 55; i++)
            {
                printf("%02X ", mread_data_buff[i]);
                if (i % 18 == 0 && i != 0)
                    printf("\n");
                vTaskDelay(1);
            }
#endif
            // Send the readout data
            uart_puts(UART0_ID, mread_data_buff);
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
    deleteChar(time_buffer, strlen(time_buffer), ':');

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
    deleteChar(date_buffer, strlen(date_buffer), '-');

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
    uint8_t reprogram_len = 4;
    uint8_t password_len = 16;
    uint8_t production_len = 14;
    uint8_t reading_all_len = 13;

    // controls for the message
    if ((len == password_len) && (strncmp(buffer, password, 4) == 0))
        return true;
    if ((len == reprogram_len) && (strncmp(buffer, reprogram, 4) == 0))
        return true;
    if (((len == reading_len) && (strncmp(buffer, reading, 8) == 0)) || ((len == reading_all_len) && (strncmp(buffer, reading_all, 11) == 0)))
        return true;
    if ((len == time_len) && (strncmp(buffer, time, 9) == 0))
        return true;
    if ((len == date_len) && (strncmp(buffer, date, 9) == 0))
        return true;
    if ((len == production_len) && (strncmp(buffer, production, 10) == 0))
        return true;

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

    // send the message to UART
    uart_puts(UART0_ID, production_obis);
}

// This function gets a password and controls the password, if password is true, device sends an ACK message, if not, device sends NACK message
void passwordHandler(uint8_t *buffer, uint8_t len)
{
    char *ptr = strchr(buffer, '(');
    ptr++;

    if (strncmp(ptr, DEVICE_PASSWORD, 8) == 0)
        uart_putc(UART0_ID, 0x06);
    else
        uart_putc(UART0_ID, 0x15);
}

// This functiın handles to request for reprogramming
void ReProgramHandler(uint8_t *buffer, uint8_t len)
{   
    // send ACK message to UART
    uart_putc(UART0_ID, 0x06);
    // change the state to reprogram
    state = WriteProgram;
    // delete ADCRead Task. This task also reaches flash so due to prevent conflict, this task is deleted
    vTaskDelete(xADCHandle);
    // delete the reprogram area to write new program
    flash_range_erase(FLASH_REPROGRAM_OFFSET, FLASH_REPROGRAM_SIZE);
    return;
}

// This function handles to reboot device. This function executes when there is no coming character in 5 second in WriteProgram state.
void RebootHandler()
{   
    // if there is any content in buffers, is_program_end flag is set
    if ((rpb_len > 0) || (data_cnt > 0))
        is_program_end = true;

    // write program execution with null variable and reboot device
    writeProgramToFlash(0x00);
    rebootDevice();
}

#endif
