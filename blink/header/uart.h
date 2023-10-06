#ifndef UART_H
#define UART_H

#include "defines.h"
#include "variables.h"
#include "bcc.h"

// UART FUNCTIONS
void UARTReceive()
{
    configASSERT(xTaskToNotify_UART == NULL);
    xTaskToNotify_UART = xTaskGetCurrentTaskHandle();
    uart_set_irq_enables(UART0_ID, true, false);
}

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

void initUART()
{
    uart_set_format(UART0_ID, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART0_ID, true);
    int UART_IRQ = UART0_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(UART_IRQ, UARTIsr);
    irq_set_enabled(UART_IRQ, true);
    uart_set_translate_crlf(UART0_ID, true);
}

enum ListeningStates checkListeningData(uint8_t *data_buffer, uint8_t size)
{
    uint8_t reprogram[4] = {0x21, 0x21, 0x21, 0x21};
    uint8_t default_control[4] = {0x01, 0x52, 0x32, 0x02};

    char reading_str[] = "P.01";
    char timeset_str[] = "0.9.1";
    char dateset_str[] = "0.9.2";
    char production_str[] = "96.1.3";

    if (strncmp(reprogram, data_buffer, 4) == 0)
        return ReProgram;

    if (!(bccControl(data_buffer, size)))
    {
#if DEBUG
        printf("bcc control error.");
#endif
        return DataError;
    }

    // Default Control ([SOH]R2[STX])
    if (strncmp(data_buffer, default_control, 4) == 0)
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

void parseReadingData(uint8_t *buffer)
{
    for (uint i = 0; buffer[i] != '\0'; i++)
    {
        if (buffer[i] == 0x28)
        {
            uint8_t k;

            for (k = i + 1; buffer[k] != 0x3B; k++)
                reading_state_start_time[k - (i + 1)] = buffer[k];

            for (uint8_t l = k + 1; buffer[l] != 0x29; l++)
                reading_state_end_time[l - (k + 1)] = buffer[l];

            break;
        }
    }
}

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
    uart_set_baudrate(UART0_ID, selected_baud_rate);
}

void __not_in_flash_func(rebootDevice)()
{
    uint8_t *old_prg_offset = (uint8_t *)(XIP_BASE);
    uint8_t *new_prg_offset = (uint8_t *)(XIP_BASE + FLASH_REPROGRAM_OFFSET);
    uint8_t *ram_offset = (uint8_t *)(0x200000c0);

    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(16 * 1024, 230 * 1024);
    for (int i = 0; i < ota_block_count; i++)
    {
        memcpy(rpb, new_prg_offset, FLASH_RPB_BLOCK_SIZE);
        flash_range_program((16 * 1024) + (i * FLASH_RPB_BLOCK_SIZE), rpb, FLASH_RPB_BLOCK_SIZE);
        new_prg_offset += FLASH_RPB_BLOCK_SIZE;
        sleep_ms(1);
    }

    restore_interrupts(ints);

    hw_clear_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_ENABLE_BITS);
    watchdog_hw->scratch[5] = ENTRY_MAGIC;
    watchdog_hw->scratch[6] = ~ENTRY_MAGIC;
    watchdog_reboot(0, 0, 0);
}

void setReProgramSize(uint8_t *data_buffer)
{
    uint8_t lsb_byte = data_buffer[7];

    for (int i = 4; i < 7; i++)
    {
        uint8_t lsb = lsb_byte & 0x01;
        data_buffer[i] = (data_buffer[i] << 1);
        data_buffer[i] += lsb;
        lsb_byte = (lsb_byte >> 1);

        reprogram_size += data_buffer[i];
        if (i != 6)
        {
            reprogram_size = (reprogram_size << 8);
        }
    }
    uart_putc(UART0_ID, 0x06);
#if DEBUG
    printf("reprogram size: %u\n", reprogram_size);
#endif
}

void resetRxBuffer()
{
    memset(rx_buffer, 0, 256);
    rx_buffer_len = 0;
#if DEBUG
    printf("reset Buffer func!\n");
#endif
}

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

void greetingStateHandler(uint8_t *buffer, uint8_t size)
{
    uint8_t greeting_head[2] = {0x2F, 0x3F};
    uint8_t greeting_head_new[5] = {0x2F, 0x3F, 0x41, 0x4C, 0x50};
    uint8_t greeting_tail[3] = {0x21, 0x0D, 0x0A};
    uint8_t *buffer_tail = strchr(buffer, 0x21);
    uint8_t *serial_num;

    bool greeting_head_check = strncmp(greeting_head, buffer, 2) == 0 ? true : false;
    bool greeting_head_new_check = strncmp(greeting_head_new, buffer, 5) == 0 ? true : false;

    if ((greeting_head_check || greeting_head_new_check) && (strncmp(greeting_tail, buffer_tail, 3) == 0))
    {
#if DEBUG
        printf("greeting entered.\n");
#endif
        if (greeting_head_check)
            serial_num = strchr(buffer, 0x3F) + 1;
        if (greeting_head_new_check)
            serial_num = strchr(buffer, 0x50) + 1;

        // is serial num 8 character and is it the correct serial number
        if (strncmp(serial_num, DEVICE_ID, 8) == 0 && (buffer_tail - serial_num == 8))
        {
            uint8_t program_baud_rate = getProgramBaudRate(max_baud_rate);
            char greeting_uart_buffer[21] = {0};

            snprintf(greeting_uart_buffer, 20, "/ALP%d<1>MAVIALPV2\r\n", program_baud_rate);

            uart_puts(UART0_ID, greeting_uart_buffer);

            state = Setting;
        }
        else
            return;

#if DEBUG
        printf("state is setting.\n");
#endif
    }
}

void settingStateHandler(uint8_t *buffer, uint8_t size)
{
#if DEBUG
    printf("setting state entered.\n");
#endif

    uint8_t load_profile[3] = {0x31, 0x0D, 0x0A};
    uint8_t meter_read[3] = {0x30, 0x0D, 0x0A};
    uint8_t default_control[2] = {0x06, 0x30};

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
            if (buffer[i] == '0')
            {
                modem_baud_rate = buffer[i + 1] - '0';
                if (modem_baud_rate > 5 && modem_baud_rate < 0)
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
            uint8_t ack_buff[17] = {0};
            snprintf(ack_buff, 16, "%cP0%c(%s)%c", 0x01, 0x02,DEVICE_ID, 0x03);
            setBCC(ack_buff, 16, 0x01);

            uart_puts(UART0_ID, ack_buff);
#if DEBUG
            printf("ack sent.\n");
            printf("state is listening.\n");
#endif
            state = Listening;
        }

        // Read Out ([ACK]0Z0[CR][LF])
        if (strncmp(meter_read, (buffer + 3), 3) == 0)
        {
#if DEBUG
            printf("meter read entered.\n");
#endif
            uint8_t mread_data_buff[55] = {0};

            snprintf(mread_data_buff, 55, "%c0.0.0(60616161)\r\n0.9.1(%02d:%02d:%02d)\r\n0.9.2(%02d-%02d-%02d)\r\n!%c", 0x02, current_time.hour, current_time.min, current_time.sec, current_time.year, current_time.month, current_time.day, 0X03);
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

            uart_puts(UART0_ID, mread_data_buff);
        }
    }
}

void setTimeFromUART(uint8_t *buffer)
{
    uint8_t time_buffer[7];
    uint8_t hour;
    uint8_t min;
    uint8_t sec;

    char *start_ptr = strchr(buffer, '(');
    start_ptr++;

    strncpy(time_buffer, start_ptr, 6);
    time_buffer[6] = '\0';

    hour = (time_buffer[0] - '0') * 10 + (time_buffer[1] - '0');
    min = (time_buffer[2] - '0') * 10 + (time_buffer[3] - '0');
    sec = (time_buffer[4] - '0') * 10 + (time_buffer[5] - '0');

    rtc_get_datetime(&current_time);
    setTimePt7c4338(I2C_PORT, I2C_ADDRESS, sec, min, hour, (uint8_t)current_time.dotw, (uint8_t)current_time.day, (uint8_t)current_time.month, (uint8_t)current_time.year);
    getTimePt7c4338(&current_time);
    rtc_set_datetime(&current_time);
    time_change_flag = 1;
}

void setDateFromUART(uint8_t *buffer)
{
    uint8_t time_buffer[7];
    uint8_t year;
    uint8_t month;
    uint8_t day;

    char *start_ptr = strchr(buffer, '(');
    start_ptr++;

    strncpy(time_buffer, start_ptr, 6);
    time_buffer[6] = '\0';

    year = (time_buffer[0] - '0') * 10 + (time_buffer[1] - '0');
    month = (time_buffer[2] - '0') * 10 + (time_buffer[3] - '0');
    day = (time_buffer[4] - '0') * 10 + (time_buffer[5] - '0');

    rtc_get_datetime(&current_time);
    setTimePt7c4338(I2C_PORT, I2C_ADDRESS, (uint8_t)current_time.sec, (uint8_t)current_time.min, (uint8_t)current_time.hour, (uint8_t)current_time.dotw, day, month, year);
    getTimePt7c4338(&current_time);
    rtc_set_datetime(&current_time);
}

bool controlRXBuffer(uint8_t *buffer, uint8_t len)
{
    uint8_t reading[8] = {0x01, 0x52, 0x32, 0x02, 0x50, 0x2E, 0x30, 0x31};
    uint8_t reading_all[11] = {0x01, 0x52, 0x32, 0x02, 0x50, 0x2E, 0x30, 0x31, 0x28, 0x3B, 0x29};
    uint8_t time[9] = {0x01, 0x52, 0x32, 0x02, 0x30, 0x2E, 0x39, 0x2E, 0x31};
    uint8_t date[9] = {0x01, 0x52, 0x32, 0x02, 0x30, 0x2E, 0x39, 0x2E, 0x32};
    uint8_t reprogram[4] = {0x21, 0x21, 0x21, 0x21};
    uint8_t production[10] = {0x01, 0x52, 0x32, 0x02, 0x39, 0x36, 0x2E, 0x31, 0x2E, 0x33};

    uint8_t reprogram_len = 8;
    uint8_t reading_len = 33;
    uint8_t reading_all_len = 13;
    uint8_t time_len = 19;
    uint8_t date_len = 19;
    uint8_t production_len = 14;

    if ((len == reprogram_len) && (strncmp(buffer, reprogram, 4) == 0))
        return 1;
    if (((len == reading_len) && (strncmp(buffer, reading, 8) == 0)) || ((len == reading_all_len) && (strncmp(buffer, reading_all, 11) == 0)))
        return 1;
    if ((len == time_len) && (strncmp(buffer, time, 9) == 0))
        return 1;
    if ((len == date_len) && (strncmp(buffer, date, 9) == 0))
        return 1;
    if ((len == production_len) && (strncmp(buffer, production, 10) == 0))
        return 1;

    return 0;
}

void sendProductionInfo()
{
    char production_obis[22] = {0};
    rtc_get_datetime(&current_time);

    snprintf(production_obis, 21, "%c96.1.3(22-10-05)\r\n%c", 0x02, current_time.year, current_time.month, current_time.day, 0x03);
    setBCC(production_obis, 21, 0x02);

    uart_puts(UART0_ID, production_obis);
}

#endif
