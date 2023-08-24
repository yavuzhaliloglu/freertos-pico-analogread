#include "stdio.h"
#include <stdlib.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include <time.h>
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/rtc.h"
#include "hardware/i2c.h"
#include "pico/util/datetime.h"
#include "hardware/adc.h"
#include "math.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include <string.h>
#include "pico/bootrom.h"
#include "hardware/irq.h"
#include "timers.h"

// SPI DEFINES
#define FLASH_DATA_COUNT_OFFSET 512 * 1024
#define FLASH_TOTAL_SECTORS 382
#define FLASH_TARGET_OFFSET (512 * 1024) + FLASH_SECTOR_SIZE
#define FLASH_DATA_SIZE 16
#define FLASH_TOTAL_DATA_COUNT (PICO_FLASH_SIZE_BYTES - (FLASH_TARGET_OFFSET)) / FLASH_DATA_SIZE

// UART DEFINES
#define UART0_ID uart0 // UART0 for RS485
#define BAUD_RATE 300
#define UART0_TX_PIN 0
#define UART0_RX_PIN 1
#define DATA_BITS 7
#define STOP_BITS 2
#define PARITY UART_PARITY_EVEN
#define UART_TASK_PRIORITY 3
#define UART_TASK_STACK_SIZE (1024 * 3)

// I2C DEFINES
#define I2C_PORT i2c0
#define I2C_ADDRESS 0x68
#define RTC_I2C_SDA_PIN 20
#define RTC_I2C_SCL_PIN 21

// RTC DEFINES
#define PT7C4338_REG_SECONDS 0x00

// RESET PIN DEFINE
#define RESET_PULSE_PIN 2
#define INTERVAL_MS 60000

// ADC DEFINES
#define VRMS_SAMPLE 500
#define VRMS_BUFFER_SIZE 15
#define VRMS_DATA_BUFFER_TIME 60000
#define CLOCK_DIV 9600
#define DEBUG 0

// ADC VARIABLES
uint8_t vrms_max = 0;
uint8_t vrms_min = 0;
uint8_t vrms_mean = 0;
uint16_t sample_buf[VRMS_SAMPLE];
TickType_t remaining_time;
uint8_t set_time_flag = 0;

// SPI VARIABLES
uint8_t *flash_sector_contents = (uint8_t *)(XIP_BASE + FLASH_DATA_COUNT_OFFSET);
uint8_t sector_buffer[FLASH_SECTOR_SIZE / sizeof(uint8_t)] = {0};
static uint8_t sector_data = 0;
struct FlashData
{
    char year[2];
    char month[2];
    char day[2];
    char hour[2];
    char min[2];
    char sec[2];
    uint8_t max_volt;
    uint8_t min_volt;
    uint8_t mean_volt;
    uint8_t eod_character;
};
struct FlashData flash_data[FLASH_SECTOR_SIZE / sizeof(struct FlashData)] = {0};

// RTC VARIABLES
char datetime_buf[64];
char *datetime_str = &datetime_buf[0];

datetime_t t = {
    .year = 2020,
    .month = 06,
    .day = 05,
    .dotw = 5, // 0 is Sunday, so 5 is Friday
    .hour = 15,
    .min = 45,
    .sec = 00};

// UART VARIABLES
volatile TaskHandle_t xTaskToNotify_UART = NULL;
uint8_t state = 0;
uint8_t baud_rate;
uint16_t max_baud_rate = 9600;
uint8_t rxBuffer[256] = {};
uint8_t data_len = 0;
uint8_t start_time[10];
uint8_t end_time[10];

// UART FUNCTIONS
void UART_receive()
{
    configASSERT(xTaskToNotify_UART == NULL);
    xTaskToNotify_UART = xTaskGetCurrentTaskHandle();
    uart_set_irq_enables(UART0_ID, true, false);
}

void UART_Isr()
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
    irq_set_exclusive_handler(UART_IRQ, UART_Isr);
    irq_set_enabled(UART_IRQ, true);
    uart_set_translate_crlf(UART0_ID, true);
}

uint8_t bcc_control_buffer(uint8_t *data_buffer, uint8_t size, uint8_t xor)
{
    for (uint8_t i = 0; i < size; i++)
    {
        xor ^= data_buffer[i];
    }
    return xor;
}

int8_t check_uart_data(uint8_t *data_buffer, uint8_t size)
{
    if ((data_buffer[0] == 0x01) && (data_buffer[1] == 0x52) && (data_buffer[2] == 0x32) && (data_buffer[3] == 0x02))
    {
        // veri alma sorgusu
        if ((data_buffer[4] == 0x50) && (data_buffer[5] == 0x2E) && (data_buffer[6] == 0x30) && (data_buffer[7] == 0x31))
        {
            // buraya son 2 biti silecek bir yapı ekleyebilirsin
            uint8_t xor_result = 0x01;

            for (uint8_t i = 0; i < size; i++)
            {
                xor_result ^= data_buffer[i];
            }

            if (xor_result == data_buffer[size - 1])
                return 0;
        }

        // zaman setleme sorgusu (0.9.1)
        if ((data_buffer[4] == 0x31) && (data_buffer[6] == 0x39) && (data_buffer[8] == 0x34))
        {
            return 1;
        }
        // tarih setleme sorgusu (0.9.2)
        if ((data_buffer[4] == 0x31) && (data_buffer[6] == 0x39) && (data_buffer[8] == 0x32))
        {
            return 2;
        }
    }
    return -1;
}

bool bcc_control(uint8_t *buffer, uint8_t size)
{
    uint8_t xor_result = 0x01;

    for (uint8_t i = 0; i < size - 3; i++)
    {
        xor_result ^= buffer[i];
    }

    return xor_result == buffer[size - 3];
}

void parse_uart_data(uint8_t *buffer)
{
    for (uint i = 0; buffer[i] != '\0'; i++)
    {
        if (buffer[i] == 0x28)
        {
            uint8_t k;

            for (k = i + 1; buffer[k] != 0x3B; k++)
            {
                start_time[k - (i + 1)] = buffer[k];
            }

            for (uint8_t l = k + 1; buffer[l] != 0x29; l++)
            {
                end_time[l - (k + 1)] = buffer[l];
            }
            break;
        }
    }
}

uint8_t get_baud_rate(uint16_t b_rate)
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

void set_baud_rate(uint8_t b_rate)
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

void reset_rxBuffer()
{
    memset(rxBuffer, 0, 256);
    data_len = 0;
}

void reset_state()
{
    state = 0;
    memset(rxBuffer, 0, 256);
    set_baud_rate(0);
    data_len = 0;
}

void dateTimeParse(char *dateTime, uint8_t dotw)
{
    t.year = dateTime[6] % 16 + 10 * (dateTime[6] / 16);
    t.month = dateTime[5] % 16 + 10 * (dateTime[5] / 16);
    t.day = dateTime[4] % 16 + 10 * (dateTime[4] / 16);
    t.dotw = dotw;
    t.hour = dateTime[2] % 16 + 10 * (dateTime[2] / 16);
    t.min = dateTime[1] % 16 + 10 * (dateTime[1] / 16);
    t.sec = dateTime[0] % 16 + 10 * (dateTime[0] / 16);
}

uint8_t decimalToBCD(uint8_t decimalValue)
{
    return ((decimalValue / 10) << 4) | (decimalValue % 10);
}

void set_time_pt7c4338(struct i2c_inst *i2c, uint8_t address, uint8_t seconds, uint8_t minutes, uint8_t hours, uint8_t day, uint8_t date, uint8_t month, uint8_t year)
{
    uint8_t buf[8];
    buf[0] = PT7C4338_REG_SECONDS;
    buf[1] = decimalToBCD(seconds);
    buf[2] = decimalToBCD(minutes);
    buf[3] = decimalToBCD(hours);
    buf[4] = decimalToBCD(day);
    buf[5] = decimalToBCD(date);
    buf[6] = decimalToBCD(month);
    buf[7] = decimalToBCD(year);

    i2c_write_blocking(i2c, address, buf, 8, false);
}

void get_time_pt7c4338(struct i2c_inst *i2c, uint8_t address)
{
    uint8_t buf[7];
    uint8_t dotw;
    buf[0] = PT7C4338_REG_SECONDS;

    i2c_write_blocking(i2c, address, buf, 1, true);
    i2c_read_blocking(i2c, address, buf, 7, false);
    dotw = buf[3];
    dateTimeParse(buf, dotw);
}

// SPI FUNCTIONS

void print_buf(uint8_t *buf, size_t len)
{
    for (size_t i = 0; i < len; ++i)
    {
        printf("%02x", buf[i]);
        if (i % 16 == 15)
            printf("\n");
        else
            printf(" ");
    }
}

// void print_buf(const uint8_t *buf, size_t len)
// {
//     char element_str[2]; // initialized for print_buf, will be removed.

//     for (size_t i = 0; i < len; ++i)
//     {
//         sprintf(element_str, "%02x", buf[i]);
//         uart_puts(UART0_ID, element_str);

//         if (i % 16 == 15)
//             uart_putc(UART0_ID, '\n');
//         else
//             uart_putc(UART0_ID, ' ');
//         if (i % 256 == 0)
//             uart_puts(UART0_ID, "\n\n");
//     }
// }

void __not_in_flash_func(set_sector_data)()
{
    // uint32_t ints = save_and_disable_interrupts();
    sector_buffer[0] = sector_data;
    flash_range_erase(FLASH_DATA_COUNT_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_DATA_COUNT_OFFSET, (uint8_t *)sector_buffer, FLASH_SECTOR_SIZE);
    // restore_interrupts(ints);
}

void set_date_to_char_array(int value, char *array)
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

void vrms_set_max_min_mean(uint8_t *buffer, uint8_t size)
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

void set_flash_data()
{
    struct FlashData data;
    uint8_t *flash_target_contents = (uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET + (sector_data * FLASH_SECTOR_SIZE));
    uint16_t i;

    set_date_to_char_array(t.year, data.year);
    set_date_to_char_array(t.month, data.month);
    set_date_to_char_array(t.day, data.day);
    set_date_to_char_array(t.hour, data.hour);
    set_date_to_char_array(t.min, data.min);
    set_date_to_char_array(t.sec, data.sec);
    data.max_volt = vrms_max;
    data.min_volt = vrms_min;
    data.mean_volt = vrms_mean;
    data.eod_character = 0x04;

    // TO-DO:
    // flasha yazma işleminde yazamadan önce silinip güç kesildiğinde tüm bitler ff kalıyordu ve yazma işlemi yapmıyordu.
    // şuan yazma işlemi yapıyor fakat sektörü silip en başından yapıyor ve veri kaybına sebep oluyor.

    for (i = 0; i < FLASH_SECTOR_SIZE; i += FLASH_DATA_SIZE)
    {
        if (flash_target_contents[i] == '\0' || flash_target_contents[i] == 0xff)
        {
            flash_data[i / FLASH_DATA_SIZE] = data;
            break;
        }
    }

    if (i >= FLASH_SECTOR_SIZE)
    {
        if (sector_data == 382)
            sector_data = 0;
        else
            sector_data++;

        memset(flash_data, 0, FLASH_SECTOR_SIZE);
        flash_data[0] = data;
        set_sector_data();
    }
}

void __not_in_flash_func(spi_write_buffer)()
{
    set_flash_data();
    // set_sector_data();
    // uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_TARGET_OFFSET + (sector_data * FLASH_SECTOR_SIZE), FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_TARGET_OFFSET + (sector_data * FLASH_SECTOR_SIZE), (uint8_t *)flash_data, FLASH_SECTOR_SIZE);
    // restore_interrupts(ints);
}

void array_to_datetime(datetime_t *dt, uint8_t *arr)
{
    dt->year = (arr[0] - '0') * 10 + (arr[1] - '0');
    dt->month = (arr[2] - '0') * 10 + (arr[3] - '0');
    dt->day = (arr[4] - '0') * 10 + (arr[5] - '0');
    dt->hour = (arr[6] - '0') * 10 + (arr[7] - '0');
    dt->min = (arr[8] - '0') * 10 + (arr[9] - '0');
}

int datetime_cmp(datetime_t *dt1, datetime_t *dt2)
{
    if (dt1->year - dt2->year != 0)
    {
        // printf("y: %d\n", dt1->year - dt2->year);
        return dt1->year - dt2->year;
    }
    else if (dt1->month - dt2->month != 0)
    {
        // printf("m: %d\n", dt1->month - dt2->month);
        return dt1->month - dt2->month;
    }
    else if (dt1->day - dt2->day != 0)
    {
        // printf("d: %d\n", dt1->day - dt2->day);
        return dt1->day - dt2->day;
    }
    else if (dt1->hour - dt2->hour != 0)
    {
        // printf("h: %d\n", dt1->hour - dt2->hour);
        return dt1->hour - dt2->hour;
    }
    else if (dt1->min - dt2->min != 0)
    {
        // printf("min: %d\n", dt1->min - dt2->min);
        return dt1->min - dt2->min;
    }
    else if (dt1->sec - dt2->sec != 0)
    {
        // printf("sec: %d\n", dt1->sec - dt2->sec);
        return dt1->sec - dt2->sec;
    }

    return 0;
}

void datetime_cpy(datetime_t *src, datetime_t *dst)
{
    dst->year = src->year;
    dst->month = src->month;
    dst->day = src->day;
    dst->dotw = src->dotw;
    dst->hour = src->hour;
    dst->min = src->min;
    dst->sec = src->sec;
}

void search_data_into_flash()
{
    datetime_t start = {0};
    datetime_t end = {0};

    array_to_datetime(&start, start_time);
    array_to_datetime(&end, end_time);

    if (datetime_cmp(&start, &end) > 0)
        return;

    datetime_t dt_start = {0};
    datetime_t dt_end = {0};
    int32_t start_index = -1;
    int32_t end_index = -1;

    char uart_bcc_checked[100] = {0};
    char uart_string[100] = {0};

    uint8_t *flash_start_content = (uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);

    for (uint32_t i = 0; i < FLASH_TOTAL_DATA_COUNT; i += 16)
    {
        datetime_t rec_time = {0};

        if (flash_start_content[i] == 0xFF || flash_start_content[i] == 0x00)
            continue;

        array_to_datetime(&rec_time, &flash_start_content[i]);
        // printf("start -> %d-%d-%d,%d:%d:%d\n", start.year, start.month, start.day, start.hour, start.min, start.sec);
        // printf("sdt -> %d-%d-%d,%d:%d:%d\n", dt_start.year, dt_start.month, dt_start.day, dt_start.hour, dt_start.min, dt_start.sec);
        // printf("end -> %d-%d-%d,%d:%d:%d\n", end.year, end.month, end.day, end.hour, end.min, end.sec);
        // printf("edt -> %d-%d-%d,%d:%d:%d\n", dt_end.year, dt_end.month, dt_end.day, dt_end.hour, dt_end.min, dt_end.sec);
        // printf("rec -> %d-%d-%d,%d:%d:%d\n", rec_time.year, rec_time.month, rec_time.day, rec_time.hour, rec_time.min, rec_time.sec);

        // printf("sc -> %d\n", datetime_cmp(&rec_time, &start));
        if (datetime_cmp(&rec_time, &start) >= 0)
        {
            if (start_index == -1 || (datetime_cmp(&rec_time, &dt_start) < 0))
            {
                // if (start_index != -1)
                // printf("rec_sc -> %d\n", datetime_cmp(&rec_time, &dt_start));
                // printf("Start match\n");
                start_index = i;
                datetime_cpy(&rec_time, &dt_start);
            }
        }

        // printf("dc -> %d\n", datetime_cmp(&rec_time, &end));
        if (datetime_cmp(&rec_time, &end) <= 0)
        {
            if (end_index == -1 || datetime_cmp(&rec_time, &dt_end) > 0)
            {
                // if (end_index != -1)
                // printf("rec_dc -> %d\n", datetime_cmp(&rec_time, &dt_end));
                // printf("End match\n");
                end_index = i;
                datetime_cpy(&rec_time, &dt_end);
            }
        }
    }

    // printf("%d - %d\n", start_index, end_index);

    if (start_index >= 0 && end_index >= 0)
    {
        // printf("1\n");
        uint8_t xor_result = 0x01;
        uint32_t start_addr = start_index;
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
                snprintf(uart_bcc_checked, 32, "(%s%s%s%s%s)(%03d,%03d,%03d)\r\n\r%c", year, month, day, hour, minute, min, max, mean, 0x03);
                xor_result = bcc_control_buffer(uart_bcc_checked, 29, xor_result);
            }
            else
            {
                snprintf(uart_bcc_checked, 30, "(%s%s%s%s%s)(%03d,%03d,%03d)\r\n", year, month, day, hour, minute, min, max, mean);
                xor_result = bcc_control_buffer(uart_bcc_checked, 27, xor_result);
            }

            if (start_addr == end_addr)
                snprintf(uart_string, 31, "(%s%s%s%s%s)(%03d,%03d,%03d)\r\n\r%c%c", year, month, day, hour, minute, min, max, mean, 0x03, xor_result);
            else
                snprintf(uart_string, 30, "(%s%s%s%s%s)(%03d,%03d,%03d)\r\n", year, month, day, hour, minute, min, max, mean);

            uart_puts(UART0_ID, uart_string);

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
        // printf("2\n");
        uart_putc(UART0_ID, 0x15);
    }
    // printf("3\n");
    memset(start_time, 0, 10);
    memset(end_time, 0, 10);
}

// ADC FUNCTIONS
void __not_in_flash_func(adc_capture)(uint16_t *buf, size_t count)
{
    adc_fifo_setup(true, false, 0, false, false);
    adc_run(true);
    for (int i = 0; i < count; i = i + 1)
        buf[i] = adc_fifo_get_blocking();
    adc_run(false);
    adc_fifo_drain();
}

void state0_handler(uint8_t *buffer, uint8_t size)
{
    if ((buffer[0] == 0x2F) && (buffer[1] == 0x3F) && (buffer[2] == 0x36) && (buffer[3] == 0x31) && (buffer[size - 3] == 0x21) && (buffer[size - 2] == 0x0D) && (buffer[size - 1] == 0x0A))
    {
        baud_rate = get_baud_rate(max_baud_rate);
        char uart_send_info_buffer[19];
        snprintf(uart_send_info_buffer, 17, "/ALP%dMAVIALPV2\r\n", baud_rate);
        uart_send_info_buffer[18] = '\0';
        uart_puts(UART0_ID, uart_send_info_buffer);
        state = 1;
    }
}

void state1_handler(uint8_t *buffer, uint8_t size)
{
    if ((buffer[0] == 0x06) && (buffer[size - 2] == 0x0D) && (buffer[size - 1] == 0x0A) && size == 6)
    {
        uint8_t machine_baud_rate;
        uint8_t i;
        uint8_t program_baud_rate = get_baud_rate(max_baud_rate);

        for (i = 0; i < size; i++)
        {
            if (buffer[i] == '0')
            {
                machine_baud_rate = buffer[i + 1] - '0';
                if ((buffer[i + 2] != '1') && machine_baud_rate > 5 && machine_baud_rate < 0)
                {
                    i = size;
                    break;
                }
                if (machine_baud_rate > max_baud_rate)
                {
                    uart_putc(UART0_ID, 0x15);
                }
                else
                {
                    uart_putc(UART0_ID, 0x06);
                    set_baud_rate(machine_baud_rate);
                    state = 2;
                }
                break;
            }
        }

        if (i == size)
        {
            uart_putc(UART0_ID, 0x15);
        }
    }
    else
    {
        uart_putc(UART0_ID, 0x15);
    }
}

void set_time_uart(uint8_t *buffer)
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

    set_time_pt7c4338(I2C_PORT, I2C_ADDRESS, sec, min, hour, (uint8_t)t.dotw, (uint8_t)t.day, (uint8_t)t.month, (uint8_t)t.year);
    get_time_pt7c4338(I2C_PORT, I2C_ADDRESS);
    datetime_to_str(datetime_str, sizeof(datetime_buf), &t);
    set_time_flag = 1;
}

void set_date_uart(uint8_t *buffer)
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

    set_time_pt7c4338(I2C_PORT, I2C_ADDRESS, (uint8_t)t.sec, (uint8_t)t.min, (uint8_t)t.hour, (uint8_t)t.dotw, day, month, year);
    get_time_pt7c4338(I2C_PORT, I2C_ADDRESS);
    datetime_to_str(datetime_str, sizeof(datetime_buf), &t);
}

// UART TASK
void vUARTTask(void *pvParameters)
{
    (void)pvParameters;
    uint32_t ulNotificationValue;
    xTaskToNotify_UART = NULL;
    uint8_t rxChar;

    TimerHandle_t ResetBufferTimer = xTimerCreate(
        "BufferTimer",
        pdMS_TO_TICKS(5000),
        pdTRUE,
        NULL,
        reset_rxBuffer);

    TimerHandle_t ResetStateTimer = xTimerCreate(
        "StateTimer",
        pdMS_TO_TICKS(30000),
        pdTRUE,
        NULL,
        reset_state);

    while (true)
    {
        xTimerStart(ResetBufferTimer, 0);
        UART_receive();
        ulNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (ulNotificationValue == 1)
        {
            while (uart_is_readable(UART0_ID))
            {
                rxChar = uart_getc(UART0_ID);

                // ENTER CONTROL
                if (rxChar != '\n')
                {
                    xTimerReset(ResetBufferTimer, 0);
                    rxBuffer[data_len++] = rxChar;
                }
                if (rxChar == '\n' || (data_len == 33 && ((rxBuffer[4] == 0x50) && (rxBuffer[5] == 0x2E) && (rxBuffer[6] == 0x30) && (rxBuffer[7] == 0x31))) || (data_len == 18 && ((rxBuffer[4] == 0x31) && (rxBuffer[6] == 0x39) && (rxBuffer[8] == 0x34))) || (data_len == 18 && ((rxBuffer[4] == 0x31) && (rxBuffer[6] == 0x39) && (rxBuffer[8] == 0x32))))
                {
                    rxBuffer[data_len++] = rxChar;
                    xTimerReset(ResetStateTimer, 0);
                    xTimerStop(ResetBufferTimer, 0);

                    // enum oluşturarak stateleri ayarla
                    switch (state)
                    {
                    case 0:
                        xTimerStart(ResetStateTimer, 0);
                        state0_handler(rxBuffer, data_len);
                        break;

                    case 1:
                        xTimerStart(ResetStateTimer, 0);
                        state1_handler(rxBuffer, data_len);
                        break;

                    case 2:
                        xTimerStart(ResetStateTimer, 0);
                        switch (check_uart_data(rxBuffer, data_len))
                        {
                        // kayıt gösterme
                        case -1:
                            uart_putc(UART0_ID, 0x15);
                            break;
                        case 0:
                            parse_uart_data(rxBuffer);
                            search_data_into_flash();
                            break;
                        // saat setleme
                        case 1:
                            if (bcc_control(rxBuffer, data_len))
                            {
                                set_time_uart(rxBuffer);
                            }
                            break;
                        // tarih setleme
                        case 2:
                            if (bcc_control(rxBuffer, data_len))
                            {
                                set_date_uart(rxBuffer);
                            }
                            break;
                        default:
                            uart_putc(UART0_ID, 0x15);
                            break;
                        }
                        break;
                    }

                    memset(rxBuffer, 0, 256);
                    data_len = 0;
                }
            }
        }
    }
}

TickType_t startTime;
TickType_t executionTime;
uint8_t vrms_buffer_count = 0;
double vrms_accumulator = 0.0;
const float conversion_factor = 1000 * (3.3f / (1 << 12));
uint8_t vrms_buffer[VRMS_BUFFER_SIZE] = {0};
double vrms = 0.0;

// ADC CONVERTER TASK
void vADCReadTask()
{

    while (1)
    {
        startTime = xTaskGetTickCount();

        adc_capture(sample_buf, VRMS_SAMPLE);

#if DEBUG
        char deneme[40] = {0};
        for (uint8_t i = 0; i < 150; i++)
        {
            snprintf(deneme, 20, "sample: %d\n", sample_buf[i]);
            deneme[21] = '\0';
            uart_puts(UART0_ID, deneme);
        }

        printf("\n");
#endif
        // double sumSamples = 0;

        // for (int i = 0; i < VRMS_SAMPLE; i++)
        // {
        //     double production = (double)(sample_buf[i]);
        //     sumSamples += production;
        // }
        // uint16_t mean = sumSamples / VRMS_SAMPLE;
        
        int mean = 2050 * conversion_factor;

#if DEBUG
        snprintf(deneme, 30, "mean: %d\n", mean);
        deneme[31] = '\0';
        uart_puts(UART0_ID, deneme);
#endif

        for (uint16_t i = 0; i < VRMS_SAMPLE; i++)
        {
            double production = (double)(sample_buf[i] * conversion_factor);
            vrms_accumulator += pow((production - mean), 2);
        }

#if DEBUG
        snprintf(deneme, 34, "vrmsAc: %f\n", vrms_accumulator);
        deneme[35] = '\0';
        uart_puts(UART0_ID, deneme);
#endif

        vrms = sqrt(vrms_accumulator / VRMS_SAMPLE);
        vrms = vrms * 75;

#if DEBUG
        char x_array[40] = {0};
        snprintf(x_array, 31, "VRMS: %d\n\n", (uint16_t)vrms);
        x_array[32] = '\0';
        uart_puts(UART0_ID, x_array);
        printf("RTC Time:%s \r\n", datetime_str);
#endif

        vrms = vrms / 1000;

        vrms_buffer[vrms_buffer_count++] = (uint8_t)vrms;

        vrms_accumulator = 0.0;
        vrms = 0.0;

        if (set_time_flag)
        {
            remaining_time = pdMS_TO_TICKS((t.sec) * 1000);
            if (t.min % 15 == 0)
            {
                spi_write_buffer();
            }
            set_time_flag = 0;
        }

        if (t.min % 15 == 14 && t.sec > 55)
        {
            t.min++;
            t.sec = 0;
        }

        if ((t.sec < 5 && t.min % 15 == 0))
        {
            vrms_set_max_min_mean(vrms_buffer, vrms_buffer_count);
            spi_write_buffer();
            memset(vrms_buffer, 0, 15);
            vrms_buffer_count = 0;
        }

        executionTime = xTaskGetTickCount() - startTime;
        vTaskDelay(pdMS_TO_TICKS(VRMS_DATA_BUFFER_TIME) - executionTime - remaining_time);
#if DEBUG
// vTaskDelay(5000);
#endif
        remaining_time = 0;
    }
}

// DEBUG TASK
void vWriteDebugTask()
{
    TickType_t startTime;
    TickType_t executionTime;

    for (;;)
    {
        startTime = xTaskGetTickCount();
        get_time_pt7c4338(I2C_PORT, I2C_ADDRESS);
        datetime_to_str(datetime_str, sizeof(datetime_buf), &t);
        // printf("RTC Time:%s \r\n", datetime_str);
        executionTime = xTaskGetTickCount() - startTime;
        vTaskDelay(1000 - executionTime);
    }
}

// RESET TASK
void vResetTask()
{
    while (1)
    {
        gpio_put(RESET_PULSE_PIN, 1);
        vTaskDelay(10);
        gpio_put(RESET_PULSE_PIN, 0);
        vTaskDelay(INTERVAL_MS);
    }
}

void __not_in_flash_func(get_flash_contents)()
{
    uint32_t ints = save_and_disable_interrupts();
    sector_data = *(uint8_t *)flash_sector_contents;
    uint8_t *flash_target_contents = (uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET + (sector_data * FLASH_SECTOR_SIZE));
    memcpy(flash_data, flash_target_contents, FLASH_SECTOR_SIZE);
    restore_interrupts(ints);
}

void main()
{
    stdio_init_all();
    sleep_ms(3000);

    // UART INIT
    uart_init(UART0_ID, BAUD_RATE);
    initUART();
    gpio_set_function(UART0_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART0_RX_PIN, GPIO_FUNC_UART);

    // RESET INIT
    gpio_init(RESET_PULSE_PIN);
    gpio_set_dir(RESET_PULSE_PIN, GPIO_OUT);

    // ADC INIT
    adc_init();
    adc_gpio_init(27);
    adc_select_input(1);
    adc_set_clkdiv(4 * 9600);
    sleep_ms(1);

    // RTC Init
    rtc_init();
    rtc_set_datetime(&t);

    // I2C Init
    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(RTC_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(RTC_I2C_SCL_PIN, GPIO_FUNC_I2C);

    // FLASH CONTENTS
    get_flash_contents();
    // spi_write_buffer();

    // RTC
    // set_time_pt7c4338(I2C_PORT, I2C_ADDRESS, 10, 23, 20, 3, 23, 8, 23);
    get_time_pt7c4338(I2C_PORT, I2C_ADDRESS);
    datetime_to_str(datetime_str, sizeof(datetime_buf), &t);
    remaining_time = pdMS_TO_TICKS((t.sec) * 1000);

    xTaskCreate(vADCReadTask, "ADCReadTask", 128, NULL, 3, NULL);
    xTaskCreate(vUARTTask, "UARTTask", UART_TASK_STACK_SIZE, NULL, UART_TASK_PRIORITY, NULL);
    xTaskCreate(vWriteDebugTask, "WriteDebugTask", 128, NULL, 2, NULL);
    xTaskCreate(vResetTask, "ResetTask", 128, NULL, 1, NULL);
    vTaskStartScheduler();

    while (true)
        ;
}