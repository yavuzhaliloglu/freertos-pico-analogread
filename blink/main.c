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
#define FLASH_SECTOR_OFFSET 512 * 1024
#define FLASH_TOTAL_SECTORS 382
#define FLASH_DATA_OFFSET (512 * 1024) + FLASH_SECTOR_SIZE
#define FLASH_RECORD_SIZE 16
#define FLASH_TOTAL_RECORDS (PICO_FLASH_SIZE_BYTES - (FLASH_DATA_OFFSET)) / FLASH_RECORD_SIZE

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
#define RTC_SET_PERIOD_MIN 10

// RESET PIN DEFINE
#define RESET_PULSE_PIN 2
#define INTERVAL_MS 60000

// ADC DEFINES
#define VRMS_SAMPLE 500
#define VRMS_BUFFER_SIZE 15
#define VRMS_SAMPLING_PERIOD 60000
#define CLOCK_DIV 4 * 9600
#define DEBUG 0

// ADC VARIABLES
uint8_t vrms_max = 0;
uint8_t vrms_min = 0;
uint8_t vrms_mean = 0;
uint16_t sample_buffer[VRMS_SAMPLE];
TickType_t adc_remaining_time = 0;
datetime_t alarm = {
    .year = -1,
    .month = -1,
    .day = -1,
    .dotw = -1,
    .hour = -1,
    .min = -1,
    .sec = 00};

// SPI VARIABLES
uint8_t *flash_sector_content = (uint8_t *)(XIP_BASE + FLASH_SECTOR_OFFSET);
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
char datetime_buffer[64];
char *datetime_str = &datetime_buffer[0];
datetime_t current_time = {
    .year = 2020,
    .month = 06,
    .day = 05,
    .dotw = 5, // 0 is Sunday, so 5 is Friday
    .hour = 15,
    .min = 45,
    .sec = 00};

// UART VARIABLES
enum States
{
    Greeting = 0,
    Setting = 1,
    Listening = 2
};
enum ListeningStates
{
    DataError = -1,
    Reading = 0,
    TimeSet = 1,
    DateSet = 2
};
volatile TaskHandle_t xTaskToNotify_UART = NULL;
enum States state = Greeting;
uint16_t max_baud_rate = 9600;
uint8_t rx_buffer[256] = {};
uint8_t rx_buffer_len = 0;
uint8_t reading_state_start_time[10];
uint8_t reading_state_end_time[10];
volatile uint8_t test_flag = 0;

// RTC FUNCTIONS

uint8_t decimalToBCD(uint8_t decimalValue)
{
    return ((decimalValue / 10) << 4) | (decimalValue % 10);
}

uint8_t bcd_to_decimal(uint8_t bcd)
{
    return bcd - 6 * (bcd >> 4);
}

void setTimePt7c4338(struct i2c_inst *i2c, uint8_t address, uint8_t seconds, uint8_t minutes, uint8_t hours, uint8_t day, uint8_t date, uint8_t month, uint8_t year)
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

datetime_t getTimePt7c4338(datetime_t *dt)
{
    uint8_t buffer[7] = {PT7C4338_REG_SECONDS};

    i2c_write_blocking(I2C_PORT, I2C_ADDRESS, buffer, 1, 1);
    i2c_read_blocking(I2C_PORT, I2C_ADDRESS, buffer, 7, 0);

    dt->year = bcd_to_decimal(buffer[6]);
    dt->month = bcd_to_decimal(buffer[5]);
    dt->day = bcd_to_decimal(buffer[4]);
    dt->dotw = buffer[3];
    dt->hour = bcd_to_decimal(buffer[2]);
    dt->min = bcd_to_decimal(buffer[1]);
    dt->sec = bcd_to_decimal(buffer[0]);
}

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

uint8_t bccCreate(uint8_t *data_buffer, uint8_t size, uint8_t xor)
{
    for (uint8_t i = 0; i < size; i++)
        xor ^= data_buffer[i];

    return xor;
}

bool bccControl(uint8_t *buffer, uint8_t size)
{
    uint8_t xor_result = 0x01;

    for (uint8_t i = 0; i < size; i++)
        xor_result ^= buffer[i];

    return xor_result == buffer[size - 1];
}

enum ListeningStates checkListeningData(uint8_t *data_buffer, uint8_t size)
{
    if (!(bccControl(data_buffer, size)))
        return DataError;

    // Default Control ([SOH]R2[STX])
    if ((data_buffer[0] == 0x01) && (data_buffer[1] == 0x52) && (data_buffer[2] == 0x32) && (data_buffer[3] == 0x02))
    {
        // Reading Control (P.01)
        if ((data_buffer[4] == 0x50) && (data_buffer[5] == 0x2E) && (data_buffer[6] == 0x30) && (data_buffer[7] == 0x31))
            return Reading;

        // Time Set Control (0.9.1)
        if ((data_buffer[4] == 0x30) && (data_buffer[6] == 0x39) && (data_buffer[8] == 0x31))
            return TimeSet;

        // Date Set Control (0.9.2)
        if ((data_buffer[4] == 0x30) && (data_buffer[6] == 0x39) && (data_buffer[8] == 0x32))
            return DateSet;
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

void resetRxBuffer()
{
    memset(rx_buffer, 0, 256);
    rx_buffer_len = 0;
}

void resetState()
{
    state = Greeting;
    memset(rx_buffer, 0, 256);
    setProgramBaudRate(0);
    rx_buffer_len = 0;
}

void greetingStateHandler(uint8_t *buffer, uint8_t size)
{
    if ((buffer[0] == 0x2F) && (buffer[1] == 0x3F) && (buffer[2] == 0x36) && (buffer[3] == 0x31) && (buffer[size - 3] == 0x21) && (buffer[size - 2] == 0x0D) && (buffer[size - 1] == 0x0A))
    {
        uint8_t program_baud_rate = getProgramBaudRate(max_baud_rate);
        char greeting_uart_buffer[19];

        snprintf(greeting_uart_buffer, 17, "/ALP%dMAVIALPV2\r\n", program_baud_rate);
        greeting_uart_buffer[18] = '\0';
        uart_puts(UART0_ID, greeting_uart_buffer);

        state = Setting;
    }
}

void settingStateHandler(uint8_t *buffer, uint8_t size)
{
    if ((buffer[0] == 0x06) && (buffer[size - 2] == 0x0D) && (buffer[size - 1] == 0x0A) && size == 6)
    {
        uint8_t modem_baud_rate;
        uint8_t i;
        uint8_t program_baud_rate = getProgramBaudRate(max_baud_rate);

        for (i = 0; i < size; i++)
        {
            if (buffer[i] == '0')
            {
                modem_baud_rate = buffer[i + 1] - '0';
                if ((buffer[i + 2] != '1') && modem_baud_rate > 5 && modem_baud_rate < 0)
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
                    uart_putc(UART0_ID, 0x06);
                    setProgramBaudRate(modem_baud_rate);
                    state = Listening;
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

    setTimePt7c4338(I2C_PORT, I2C_ADDRESS, sec, min, hour, (uint8_t)current_time.dotw, (uint8_t)current_time.day, (uint8_t)current_time.month, (uint8_t)current_time.year);
    getTimePt7c4338(&current_time);
    rtc_set_datetime(&current_time);
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

    setTimePt7c4338(I2C_PORT, I2C_ADDRESS, (uint8_t)current_time.sec, (uint8_t)current_time.min, (uint8_t)current_time.hour, (uint8_t)current_time.dotw, day, month, year);
    getTimePt7c4338(&current_time);
    rtc_set_datetime(&current_time);
}

bool controlRXBuffer(uint8_t *buffer,uint8_t len)
{
    uint8_t reading[8] = {0x01, 0x52, 0x32, 0x02, 0x50, 0x2E, 0x30, 0x31};
    uint8_t time[9] = {0x01, 0x52, 0x32, 0x02, 0x30, 0x2E, 0x39, 0x2E, 0x31};
    uint8_t date[9] = {0x01, 0x52, 0x32, 0x02, 0x30, 0x2E, 0x39, 0x2E, 0x32};

    uint8_t reading_len = 33;
    uint8_t time_len = 19;
    uint8_t date_len = 19;

    if((len == reading_len) && (strncmp(buffer,reading,8) == 0))
        return 1;
    if((len == time_len) && (strncmp(buffer,time,9)==0))
        return 1;
    if((len == date_len) && (strncmp(buffer,date,9)==0))
        return 1;
    return 0;
}

// SPI FUNCTIONS

void __not_in_flash_func(getFlashContents)()
{
    uint32_t ints = save_and_disable_interrupts();
    sector_data = *(uint8_t *)flash_sector_content;
    uint8_t *flash_target_contents = (uint8_t *)(XIP_BASE + FLASH_DATA_OFFSET + (sector_data * FLASH_SECTOR_SIZE));
    memcpy(flash_data, flash_target_contents, FLASH_SECTOR_SIZE);
    restore_interrupts(ints);
}

void printBufferHex(uint8_t *buf, size_t len)
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

void __not_in_flash_func(setSectorData)()
{
    uint8_t sector_buffer[FLASH_SECTOR_SIZE / sizeof(uint8_t)] = {0};
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

    arrayToDatetime(&start, reading_state_start_time);
    arrayToDatetime(&end, reading_state_end_time);

    if (datetimeComp(&start, &end) > 0)
        return;

    datetime_t dt_start = {0};
    datetime_t dt_end = {0};
    int32_t start_index = -1;
    int32_t end_index = -1;

    char uart_bcc_checked[100] = {0};
    char uart_string[100] = {0};

    uint8_t *flash_start_content = (uint8_t *)(XIP_BASE + FLASH_DATA_OFFSET);

    for (uint32_t i = 0; i < FLASH_TOTAL_RECORDS; i += 16)
    {
        datetime_t rec_time = {0};

        if (flash_start_content[i] == 0xFF || flash_start_content[i] == 0x00)
            continue;

        arrayToDatetime(&rec_time, &flash_start_content[i]);

        if (datetimeComp(&rec_time, &start) >= 0)
        {
            if (start_index == -1 || (datetimeComp(&rec_time, &dt_start) < 0))
            {
                start_index = i;
                datetimeCopy(&rec_time, &dt_start);
            }
        }

        if (datetimeComp(&rec_time, &end) <= 0)
        {
            if (end_index == -1 || datetimeComp(&rec_time, &dt_end) > 0)
            {
                end_index = i;
                datetimeCopy(&rec_time, &dt_end);
            }
        }
    }

    if (start_index >= 0 && end_index >= 0)
    {
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
                xor_result = bccCreate(uart_bcc_checked, 29, xor_result);
            }
            else
            {
                snprintf(uart_bcc_checked, 30, "(%s%s%s%s%s)(%03d,%03d,%03d)\r\n", year, month, day, hour, minute, min, max, mean);
                xor_result = bccCreate(uart_bcc_checked, 27, xor_result);
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
        uart_putc(UART0_ID, 0x15);
    }
    memset(reading_state_start_time, 0, 10);
    memset(reading_state_end_time, 0, 10);
}

// ADC FUNCTIONS
void __not_in_flash_func(adcCapture)(uint16_t *buf, size_t count)
{
    adc_fifo_setup(true, false, 0, false, false);
    adc_run(true);

    for (int i = 0; i < count; i = i + 1)
        buf[i] = adc_fifo_get_blocking();

    adc_run(false);
    adc_fifo_drain();
}

// UART TASK
void vUARTTask(void *pvParameters)
{
    (void)pvParameters;
    uint32_t ulNotificationValue;
    xTaskToNotify_UART = NULL;
    uint8_t rx_char;
    uint8_t rx_control_buffer[10];
    TimerHandle_t ResetBufferTimer = xTimerCreate(
        "BufferTimer",
        pdMS_TO_TICKS(5000),
        pdFALSE,
        NULL,
        resetRxBuffer);

    TimerHandle_t ResetStateTimer = xTimerCreate(
        "StateTimer",
        pdMS_TO_TICKS(30000),
        pdTRUE,
        NULL,
        resetState);

    while (true)
    {
        xTimerStart(ResetBufferTimer, 0);
        UARTReceive();
        ulNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (ulNotificationValue == 1)
        {
            while (uart_is_readable(UART0_ID))
            {
                rx_char = uart_getc(UART0_ID);

                // ENTER CONTROL
                if (rx_char != '\n')
                {
                    xTimerReset(ResetBufferTimer, 0);
                    rx_buffer[rx_buffer_len++] = rx_char;
                }
                // (rx_buffer_len == 33 && ((rx_buffer[4] == 0x50) && (rx_buffer[5] == 0x2E) && (rx_buffer[6] == 0x30) && (rx_buffer[7] == 0x31))) || (rx_buffer_len == 19 && ((rx_buffer[4] == 0x30) && (rx_buffer[6] == 0x39) && (rx_buffer[8] == 0x31))) || (rx_buffer_len == 19 && ((rx_buffer[4] == 0x30) && (rx_buffer[6] == 0x39) && (rx_buffer[8] == 0x32)))
                if (rx_char == '\n' || controlRXBuffer(rx_buffer,rx_buffer_len))
                {
                    rx_buffer[rx_buffer_len++] = rx_char;
                    xTimerReset(ResetStateTimer, 0);
                    xTimerStop(ResetBufferTimer, 0);

                    switch (state)
                    {
                    case Greeting:
                        xTimerStart(ResetStateTimer, 0);
                        greetingStateHandler(rx_buffer, rx_buffer_len);
                        break;

                    case Setting:
                        xTimerStart(ResetStateTimer, 0);
                        settingStateHandler(rx_buffer, rx_buffer_len);
                        break;

                    case Listening:
                        xTimerStart(ResetStateTimer, 0);
                        switch (checkListeningData(rx_buffer, rx_buffer_len))
                        {
                        case DataError:
                            uart_putc(UART0_ID, 0x15);
                            break;

                        case Reading:
                            parseReadingData(rx_buffer);
                            searchDataInFlash();
                            break;

                        case TimeSet:
                            setTimeFromUART(rx_buffer);
                            break;

                        case DateSet:
                            setDateFromUART(rx_buffer);
                            break;

                        default:
                            uart_putc(UART0_ID, 0x15);
                            break;
                        }
                        break;
                    default:
                        uart_putc(UART0_ID, 0x15);
                        break;
                    }
                    memset(rx_buffer, 0, 256);
                    rx_buffer_len = 0;
                }
            }
        }
    }
}

// ADC CONVERTER TASK

static void adc_read(void)
{
    rtc_get_datetime(&current_time);
    datetime_to_str(datetime_str, sizeof(datetime_buffer), &current_time);
    printf("Alarm Fired At %s\n", datetime_str);
    stdio_flush();

    uint8_t vrms_buffer_count = 0;
    double vrms_accumulator = 0.0;
    const float conversion_factor = 1000 * (3.3f / (1 << 12));
    uint8_t vrms_buffer[VRMS_BUFFER_SIZE] = {0};
    double vrms = 0.0;

    adcCapture(sample_buffer, VRMS_SAMPLE);

    double mean = 2050 * conversion_factor;

    for (uint16_t i = 0; i < VRMS_SAMPLE; i++)
    {
        double production = (double)(sample_buffer[i] * conversion_factor);
        vrms_accumulator += pow((production - mean), 2);
    }

    vrms = sqrt(vrms_accumulator / VRMS_SAMPLE);
    vrms = vrms * 75;
    vrms = vrms / 1000;

    if (vrms < 0)
        vrms = 0.0;

    vrms_buffer[(vrms_buffer_count++) % 15] = (uint8_t)vrms;

    if ((current_time.sec < 5 && current_time.min % 15 == 0))
    {
        vrmsSetMinMaxMean(vrms_buffer, vrms_buffer_count);
        SPIWriteToFlash();
        memset(vrms_buffer, 0, 15);
        vrms_buffer_count = 0;
    }
}

// DEBUG TASK

void vWriteDebugTask()
{
    TickType_t startTime = xTaskGetTickCount();

    for (;;)
    {
        rtc_get_datetime(&current_time);
        datetime_to_str(datetime_str, sizeof(datetime_buffer), &current_time);
        // printf("RTC Time:%s \r\n", datetime_str);
        vTaskDelayUntil(&startTime, pdMS_TO_TICKS(1000));
    }
}

void vRTCTask()
{
    TickType_t startTime = xTaskGetTickCount();

    while (true)
    {
        vTaskDelayUntil(&startTime, RTC_SET_PERIOD_MIN * 60 * 1000);
        getTimePt7c4338(&current_time);
        rtc_set_datetime(&current_time);
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

void main()
{
    stdio_init_all();
    sleep_ms(1000);

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
    adc_set_clkdiv(CLOCK_DIV);
    sleep_ms(1);

    // RTC Init
    rtc_init();

    // I2C Init
    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(RTC_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(RTC_I2C_SCL_PIN, GPIO_FUNC_I2C);

    // FLASH CONTENTS
    getFlashContents();
    // SPIWriteToFlash();

    // RTC
    // setTimePt7c4338(I2C_PORT, I2C_ADDRESS, 10, 23, 20, 3, 23, 8, 23);
    getTimePt7c4338(&current_time);
    rtc_set_datetime(&current_time);
    sleep_us(64);
    rtc_set_alarm(&alarm, &adc_read);

    xTaskCreate(vUARTTask, "UARTTask", UART_TASK_STACK_SIZE, NULL, UART_TASK_PRIORITY, NULL);
    xTaskCreate(vWriteDebugTask, "WriteDebugTask", 128, NULL, 2, NULL);
    xTaskCreate(vRTCTask, "RTCTask", 128, NULL, 2, NULL);
    xTaskCreate(vResetTask, "ResetTask", 128, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true)
        ;
}
