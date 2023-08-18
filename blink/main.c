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
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE
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

// ADC VARIABLES
uint8_t vrms_max = 0;
uint8_t vrms_min = 0;
uint8_t vrms_mean = 0;
uint16_t sample_buf[VRMS_SAMPLE];
static uint8_t vrms_buffer_count = 0;
double vrms_accumulator = 0.0;
char deneme_str[20];
const float conversion_factor = 1000 * (3.3f / (1 << 12));
uint8_t vrms_buffer[VRMS_BUFFER_SIZE];
double vrms = 0.0;
TickType_t remaining_time;
uint8_t set_time_flag = 0;

// SPI VARIABLES
uint8_t *flash_sector_contents = (uint8_t *)(XIP_BASE + FLASH_DATA_COUNT_OFFSET);
uint16_t sector_buffer[FLASH_PAGE_SIZE / sizeof(uint16_t)] = {};
static uint16_t sector_data = 0; // 382 is last sector
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
struct FlashData flash_data[FLASH_SECTOR_SIZE / sizeof(struct FlashData)] = {};

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
int data_len = 0;
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

            for (uint8_t i = 0; i < size - 3; i++)
            {
                xor_result ^= data_buffer[i];
            }

            if (xor_result == data_buffer[size - 3])
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
void print_buf(const uint8_t *buf, size_t len)
{
    char element_str[2]; // initialized for print_buf, will be removed.

    for (size_t i = 0; i < len; ++i)
    {
        sprintf(element_str, "%02x", buf[i]);
        uart_puts(UART0_ID, element_str);
        if (i % 16 == 15)
            uart_putc(UART0_ID, '\n');
        else
            uart_putc(UART0_ID, ' ');
        if (i % 256 == 0)
            uart_puts(UART0_ID, "\n\n");
    }
}

void set_sector_data()
{
    sector_buffer[0] = sector_data;
    flash_range_erase(FLASH_DATA_COUNT_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_DATA_COUNT_OFFSET, (uint8_t *)sector_buffer, FLASH_PAGE_SIZE);
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

void spi_write_buffer()
{
    uint8_t *flash_start_content = (uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);
    uint8_t *flash_sector_content = (uint8_t *)(XIP_BASE + FLASH_DATA_COUNT_OFFSET);
    set_flash_data();
    // set_sector_data();

    flash_range_erase(FLASH_TARGET_OFFSET + (sector_data * FLASH_SECTOR_SIZE), FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_TARGET_OFFSET + (sector_data * FLASH_SECTOR_SIZE), (uint8_t *)flash_data, FLASH_SECTOR_SIZE);
    // print_buf(flash_sector_content, FLASH_PAGE_SIZE);
    // print_buf(flash_start_content, (2 * FLASH_PAGE_SIZE));
}

void search_data_into_flash()
{
    // zamanı küçük büyük olarak kontrol et
    uint8_t start_control_time = 0;
    uint8_t end_control_time = 0;
    uint8_t date_check;
    uint32_t st_idx;
    uint8_t start_time_flag = 0;
    uint32_t end_idx;
    uint8_t end_time_flag = 0;
    char uart_bcc_checked[100] = {0};
    char uart_string[100] = {0};

    uint8_t *flash_start_content = (uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);

    for (date_check = 0; date_check < 10; date_check += 2)
    {
        start_control_time = (start_time[date_check] - '0') * 10;
        start_control_time += (start_time[date_check + 1] - '0');

        end_control_time = (end_time[date_check] - '0') * 10;
        end_control_time += (end_time[date_check + 1] - '0');

        if (start_control_time < end_control_time)
        {
            break;
        }
        else
        {
            start_control_time = 0;
            end_control_time = 0;
        }
    }

    if (date_check == 10)
    {
        return;
    }

    for (uint32_t i = 0; i < FLASH_TOTAL_DATA_COUNT; i += 16)
    {
        uint8_t k;
        uint8_t l;
        for (k = 0; k < 10; k++)
        {
            if (flash_start_content[i + k] != start_time[k])
            {
                break;
            }
        }
        if (k == 10)
        {
            start_time_flag = 1;
            st_idx = i;
        }

        for (l = 0; l < 10; l++)
        {
            if (flash_start_content[i + l] != end_time[l])
            {
                break;
            }
        }
        if (l == 10)
        {
            end_time_flag = 1;
            end_idx = i;
        }
    }

    if (start_time_flag && end_time_flag)
    {
        uint8_t xor_result = 0x01;
        for (st_idx; st_idx <= end_idx; st_idx += 16)
        {
            char year[3] = {flash_start_content[st_idx], flash_start_content[st_idx + 1], 0x00};
            char month[3] = {flash_start_content[st_idx + 2], flash_start_content[st_idx + 3], 0x00};
            char day[3] = {flash_start_content[st_idx + 4], flash_start_content[st_idx + 5], 0x00};
            char hour[3] = {flash_start_content[st_idx + 6], flash_start_content[st_idx + 7], 0x00};
            char minute[3] = {flash_start_content[st_idx + 8], flash_start_content[st_idx + 9], 0x00};
            char second[3] = {flash_start_content[st_idx + 10], flash_start_content[st_idx + 11], 0x00};
            uint8_t max = flash_start_content[st_idx + 12];
            uint8_t min = flash_start_content[st_idx + 13];
            uint8_t mean = flash_start_content[st_idx + 14];

            if (st_idx == end_idx)
            {
                snprintf(uart_bcc_checked, 32, "(%s%s%s%s%s%s)(%03d,%03d,%03d)\r\n\r%c", year, month, day, hour, minute, second, max, min, mean, 0x03);
                xor_result = bcc_control_buffer(uart_bcc_checked, 31, xor_result);
            }
            else
            {
                snprintf(uart_bcc_checked, 30, "(%s%s%s%s%s%s)(%03d,%03d,%03d)\r\n", year, month, day, hour, minute, second, max, min, mean);
                xor_result = bcc_control_buffer(uart_bcc_checked, 29, xor_result);
            }

            if (st_idx == end_idx)
                snprintf(uart_string, 33, "(%s%s%s%s%s%s)(%03d,%03d,%03d)\r\n\r%c%c", year, month, day, hour, minute, second, max, min, mean, 0x03, xor_result);
            else
                snprintf(uart_string, 30, "(%s%s%s%s%s%s)(%03d,%03d,%03d)\r\n", year, month, day, hour, minute, second, max, min, mean);

            uart_puts(UART0_ID, uart_string);
        }
    }
    else
    {
        uart_putc(UART0_ID, 0x15);
    }
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
    vrms_mean = buffer_sum / buffer_size;
}

void state0_handler(uint8_t *buffer, uint8_t size)
{
    if ((buffer[0] == 0x2F) && (buffer[1] == 0x3F) && (buffer[size - 3] == 0x21) && (buffer[size - 2] == 0x0D) && (buffer[size - 1] == 0x0A))
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
                if (rxChar == '\n')
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

// ADC CONVERTER TASK
void vADCReadTask()
{
    TickType_t startTime;
    TickType_t executionTime;

    while (1)
    {
        startTime = xTaskGetTickCount();

        adc_capture(sample_buf, VRMS_SAMPLE);

        int sumSamples = 0;

        for (int i = 0; i < VRMS_SAMPLE; i++)
        {
            uint16_t production = (uint16_t)(sample_buf[i] * conversion_factor);
            sumSamples += production;
        }

        uint16_t mean = sumSamples / VRMS_SAMPLE;

        for (uint16_t i = 0; i < VRMS_SAMPLE; i++)
        {
            uint16_t production = (uint16_t)(sample_buf[i] * conversion_factor);
            vrms_accumulator += pow((production - mean), 2);
        }

        vrms = sqrt(vrms_accumulator / VRMS_SAMPLE);
        vrms = vrms * 73;
        vrms = vrms / 1000;

        vrms_buffer[vrms_buffer_count++] = (uint8_t)vrms;

        // printf("RTC Time:%s \r\n", datetime_str);
        // char x_array[20] = {30};
        // snprintf(x_array, 15, "vrms: %d\n", (int)vrms);
        // x_array[19] = '\0';
        // uart_puts(UART0_ID, x_array);

        vrms_accumulator = 0.0;

        if (set_time_flag)
        {
            remaining_time = pdMS_TO_TICKS((t.sec) * 1000);
            if (t.min % 15 == 0)
            {
                spi_write_buffer();
            }
            set_time_flag = 0;
        }

        if (t.sec == 0 || t.sec == 1)
        {
            if (t.min % 15 == 0)
            {
                vrms_set_max_min_mean(vrms_buffer, vrms_buffer_count);
                spi_write_buffer();
                vrms_buffer_count = 0;
            }
        }

        executionTime = xTaskGetTickCount() - startTime;
        vTaskDelay(pdMS_TO_TICKS(VRMS_DATA_BUFFER_TIME) - executionTime - remaining_time);
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

void main()
{
    stdio_init_all();
    sleep_ms(1);

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
    adc_set_clkdiv(9600);
    sleep_ms(1);

    // RTC Init
    rtc_init();
    rtc_set_datetime(&t);

    // I2C Init
    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(RTC_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(RTC_I2C_SCL_PIN, GPIO_FUNC_I2C);

    // RTC
    get_time_pt7c4338(I2C_PORT, I2C_ADDRESS);
    datetime_to_str(datetime_str, sizeof(datetime_buf), &t);
    remaining_time = pdMS_TO_TICKS(t.sec * 1000);

    // FLASH CONTENTS
    sector_data = *(uint8_t *)flash_sector_contents;
    uint8_t *flash_target_contents = (uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET + (sector_data * FLASH_SECTOR_SIZE));
    memcpy(flash_data, flash_target_contents, FLASH_SECTOR_SIZE);

    // xTaskCreate(vBlinkTask, "BlinkTask", 128, NULL, 1, NULL);
    xTaskCreate(vADCReadTask, "ADCReadTask", 128, NULL, 3, NULL);
    xTaskCreate(vUARTTask, "UARTTask", UART_TASK_STACK_SIZE, NULL, UART_TASK_PRIORITY, NULL);
    xTaskCreate(vWriteDebugTask, "WriteDebugTask", 128, NULL, 2, NULL);
    xTaskCreate(vResetTask, "ResetTask", 128, NULL, 1, NULL);
    vTaskStartScheduler();

    while (true)
        ;
}