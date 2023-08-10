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

// SPI DEFINES
#define FLASH_DATA_COUNT_OFFSET 512 * 1024
#define FLASH_TOTAL_SECTORS 382
#define FLASH_TARGET_OFFSET (512 * 1024) + FLASH_SECTOR_SIZE
#define FLASH_DATA_SIZE 16
#define FLASH_TOTAL_DATA_COUNT (PICO_FLASH_SIZE_BYTES - (FLASH_TARGET_OFFSET)) / FLASH_DATA_SIZE

// UART DEFINES
#define UART0_ID uart0 // UART0 for RS485
#define BAUD_RATE 115200
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
    // char date[12];
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
    printf("UART_INIT TASK\r\n");
    uart_set_format(UART0_ID, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART0_ID, true);
    int UART_IRQ = UART0_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(UART_IRQ, UART_Isr);
    irq_set_enabled(UART_IRQ, true);
    uart_set_translate_crlf(UART0_ID, true);
}

bool check_uart_data(uint8_t *data_buffer, uint8_t size)
{
    uint8_t xor_result = 0x01;

    for (uint8_t i = 0; i < size - 1; i++)
    {
        xor_result ^= data_buffer[i];
    }

    printf("\nxor result in check_uart_data: %02X", xor_result);
    printf("\nxor result in received data: %02X", data_buffer[size - 1]);

    return xor_result == data_buffer[size - 1];
}

uint8_t start_time[10];
uint8_t end_time[10];

void parse_uart_data(uint8_t *buffer)
{
    for (uint i = 0; buffer[i] != '\0'; i++)
    {
        // left bracket control
        if (buffer[i] == 0x28)
        {
            uint8_t k;
            // comma control
            for (k = i + 1; buffer[k] != 0x3B; k++)
            {
                start_time[k - (i + 1)] = buffer[k];
            }
            // right bracket control
            for (uint8_t l = k + 1; buffer[l] != 0x29; l++)
            {
                end_time[l - (k + 1)] = buffer[l];
            }
            break;
        }
    }

    printf("\nstart time: %s\n", start_time);
    printf("\nend time: %s\n", end_time);
}

// RTC FUNCTIONS
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

void set_time_pt7c4338(struct i2c_inst *i2c, uint8_t address, uint8_t seconds, uint8_t minutes, uint8_t hours, uint8_t day, uint8_t date, uint8_t month, uint8_t year)
{
    uint8_t buf[8];
    buf[0] = PT7C4338_REG_SECONDS;
    buf[1] = seconds;
    buf[2] = minutes;
    buf[3] = hours;
    buf[4] = day;
    buf[5] = date;
    buf[6] = month;
    buf[7] = year;

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
        // printf("%s", element_str);
        uart_puts(UART0_ID, element_str);
        if (i % 16 == 15)
            // printf("\n");
            uart_putc(UART0_ID, '\n');
        else
            // printf(" ");
            uart_putc(UART0_ID, ' ');

        if (i % 256 == 0)
            // printf("\n\n");
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

    // datalar dizinin başına yazıldığı için gelen istek formatıyla uymuyor
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
        // Last sector control
        if (sector_data == FLASH_TOTAL_SECTORS)
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
    set_flash_data();
    // set_sector_data();

    uart_puts(UART0_ID, "\nSECTOR PAGE: \n");
    print_buf(flash_sector_contents, FLASH_PAGE_SIZE);

    uart_puts(UART0_ID, "\nERASING: \n");
    flash_range_erase(FLASH_TARGET_OFFSET + (sector_data * FLASH_SECTOR_SIZE), FLASH_SECTOR_SIZE);
    uart_puts(UART0_ID, "\nERASED: \n");

    uart_puts(UART0_ID, "\nERASED CONTENT: \n");
    print_buf(flash_start_content, (2 * FLASH_PAGE_SIZE));

    uart_puts(UART0_ID, "\nPROGRAMMING: \n");
    flash_range_program(FLASH_TARGET_OFFSET + (sector_data * FLASH_SECTOR_SIZE), (uint8_t *)flash_data, FLASH_SECTOR_SIZE);
    uart_puts(UART0_ID, "\nPROGRAMMED: \n");

    uart_puts(UART0_ID, "\nWRITTEN CONTENT: \n");
    print_buf(flash_start_content, (2 * FLASH_PAGE_SIZE));
}

void search_data_into_flash()
{
    uint8_t *flash_start_content = (uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);
    char uart_string[256];

    for (uint32_t i = 0; i < FLASH_TOTAL_DATA_COUNT; i += 16)
    {
        uint8_t k;
        for (k = 0; k < 10; k++)
        {
            if (flash_start_content[i + k] != start_time[k])
            {
                break;
            }
        }

        if (k == 10)
        {
            // string size will be changed
            for (int l = i; i < FLASH_TOTAL_DATA_COUNT; l += 16)
            {
                char year[2] = {flash_start_content[l], flash_start_content[l + 1]};
                char month[2] = {flash_start_content[l + 2], flash_start_content[l + 3]};
                char day[2] = {flash_start_content[l + 4], flash_start_content[l + 5]};
                char hour[2] = {flash_start_content[l + 6], flash_start_content[l + 7]};
                char minute[2] = {flash_start_content[l + 8], flash_start_content[l + 9]};
                char second[2] = {flash_start_content[l + 10], flash_start_content[l + 11]};
                uint8_t max = flash_start_content[l + 12];
                uint8_t min = flash_start_content[l + 13];
                uint8_t mean = flash_start_content[l + 14];
                sprintf(uart_string, "\nTime: %s/%s/%s %s:%s:%s max:%d,min:%d,mean:%d\n", year, month, day, hour, minute, second, max, min, mean);
                uart_puts(UART0_ID, uart_string);
                int t;
                for (t = 0; t < 10; t++)
                {
                    if (flash_start_content[l + t] != end_time[t])
                    {
                        break;
                    }
                }

                if (t == 10)
                {
                    break;
                }
            }
            break;
        }
    }
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

// UART TASK
void vUARTTask(void *pvParameters)
{
    (void)pvParameters;
    uint32_t ulNotificationValue;
    xTaskToNotify_UART = NULL;
    int data_len = 0;
    uint8_t rxBuffer[256] = {};
    uint8_t rxChar;

    while (true)
    {
        UART_receive();
        ulNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (ulNotificationValue == 1)
        {
            while (uart_is_readable(UART0_ID))
            {
                rxChar = getchar_timeout_us(10);

                // ENTER CONTROL
                if (rxChar != 13)
                {
                    rxBuffer[data_len++] = rxChar;
                }
                if (rxChar == 13)
                {
                    rxBuffer[data_len] = '\0';
                    printf("\nreceived data: ");

                    for (int i = 0; i < data_len; i++)
                    {
                        printf("%02X ", rxBuffer[i]);
                    }

                    printf("\nentering check_uart_data.\n");

                    if (check_uart_data(rxBuffer, data_len))
                    {
                        printf("\nentered true area:");
                        printf("\nwriting rxBuffer:");
                        for (int i = 0; i < data_len; i++)
                        {
                            printf("%02X ", rxBuffer[i]);
                        }
                        parse_uart_data(rxBuffer);
                        search_data_into_flash();
                    }
                    else
                    {
                        printf("\nentered falsae area:\n");
                        for (int i = 0; i < data_len; i++)
                        {
                            printf("%02X ", rxBuffer[i]);
                        }
                        uart_puts(UART0_ID, "\nDATA IS WRONG!\n");
                    }
                    memset(rxBuffer, 0, 256);
                    printf("\nnewline\n");
                    data_len = 0;
                }
            }
        }
    }
}

// BLINK TASK
void vBlinkTask()
{
    for (;;)
    {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        vTaskDelay(10);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        vTaskDelay(600);
    }
}

static uint8_t vrms_buffer_count = 0;
double vrms_accumulator = 0.0;
char deneme_str[20];
const float conversion_factor = 1000 * (3.3f / (1 << 12));
uint8_t vrms_buffer[VRMS_BUFFER_SIZE];
double vrms;

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
        sprintf(deneme_str, "mean:%d\n", mean);
        uart_puts(UART0_ID, deneme_str);

        for (uint16_t i = 0; i < VRMS_SAMPLE; i++)
        {
            uint16_t production = (uint16_t)(sample_buf[i] * conversion_factor);
            vrms_accumulator += pow((production - mean), 2);
        }

        vrms = sqrt(vrms_accumulator / VRMS_SAMPLE);
        vrms = vrms * 65.57;
        vrms = vrms / 1000;

        sprintf(deneme_str, "vrms%f\n", vrms);
        uart_puts(UART0_ID, deneme_str);

        vrms_buffer[vrms_buffer_count++] = (uint8_t)vrms;

        if (t.min % 15 == 0)
        {
            vrms_set_max_min_mean(vrms_buffer, vrms_buffer_count);
            spi_write_buffer();
            vrms_buffer_count = 0;
        }

        executionTime = xTaskGetTickCount() - startTime;
        vTaskDelay(VRMS_DATA_BUFFER_TIME - executionTime);
    }
}

// DEBUG TASK
void vWriteDebugTask()
{
    for (;;)
    {
        get_time_pt7c4338(I2C_PORT, I2C_ADDRESS);
        // rtc_get_datetime(&t);    This function uses rp2040's rtc
        datetime_to_str(datetime_str, sizeof(datetime_buf), &t);
        // printf("RTC Time:%s \r\n", datetime_str);
        vTaskDelay(1000);
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
    uart_puts(UART0_ID, "Hello, UART0\n");

    // RESET INIT
    gpio_init(RESET_PULSE_PIN);
    gpio_set_dir(RESET_PULSE_PIN, GPIO_OUT);

    // ADC INIT
    adc_init();
    adc_gpio_init(27);
    adc_select_input(1);
    adc_set_clkdiv(9600);
    sleep_ms(1000);

    // RTC Init
    rtc_init();
    rtc_set_datetime(&t);

    // I2C Init
    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(RTC_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(RTC_I2C_SCL_PIN, GPIO_FUNC_I2C);

    // SET TIMER TO RTC
    // set_time_pt7c4338(I2C_PORT, I2C_ADDRESS, 0x00, 0x35, 0x16, 0x03, 0x09, 0x08, 0x23);

    // FLASH CONTENTS
    sector_data = *(uint16_t *)flash_sector_contents;
    uint8_t *flash_target_contents = (uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET + (sector_data * FLASH_SECTOR_SIZE));
    memcpy(flash_data, flash_target_contents, FLASH_SECTOR_SIZE);

    xTaskCreate(vBlinkTask, "BlinkTask", 128, NULL, 1, NULL);
    xTaskCreate(vADCReadTask, "ADCReadTask", 128, NULL, 1, NULL);
    xTaskCreate(vUARTTask, "UARTTask", UART_TASK_STACK_SIZE, NULL, UART_TASK_PRIORITY, NULL);
    xTaskCreate(vWriteDebugTask, "WriteDebugTask", 128, NULL, 1, NULL);
    xTaskCreate(vResetTask, "ResetTask", 128, NULL, 1, NULL);
    vTaskStartScheduler();

    while (true)
    {
    };
}