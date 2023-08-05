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
#include "hardware/flash.h"
#include "hardware/sync.h"
#include <string.h>
#include "pico/bootrom.h"

// SPI DEFINES
#define FLASH_DATA_COUNT_OFFSET 512 * 1024
#define FLASH_TARGET_OFFSET (512 * 1024) + FLASH_SECTOR_SIZE
#define FLASH_TOTAL_SECTORS ((PICO_FLASH_SIZE_BYTES - FLASH_TARGET_OFFSET) / FLASH_SECTOR_SIZE) - 1
#define FLASH_DATA_SIZE 9

// UART DEFINES
#define UART0_ID uart0 // UART0 for RS485
#define BAUD_RATE 115200
#define UART0_TX_PIN 0
#define UART0_RX_PIN 1
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE
#define UART_TASK_PRIORITY (2)
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
#define DATETIME_SIZE 42
#define VRMS_BUFFER_SIZE 15
#define VRMS_DATA_BUFFER_TIME 4000
#define VRMS_DATA_FLASH_TIME VRMS_DATA_SIZE *VRMS_DATA_BUFFER_TIME

// ADC VARIABLES
double vrms;
volatile double vrms_accumulator = 0.0;
uint8_t max_vrms = 2;
uint8_t min_vrms = 10;
uint16_t vrms_buffer[VRMS_BUFFER_SIZE];

// SPI VARIABLES
uint8_t *flash_sector_contents = (uint8_t *)(XIP_BASE + FLASH_DATA_COUNT_OFFSET);
char element_str[2]; // initialized for print_buf, will be removed.
char sector_data_str[2];
uint8_t flash_data[FLASH_SECTOR_SIZE] = {};
static uint16_t sector_data = 0; // 382 is last sector
uint16_t sector_buffer[FLASH_PAGE_SIZE / sizeof(uint16_t)];

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
uint8_t rxChar;

// UART FUNCTIONS
void UART_receive()
{
    printf("UART_RECEIVE TASK\r\n");
    configASSERT(xTaskToNotify_UART == NULL);
    xTaskToNotify_UART = xTaskGetCurrentTaskHandle();
    uart_set_irq_enables(UART0_ID, true, false);
}

void UART_Isr()
{
    printf("UART_ISR TASK\r\n");
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
    uart_set_fifo_enabled(UART0_ID, false);
    int UART_IRQ = UART0_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(UART_IRQ, UART_Isr);
    irq_set_enabled(UART_IRQ, true);
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
// SPI FUNCTION
void print_buf(const uint8_t *buf, size_t len)
{
    for (size_t i = 0; i < len; ++i)
    {
        sprintf(element_str, "%02x", buf[i]);
        uart_puts(UART0_ID, element_str);
        if (i % 16 == 15)
            uart_puts(UART0_ID, "\n");
        else
            uart_puts(UART0_ID, " ");
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

void set_flash_data()
{
    uint8_t *flash_target_contents = (uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET + (sector_data * FLASH_SECTOR_SIZE));
    uint16_t i;

    // TO-DO:
    // flasha yazma işleminde yazamadan önce silinip güç kesildiğinde tüm bitler ff kalıyordu ve yazma işlemi yapmıyordu.
    // şuan yazma işlemi yapıyor fakat sektörü silip en başından yapıyor ve veri kaybına sebep oluyor.

    for (i = 0; i < FLASH_SECTOR_SIZE; i += 8)
    {

        if (flash_target_contents[i] == '\0' || flash_target_contents[i] == 0xff)
        {
            flash_data[i] = t.day;
            flash_data[i + 1] = t.month;
            flash_data[i + 2] = t.year;
            flash_data[i + 3] = t.hour;
            flash_data[i + 4] = t.min;
            flash_data[i + 5] = t.sec;
            flash_data[i + 6] = max_vrms;
            flash_data[i + 7] = min_vrms;
            break;
        }
    }

    if (i == FLASH_SECTOR_SIZE)
    {
        if (sector_data == 382)
        {
            sector_data = 0;
        }
        else
        {
            sector_data++;
        }
        memset(flash_data, 0, FLASH_SECTOR_SIZE);
        flash_data[0] = t.day;
        flash_data[1] = t.month;
        flash_data[2] = t.year;
        flash_data[3] = t.hour;
        flash_data[4] = t.min;
        flash_data[5] = t.sec;
        flash_data[6] = max_vrms;
        flash_data[7] = min_vrms;
        set_sector_data();
    }
}

void reset_flash()
{
    sector_data = 0;
    memset(flash_data, 0, FLASH_SECTOR_SIZE);
    set_sector_data();
    flash_range_erase(FLASH_TARGET_OFFSET, (FLASH_TOTAL_SECTORS * FLASH_SECTOR_SIZE) - FLASH_PAGE_SIZE);
}

// SPI WRITE FUNCTION
void spi_write_buffer()
{

    // uint8_t *flash_end_content = (uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET + (382 * FLASH_SECTOR_SIZE) + (14 * FLASH_PAGE_SIZE));
    uint8_t *flash_start_content = (uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET + (14 * FLASH_PAGE_SIZE));
    set_flash_data();
    vTaskDelay(10);
    // set_sector_data();

    printf("\nSECTOR PAGE: \n");
    print_buf(flash_sector_contents, FLASH_PAGE_SIZE);

    printf("\nerasing...\n");
    flash_range_erase(FLASH_TARGET_OFFSET + (sector_data * FLASH_SECTOR_SIZE), FLASH_SECTOR_SIZE);
    printf("DATA PAGE erased\n");
    vTaskDelay(10);

    printf("\nerased content:\n");
    print_buf(flash_start_content, (4 * FLASH_PAGE_SIZE));

    // printf("\nFIRST 2 PAGES:\n");
    // print_buf(flash_start_content, (2 * FLASH_PAGE_SIZE));
    // printf("\nLAST 2 PAGES:\n");
    // print_buf(flash_end_content, FLASH_PAGE_SIZE * 2);

    printf("\nprogramming...\n");
    flash_range_program(FLASH_TARGET_OFFSET + (sector_data * FLASH_SECTOR_SIZE), (uint8_t *)flash_data, FLASH_SECTOR_SIZE);
    printf("DATA PAGE programmed.\n");

    printf("\nwrited content:\n");
    print_buf(flash_start_content, (4 * FLASH_PAGE_SIZE));

    // printf("\nFIRST 2 PAGES:\n");
    // print_buf(flash_start_content, (2 * FLASH_PAGE_SIZE));
    // printf("\nLAST 2 PAGES:\n");
    // print_buf(flash_end_content, FLASH_PAGE_SIZE * 2);
}

// UART TASK
void vUARTTask(void *pvParameters)
{
    (void)pvParameters;
    uint32_t ulNotificationValue;
    xTaskToNotify_UART = NULL;

    while (true)
    {
        printf("UART TASK\r\n");
        UART_receive();
        ulNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (ulNotificationValue == 1)
        {
            while (uart_is_readable(UART0_ID))
            {
                rxChar = uart_getc(UART0_ID);

                if (uart_is_writable(UART0_ID))
                {
                    uart_putc(UART0_ID, rxChar);
                }
                if (rxChar == 'b')
                {
                    get_time_pt7c4338(I2C_PORT, I2C_ADDRESS);
                    datetime_to_str(datetime_str, sizeof(datetime_buf), &t);
                    uart_puts(UART0_ID, datetime_str);
                    uart_puts(UART0_ID, "\r\n");
                }
                if (rxChar == 'v')
                {
                    uart_puts(UART0_ID, "vrms = ");
                    uart_puts(UART0_ID, "\r\n");
                }
                if (rxChar == 'r')
                {
                    reset_flash();
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

// ADC CONVERTER TASK
void vADCReadTask()
{
    TickType_t startTime;
    TickType_t executionTime;
    const float conversion_factor = 3.3f / (1 << 12);

    while (1)
    {
        startTime = xTaskGetTickCount();
        // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
        int sample_count;
        vrms_accumulator = 0;
        uint16_t sample;

        for (sample_count = 0; sample_count < VRMS_SAMPLE; sample_count++)
        {
            sample = adc_read() * conversion_factor;
            double vrms_sample = pow(sample, 2);
            vrms_accumulator += vrms_sample;
            vTaskDelay(4);
        }
        vrms = sqrt(vrms_accumulator / VRMS_SAMPLE);

        spi_write_buffer();
        executionTime = xTaskGetTickCount() - startTime;
        // vTaskDelay(3000 - (unsigned int)executionTime);
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
        printf("RTC Time:%s \r\n", datetime_str);
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
        printf("\nReset Pulse!\n");
        vTaskDelay(INTERVAL_MS);
    }
}

void main()
{
    stdio_init_all();
    sleep_ms(2000);

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
    // Select ADC input 0 (GPIO26)
    adc_select_input(1);

    // RTC Init
    rtc_init();
    rtc_set_datetime(&t);

    // I2C Init
    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(RTC_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(RTC_I2C_SCL_PIN, GPIO_FUNC_I2C);

    // SET TIMER TO RTC
    // set_time_pt7c4338(I2C_PORT, I2C_ADDRESS, 0x00, 0x04, 0x21, 0x01, 0x30, 0x07, 0x23);

    sector_data = *(uint16_t *)flash_sector_contents;
    printf("%d", sector_data);

    uint8_t *flash_target_contents = (uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET + (sector_data * FLASH_SECTOR_SIZE));

    for (uint16_t k = 0; k < FLASH_SECTOR_SIZE; k++)
    {
        flash_data[k] = flash_target_contents[k];
    }

    // for (int i = 0; i < FLASH_SECTOR_SIZE - FLASH_PAGE_SIZE; i++)
    // {
    //     flash_data[i] = 12;
    // }

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
