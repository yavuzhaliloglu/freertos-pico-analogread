#include "stdio.h"
// #include <stdlib.h>
#include <time.h>
#include <string.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"
#include "timers.h"
#include "pico/util/datetime.h"
#include "pico/binary_info.h"
#include "pico/bootrom.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/rtc.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/flash.h"
#include "hardware/watchdog.h"
#include "hardware/structs/watchdog.h"
#include "hardware/timer.h"
#include "header/defines.h"
#include "header/variables.h"
#include "header/md5.h"
#include "header/bcc.h"
#include "header/rtc.h"
#include "header/spiflash.h"
#include "header/uart.h"
#include "header/adc.h"

// UART TASK
void vUARTTask(void *pvParameters)
{
    (void)pvParameters;
    uint32_t ulNotificationValue;
    xTaskToNotify_UART = NULL;
    uint8_t rx_char;

    TimerHandle_t ResetBufferTimer = xTimerCreate(
        "BufferTimer",
        pdMS_TO_TICKS(5000),
        pdFALSE,
        NULL,
        resetRxBuffer);

    TimerHandle_t ResetStateTimer = xTimerCreate(
        "StateTimer",
        pdMS_TO_TICKS(30000),
        pdFALSE,
        NULL,
        resetState);

    TimerHandle_t ReprogramTimer = xTimerCreate(
        "ReprogramTimer",
        pdMS_TO_TICKS(5000),
        pdFALSE,
        NULL,
        RebootHandler);

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
                // printBinary(rx_char);
                // stdio_flush();

                if (state == WriteProgram)
                {
                    xTimerReset(ReprogramTimer, 0);
                    writeProgramToFlash(rx_char);
                    continue;
                }

                // ENTER CONTROL
                if (rx_char != '\n')
                {
                    rx_buffer[rx_buffer_len++] = rx_char;
                }
                if (rx_char == '\n' || controlRXBuffer(rx_buffer, rx_buffer_len))
                {
                    printf("entered\n");
                    rx_buffer[rx_buffer_len++] = rx_char;
                    vTaskDelay(pdMS_TO_TICKS(200));

                    xTimerReset(ResetStateTimer, 0);
                    xTimerStop(ResetBufferTimer, 0);
                    char *ptr = strchr(rx_buffer, 0x2F);

                    switch (state)
                    {
                    case Greeting:
                        if (ptr != NULL)
                            greetingStateHandler(ptr, rx_buffer_len - ((uint8_t *)ptr - rx_buffer));

                        xTimerStart(ResetStateTimer, 0);
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

                        case ReProgram:
                            ReProgramHandler(rx_buffer, rx_buffer_len);
                            printf("state is writeprogram.\n");
                            xTimerStop(ResetBufferTimer, 0);
                            xTimerStop(ResetStateTimer, 0);
                            xTimerStart(ReprogramTimer, pdMS_TO_TICKS(100));
                            break;

                        case Password:
                            passwordHandler(rx_buffer, rx_buffer_len);
                            break;

                        case TimeSet:
                            setTimeFromUART(rx_buffer);
                            break;

                        case DateSet:
                            setDateFromUART(rx_buffer);
                            break;

                        case ProductionInfo:
                            sendProductionInfo();
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

void vADCReadTask()
{
    TickType_t startTime;
    TickType_t xFrequency = pdMS_TO_TICKS(60000);
    static uint8_t vrms_buffer_count = 0;
    uint8_t vrms_buffer[VRMS_BUFFER_SIZE] = {0};
    double vrms = 0.0;
    TickType_t vaitingTime = 0;

    while (1)
    {

#if !DEBUG
        if (adc_remaining_time > 0)
        {
            vTaskDelay(pdMS_TO_TICKS(60000) - adc_remaining_time);
            adc_remaining_time = 0;
        }
#endif
        startTime = xTaskGetTickCount();

        printf("Alarm Fired At %s\n", datetime_str);

        vrms = calculateVRMS();
        vrms_buffer[vrms_buffer_count++] = (uint8_t)vrms;

        // vrms_accumulator = 0.0;
        vrms = 0.0;

        if (time_change_flag)
        {
            vaitingTime = pdMS_TO_TICKS(60000 - ((current_time.sec) * 1000));
            if (current_time.min % 15 == 0)
            {
                SPIWriteToFlash();
            }
            time_change_flag = 0;
            vTaskDelay(vaitingTime);
            vaitingTime = 0;
            continue;
        }

        if ((current_time.sec < 5 && current_time.min % 15 == 0))
        {
            vrmsSetMinMaxMean(vrms_buffer, vrms_buffer_count);
            SPIWriteToFlash();
            memset(vrms_buffer, 0, 15);
            vrms_buffer_count = 0;
        }

#if DEBUG
        vTaskDelay(5000);
#else
        vTaskDelayUntil(&startTime, xFrequency);
#endif
    }
}

// DEBUG TASK

// void vWriteDebugTask()
// {
//     TickType_t startTime;
//     const TickType_t xFrequency = pdMS_TO_TICKS(1000);
//     startTime = xTaskGetTickCount();

//     while (true)
//     {
//         vTaskDelayUntil(&startTime, xFrequency);
//         rtc_get_datetime(&current_time);
//         datetime_to_str(datetime_str, sizeof(datetime_buffer), &current_time);
//         printf("The Time is:%s \r\n", datetime_str);
//     }
// }

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

bool repeating_timer_callback(struct repeating_timer *rt)
{
    rtc_get_datetime(&current_time);
    datetime_to_str(datetime_str, sizeof(datetime_buffer), &current_time);
    printf("The Time for the mustafa abi:%s \r\n", datetime_str);

    return true;
}

void main()
{
    stdio_init_all();
    sleep_ms(1000);

    // uint8_t *ptr = (uint8_t *)(XIP_BASE);
    // printBufferHex(ptr, 12 * 4096);

    // while (true)
    // {
    //     printf("hello world!\n");
    //     sleep_us(1000000);
    // }

    // UART INIT
    uart_init(UART0_ID, BAUD_RATE);
    initUART();
    gpio_set_function(UART0_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART0_RX_PIN, GPIO_FUNC_UART);

    // RESET INIT
    gpio_init(RESET_PULSE_PIN);
    gpio_set_dir(RESET_PULSE_PIN, GPIO_OUT);
    gpio_init(3);
    gpio_set_dir(3, GPIO_OUT);

    // ADC INIT
    adc_init();
    adc_gpio_init(ADC_READ_PIN);
    adc_select_input(ADC_SELECT_INPUT);
    adc_set_clkdiv(CLOCK_DIV);
    adcCapture(sample_buffer, VRMS_SAMPLE);
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

    // char helloworld[] = "Hello World!";
    // int size = strlen(helloworld);
    // unsigned char md5sum[MD5_DIGEST_LENGTH];

    // calculateMD5(helloworld, size, md5sum);

    // printf("MD5 Checksum for %s string is:", helloworld);
    // for (int i = 0; i < MD5_DIGEST_LENGTH; i++)
    // {
    //     printf("%02x", md5sum[i]);
    // }
    // printf("\n");

    // serial number addition

    uint8_t s_number[256] = "60616161";
    // printBufferHex(s_number, 256);

    flash_range_erase(FLASH_SERIAL_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_SERIAL_OFFSET, s_number, FLASH_PAGE_SIZE);
    // printf("%s", serial_number);

    // printBufferHex((uint8_t *)(XIP_BASE + FLASH_SERIAL_OFFSET), 256);

    // RTC
    getTimePt7c4338(&current_time);
    rtc_set_datetime(&current_time);
    sleep_us(64);
    adc_remaining_time = pdMS_TO_TICKS((current_time.sec) * 1000);

    struct repeating_timer timer;
    add_repeating_timer_us(1000000, repeating_timer_callback, NULL, &timer);

    xTaskCreate(vADCReadTask, "ADCReadTask", 256, NULL, 2, &xADCHandle);
    xTaskCreate(vUARTTask, "UARTTask", UART_TASK_STACK_SIZE, NULL, UART_TASK_PRIORITY, NULL);
    // xTaskCreate(vWriteDebugTask, "WriteDebugTask", 256, NULL, 2, NULL);
    xTaskCreate(vResetTask, "ResetTask", 256, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true)
        ;
}
