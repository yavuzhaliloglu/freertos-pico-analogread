#include "stdio.h"
#include <stdlib.h>
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
#include "header/rtc.h"
#include "header/spiflash.h"
#include "header/defines.h"
#include "header/bcc.h"
#include "header/variables.h"
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

                // char test[3];
                // test[0] = rx_char;
                // test[1] = '\n';
                // test[2] = '\0';

                // printf("%s", test);

                if (state == WriteProgram)
                {
                    xTimerReset(ResetBufferTimer, 0);
                    xTimerReset(ResetStateTimer, 0);
                    xTimerStop(ResetBufferTimer, 0);
                    xTimerStop(ResetStateTimer, 0);
                    bool reprogram_is_done = writeProgramToFlash(rx_char);
                    if (reprogram_is_done)
                        rebootDevice();
                    continue;
                }
                // ENTER CONTROL
                if (rx_char != '\n')
                {
                    rx_buffer[rx_buffer_len++] = rx_char;
                }
                if (rx_char == '\n' || controlRXBuffer(rx_buffer, rx_buffer_len))
                {
                    printf("\n");

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
                        printf("listening state entered.\n");
                        xTimerStart(ResetStateTimer, 0);

                        switch (checkListeningData(rx_buffer, rx_buffer_len))
                        {
                        case ReProgram:
                            state = WriteProgram;
                            setReProgramSize(rx_buffer);

                        case DataError:
                            uart_putc(UART0_ID, 0x15);
                            break;

                        case Reading:
                            printf("state is listening-reading.\n");
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

void vADCReadTask()
{
    TickType_t startTime;
    TickType_t xFrequency = pdMS_TO_TICKS(60000);
    uint8_t vrms_buffer_count = 0;
    double vrms_accumulator = 0.0;
    const float conversion_factor = 1000 * (3.3f / (1 << 12));
    uint8_t vrms_buffer[VRMS_BUFFER_SIZE] = {0};
    double vrms = 0.0;

    while (1)
    {

        if (adc_remaining_time > 0)
        {
            vTaskDelay(pdMS_TO_TICKS(60000) - adc_remaining_time);
            adc_remaining_time = 0;
        }

        startTime = xTaskGetTickCount();

        rtc_get_datetime(&current_time);
        datetime_to_str(datetime_str, sizeof(datetime_buffer), &current_time);
        printf("Alarm Fired At %s\n", datetime_str);

        adcCapture(sample_buffer, VRMS_SAMPLE);

#if DEBUG
        char deneme[40] = {0};
        for (uint8_t i = 0; i < 150; i++)
        {
            snprintf(deneme, 20, "sample: %d\n", sample_buffer[i]);
            deneme[21] = '\0';

            printf("%s", deneme);
            vTaskDelay(1);
        }
        printf("\n");
#endif

        float mean = 2050 * conversion_factor / 1000;

#if DEBUG
        snprintf(deneme, 30, "mean: %f\n", mean);
        deneme[31] = '\0';
        printf("%s", deneme);
#endif

        for (uint16_t i = 0; i < VRMS_SAMPLE; i++)
        {
            double production = (double)(sample_buffer[i] * conversion_factor) / 1000;
            vrms_accumulator += pow((production - mean), 2);
        }

#if DEBUG
        snprintf(deneme, 34, "vrmsAc: %f\n", vrms_accumulator);
        deneme[35] = '\0';
        printf("%s", deneme);
#endif
        vrms = sqrt(vrms_accumulator / VRMS_SAMPLE);
        vrms = vrms * 75;

        // printf("vrms: %d\n",(uint8_t)vrms);

        vrms_buffer[vrms_buffer_count++] = (uint8_t)vrms;

        vrms_accumulator = 0.0;
        vrms = 0.0;

        if (time_change_flag)
        {
            adc_remaining_time = pdMS_TO_TICKS((current_time.sec) * 1000);
            if (current_time.min % 15 == 0)
            {
                SPIWriteToFlash();
            }
            time_change_flag = 0;
            vTaskDelay(60000 - adc_remaining_time);
            adc_remaining_time = 0;
        }

        if ((current_time.sec < 5 && current_time.min % 15 == 0))
        {
            vrmsSetMinMaxMean(vrms_buffer, vrms_buffer_count);
            SPIWriteToFlash();
            memset(vrms_buffer, 0, 15);
            vrms_buffer_count = 0;
        }

        // vTaskDelay(5000);
        vTaskDelayUntil(&startTime, xFrequency);
    }
}

// DEBUG TASK

void vWriteDebugTask()
{
    TickType_t startTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000);
    startTime = xTaskGetTickCount();

    while (true)
    {
        vTaskDelayUntil(&startTime, xFrequency);
        rtc_get_datetime(&current_time);
        datetime_to_str(datetime_str, sizeof(datetime_buffer), &current_time);
        printf("RTC Time:%s \r\n", datetime_str);
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

    // RTC
    getTimePt7c4338(&current_time);
    rtc_set_datetime(&current_time);
    sleep_us(64);
    adc_remaining_time = pdMS_TO_TICKS((current_time.sec + 1) * 1000);

    xTaskCreate(vADCReadTask, "ADCReadTask", 256, NULL, 3, NULL);
    xTaskCreate(vUARTTask, "UARTTask", UART_TASK_STACK_SIZE, NULL, UART_TASK_PRIORITY, NULL);
    xTaskCreate(vWriteDebugTask, "WriteDebugTask", 256, NULL, 2, NULL);
    xTaskCreate(vResetTask, "ResetTask", 256, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true)
        ;
}
