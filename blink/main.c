#include <stdio.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
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
#include "header/print.h"
#include "header/md5.h"
#include "header/bcc.h"
#include "header/rtc.h"
#include "header/spiflash.h"
#include "header/uart.h"
#include "header/adc.h"

// UART TASK: This task gets uart characters and handles them
void vUARTTask(void *pvParameters)
{
    // Confugiration parameters
    (void)pvParameters;
    uint32_t ulNotificationValue;
    xTaskToNotify_UART = NULL;
    uint8_t rx_char;
    char *st_chr_msg;

    // This timer deletes rx_buffer if there is no character coming in 5 seconds.
    TimerHandle_t ResetBufferTimer = xTimerCreate(
        "BufferTimer",
        pdMS_TO_TICKS(5000),
        pdFALSE,
        NULL,
        resetRxBuffer);

    // This timer sets state to Greeting(Initial) if there is no request or message in 30 seconds.
    TimerHandle_t ResetStateTimer = xTimerCreate(
        "StateTimer",
        pdMS_TO_TICKS(30000),
        pdFALSE,
        NULL,
        resetState);

    // This timer handles to reboot device if there is no character coming in WriteProgram state. When this timer executes, it means that program data coming from UART is over or cut off
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
        // If a character comes, this block is going to be executed
        if (ulNotificationValue == 1)
        {
            // Check if UART port is readable
            while (uart_is_readable(UART0_ID))
            {
                // Get character from UART
                rx_char = uart_getc(UART0_ID);

                // If state is WriteProgram then the characters coming from UART are going to handle in this block, because that characters are represent program bytes.
                if (state == WriteProgram)
                {
                    xTimerReset(ReprogramTimer, 0);
                    writeProgramToFlash(rx_char);
                    continue;
                }
#if DEBUG
                printf("%02X\n", rx_char);
#endif
                // CR/LF control for the message, also this is the end character control for the message
                if (rx_char != '\n')
                {
                    rx_buffer[rx_buffer_len++] = rx_char;
                }
                // The end of the message could be '\n' character or a BCC, so this if block checks if the character is '\n' or the whole message is request message according to its length and order of characters
                if (rx_char == '\n' || controlRXBuffer(rx_buffer, rx_buffer_len))
                {
                    // Get the last character of the message and wait for 250 ms. This waiting function is a requirement for the IEC 620256-21 protocol.
                    if (state != Listening)
                        rx_buffer[rx_buffer_len++] = rx_char;

                    vTaskDelay(pdMS_TO_TICKS(250));
                    xTimerReset(ResetStateTimer, 0);
                    xTimerStop(ResetBufferTimer, 0);
#if DEBUG
                    printf("UART TASK: message end and entered the processing area\n");
                    printf("UART TASK: rx len content: ");
                    printBufferHex(rx_buffer, rx_buffer_len);
                    printf("\n");
                    printf("UART TASK: rx_buffer_len value: %d\n", rx_buffer_len);
#endif

                    switch (state)
                    {
                    // This is the Initial state in device. In this state, modem and device will handshake.
                    case Greeting:
#if DEBUG
                        printf("UART TASK: entered greeting state\n");
#endif
                        // Start character of the request message (st_chr_msg) is a protection for message integrity. If there are characters before the greeting message, these characters will be ignored and message will be clear to send to the greeting handler.
                        st_chr_msg = strchr(rx_buffer, 0x2F);
                        greetingStateHandler(st_chr_msg, rx_buffer_len - ((uint8_t *)st_chr_msg - rx_buffer));

                        xTimerStart(ResetStateTimer, 0);
                        break;

                    // This state sets baud rate or sends readout message.
                    case Setting:
#if DEBUG
                        printf("UART TASK: entered setting state\n");
#endif
                        xTimerStart(ResetStateTimer, 0);
                        settingStateHandler(rx_buffer, rx_buffer_len);
                        break;

                    // This state handles the request messages for load profile, set date and time, send production info and also before entering WriteProgram state.
                    case Listening:
#if DEBUG
                        printf("UART TASK: entered listening state\n");
#endif
                        xTimerStart(ResetStateTimer, 0);

                        // This switch block checks the request message for the which state is going to handled according to message.
                        switch (checkListeningData(rx_buffer, rx_buffer_len))
                        {
                        case DataError:
#if DEBUG
                            printf("UART TASK: entered listening-dataerror\n");
#endif
                            uart_putc(UART0_ID, 0x15);
                            break;

                        // This state represents Load Profile request and send a load profile message for specified dates. If there is no date information, device send all the load profile contents.
                        case Reading:
#if DEBUG
                            printf("UART TASK: entered listening-reading\n");
#endif
                            parseReadingData(rx_buffer);
                            searchDataInFlash();
                            break;

                        // This state handles the tasks, timers and sets the state to WriteProgram to start program data handling.
                        case ReProgram:
#if DEBUG
                            printf("UART TASK: entered listening-reprogram\n");
#endif
                            ReProgramHandler(rx_buffer, rx_buffer_len);
                            xTimerStop(ResetBufferTimer, 0);
                            xTimerStop(ResetStateTimer, 0);
                            xTimerStart(ReprogramTimer, pdMS_TO_TICKS(100));
                            break;

                        // This state accepts the password and checks. If the password is not correct, time and date in this device cannot be changed.
                        case Password:
#if DEBUG
                            printf("UART TASK: entered listening-password\n");
#endif
                            passwordHandler(rx_buffer, rx_buffer_len);
                            break;

                        // This state changes time of this device.
                        case TimeSet:
#if DEBUG
                            printf("UART TASK: entered listening-timeset\n");
#endif
                            setTimeFromUART(rx_buffer);
                            break;

                        // This state changes date of this device.
                        case DateSet:
#if DEBUG
                            printf("UART TASK: entered listening-dateset\n");
#endif
                            setDateFromUART(rx_buffer);
                            break;

                        // This state sends production info for this device.
                        case ProductionInfo:
#if DEBUG
                            printf("UART TASK: entered listening-productioninfo\n");
#endif
                            sendProductionInfo();
                            break;

                        case SetThreshold:
#if DEBUG
                            printf("UART TASK: entered listening-setthreshold\n");
#endif
                            setThresholdValue(rx_buffer);
                            break;
                            // If the message is not correct, device sends a NACK message (0x15) to modem.

                        case GetThreshold:
#if DEBUG
                            printf("UART TASK: entered listening-getthreshold\n");
#endif
                            getThresholdRecord();
                            break;

                        default:
#if DEBUG
                            printf("UART TASK: entered listening-default\n");
#endif
                            uart_putc(UART0_ID, 0x15);
                            break;
                        }
                        break;
                    }
                    // After a request or message, buffers and index variables will be set to zero.
                    memset(rx_buffer, 0, 256);
                    rx_buffer_len = 0;
#if DEBUG
                    printf("UART TASK: buffer content deleted\n");
#endif
                }
            }
        }
    }
}

// ADC CONVERTER TASK: This task read ADC PIN to calculate VRMS value and writes a record to flash memory according to current time.
void vADCReadTask()
{
    // Set the parameters for this task.
    TickType_t startTime;
    TickType_t xFrequency = pdMS_TO_TICKS(60000);
    double vrms = 0.0;
    TickType_t vaitingTime = 0;
    double bias_voltage;

    while (1)
    {
        if (adc_remaining_time > 0)
        {
            vTaskDelay(pdMS_TO_TICKS(60000) - adc_remaining_time);
            adc_remaining_time = 0;
        }
        // If time of this device is changed, this block will be executed. This task executes periodically and when time changes, the period of this task is going to slip.
        // This block aligns the period to beginning of the minute and if the current minute is aligned to 15.period, writes the record to flash and wait until beginning of the minute and continues.
        if (time_change_flag)
        {
#if DEBUG
            printf("ADC READ TASK: time change flag - block is running...\n");
#endif
            // Calculate waiting time to align the task to beginning of the minute.
            vaitingTime = pdMS_TO_TICKS(60000 - ((current_time.sec) * 1000) + 100);
            // Set the parameters for this block to zero and wait until beginning of the minute.
            time_change_flag = 0;
            vTaskDelay(vaitingTime);
            vaitingTime = 0;
        }

        startTime = xTaskGetTickCount();
#if DEBUG
        printf("ADC READ TASK: adc task entered at %s.\n", datetime_str);
#endif
        // Select the ADC input to BIAS voltage PIN and Calculate BIAS voltage
        adc_select_input(ADC_BIAS_INPUT);
        adcCapture(bias_buffer, BIAS_SAMPLE);
        bias_voltage = getMean(bias_buffer, BIAS_SAMPLE);

        // Select the ADC input to voltage PIN and calculate VRMS value
        adc_select_input(ADC_SELECT_INPUT);
        vrms = calculateVRMS(bias_voltage);
#if DEBUG
        printf("ADC READ TASK: bias voltage is: %lf\n", bias_voltage);
        printf("ADC READ TASK: calcualted vrms is: %lf\n", vrms);
#endif
        // check if vrms value is bigger than threshold
        checkVRMSThreshold(vrms);

        // Add calculated VRMS value to VRMS buffer and set VRMS value to zero.
        vrms_buffer[(vrms_buffer_count++) % 15] = vrms;
        vrms = 0.0;
#if DEBUG
        printf("ADC READ TASK: now buffer content is: \n");
        for (int8_t i = 0; i < vrms_buffer_count; i++)
            printf("%lf\n", vrms_buffer[i]);
        printf("\n");
#endif
        // Write a record to the flash memory periodically
        if ((current_time.sec < 5 && current_time.min % 15 == 0))
        {
#if DEBUG
            printf("ADC READ TASK: minute is multiple of 15. write flash block is running...\n");
#endif
            if (vrms_buffer_count > 15)
                vrms_buffer_count = 15;

            vrmsSetMinMaxMean(vrms_buffer, vrms_buffer_count);
#if DEBUG
            printf("ADC READ TASK: calculated VRMS values.\n");
            printf("ADC READ TASK: vrms max is: %d,vrms min is:%d,vrms mean is: %d,vrms max dec is: %d,vrms min dec is: %d,vrms mean dec is: %d\n", vrms_max, vrms_min, vrms_mean, vrms_max_dec, vrms_min_dec, vrms_mean_dec);
#endif
            SPIWriteToFlash();
#if DEBUG
            printf("ADC READ TASK: writing flash memory process is completed.\n");
#endif
            memset(vrms_buffer, 0, VRMS_BUFFER_SIZE);
            vrms_buffer_count = 0;
#if DEBUG
            printf("ADC READ TASK: buffer content is deleted\n");
#endif
        }
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
#if DEBUG
        printf("WRITE DEBUG TASK: The Time is:%s \r\n", datetime_str);
#endif
    }
}

// RESET TASK: This task sends a pulse to reset PIN.
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
    adc_gpio_init(ADC_BIAS_PIN);
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

    // sector content control
    checkSectorContent();

    // // Reset Record Settings
    // resetFlashSettings();

    // FLASH CONTENTS
    getFlashContents();

    // Get PT7C4338's Time information and set RP2040's RTC module
    getTimePt7c4338(&current_time);
    rtc_set_datetime(&current_time);
    sleep_us(64);
    // Align itself to beginning of the minute
    adc_remaining_time = pdMS_TO_TICKS(((current_time.sec + 1) * 1000) - 100);

#if DEBUG
    // // FLASH RECORD AREA DEBUG
    // uint8_t *flash_record_offset = (uint8_t *)(XIP_BASE + FLASH_DATA_OFFSET);
    // printBufferHex(flash_record_offset, 10 * FLASH_PAGE_SIZE);

    printf("MAIN: flash sector is: %d\n", sector_data);
    printf("MAIN: adc_remaining_time is %ld\n", adc_remaining_time);
#endif

    xTaskCreate(vADCReadTask, "ADCReadTask", 1024, NULL, 3, &xADCHandle);
    xTaskCreate(vUARTTask, "UARTTask", UART_TASK_STACK_SIZE, NULL, UART_TASK_PRIORITY, NULL);
    xTaskCreate(vWriteDebugTask, "WriteDebugTask", 256, NULL, 5, NULL);
    xTaskCreate(vResetTask, "ResetTask", 256, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true)
        ;
}
