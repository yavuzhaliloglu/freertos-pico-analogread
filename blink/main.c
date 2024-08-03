#include <stdio.h>
#include "header/project-conf.h"
#include <time.h>
#include <string.h>
#include <float.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
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
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"
#include "hardware/structs/watchdog.h"
#include "hardware/timer.h"
#include "header/defines.h"
#include "header/variables.h"
#include "header/print.h"
#include "header/fifo.h"
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
                uint8_t rx_char = uart_getc(UART0_ID);

                // If state is ReProgram then the characters coming from UART are going to handle in this block, because that characters are represent program bytes.
                if (state == ReProgram)
                {
                    xTimerReset(ReprogramTimer, 0);
                    writeProgramToFlash(rx_char);

                    continue;
                }

                PRINTF("%02X\n", rx_char);

                rx_buffer[rx_buffer_len++] = rx_char;

                // The end of the message could be '\n' character or a BCC, so this if block checks if the character is '\n' or the whole message is request message according to its length and order of characters
                if (rx_char == '\n' || controlRXBuffer(rx_buffer, rx_buffer_len))
                {
                    // Get the last character of the message and wait for 250 ms. This waiting function is a requirement for the IEC 620256-21 protocol.
                    vTaskDelay(pdMS_TO_TICKS(250));

                    xTimerReset(ResetStateTimer, 0);
                    xTimerStop(ResetBufferTimer, 0);

                    PRINTF("UART TASK: message end and entered the processing area\n");
                    PRINTF("UART TASK: rx buffer content: ");
                    printBufferHex(rx_buffer, rx_buffer_len);
                    PRINTF("\n");
                    PRINTF("UART TASK: rx buffer len value: %d\n", rx_buffer_len);

                    // check if the message is for end connection
                    if (is_end_connection_message(rx_buffer))
                    {
                        resetRxBuffer();
                        resetState();

                        break;
                    }

                    switch (state)
                    {
                    // This is the Initial state in device. In this state, modem and device will handshake.
                    case Greeting:
                        PRINTF("UART TASK: entered greeting state\n");
                        xTimerStart(ResetStateTimer, 0);

                        // Start character of the request message (st_chr_msg) is a protection for message integrity. If there are characters before the greeting message, these characters will be ignored and message will be clear to send to the greeting handler.
                        char *st_chr_msg = strchr((char *)rx_buffer, 0x2F);
                        greetingStateHandler((uint8_t *)st_chr_msg);
                        break;

                    // This state sets baud rate or sends readout message.
                    case Setting:
                        PRINTF("UART TASK: entered setting state\n");
                        xTimerStart(ResetStateTimer, 0);

                        settingStateHandler(rx_buffer, rx_buffer_len);
                        break;

                    // This state handles the request messages for load profile, set date and time, send production info and also before entering WriteProgram state.
                    case Listening:
                        PRINTF("UART TASK: entered listening state\n");
                        xTimerStart(ResetStateTimer, 0);

                        // This switch block checks the request message for which state is going to handled according to message.
                        switch (checkListeningData(rx_buffer, rx_buffer_len))
                        {
                        case DataError:
                            PRINTF("UART TASK: entered listening-dataerror\n");
                            sendErrorMessage((char *)"DATAERROR");
                            break;

                        case BCCError:
                            PRINTF("UART TASK: entered listening-bccerror\n");
                            sendErrorMessage((char *)"BCCERROR");
                            break;

                        // This state represents Load Profile request and send a load profile message for specified dates. If there is no date information, device send all the load profile contents.
                        case Reading:
                            PRINTF("UART TASK: entered listening-reading\n");
                            parseLoadProfileDates(rx_buffer, rx_buffer_len);
                            searchDataInFlash();
                            break;

                        // This state handles the tasks, timers and sets the state to WriteProgram to start program data handling.
                        case WriteProgram:
                            PRINTF("UART TASK: entered listening-reprogram\n");
                            ReProgramHandler();

                            xTimerStop(ResetBufferTimer, 0);
                            xTimerStop(ResetStateTimer, 0);
                            xTimerStart(ReprogramTimer, pdMS_TO_TICKS(100));
                            break;

                        // This state accepts the password and checks. If the password is not correct, time and date in this device cannot be changed.
                        case Password:
                            PRINTF("UART TASK: entered listening-password\n");
                            passwordHandler(rx_buffer);
                            break;

                        // This state changes time of this device.
                        case TimeSet:
                            PRINTF("UART TASK: entered listening-timeset\n");
                            setTimeFromUART(rx_buffer);
                            break;

                        // This state changes date of this device.
                        case DateSet:
                            PRINTF("UART TASK: entered listening-dateset\n");
                            setDateFromUART(rx_buffer);
                            break;

                        // This state sends production info for this device.
                        case ProductionInfo:
                            PRINTF("UART TASK: entered listening-productioninfo\n");
                            sendProductionInfo();
                            break;

                        case SetThreshold:
                            PRINTF("UART TASK: entered listening-setthreshold\n");
                            setThresholdValue(rx_buffer);
                            break;

                        case GetThreshold:
                            PRINTF("UART TASK: entered listening-getthreshold\n");
                            getThresholdRecord();
                            break;

                        case ThresholdPin:
                            PRINTF("UART TASK: entered listening-thresholdpin\n");
                            resetThresholdPIN();
                            break;

                        default:
                            PRINTF("UART TASK: entered listening-default\n");
                            sendErrorMessage((char *)"UNSUPPORTEDLSTMSG");
                            break;
                        }
                        break;
                    case ReProgram:
                        PRINTF("UART TASK: entered reprogram state (UNSUPPORTED!)\n");
                        break;
                    }
                    // After a request or message, buffers and index variables will be set to zero.
                    memset(rx_buffer, 0, 256);
                    rx_buffer_len = 0;
                    PRINTF("UART TASK: buffer content deleted\n");
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
    TickType_t xFrequency = pdMS_TO_TICKS(1000);
    startTime = xTaskGetTickCount();

    while (1)
    {
        // // delay until next cycle
        vTaskDelayUntil(&startTime, xFrequency);

        getLastNElementsToBuffer(&adc_fifo, adc_samples_buffer, VRMS_SAMPLE_SIZE);

        adc_select_input(ADC_BIAS_INPUT);
        adcCapture(bias_buffer, BIAS_SAMPLE);
        float bias_voltage = getMean(bias_buffer, BIAS_SAMPLE);
        PRINTF("bias voltage is: %lf\n", bias_voltage);

        adc_select_input(ADC_SELECT_INPUT);
        float vrms = calculateVRMS(bias_voltage, adc_samples_buffer);
        PRINTF("vrms is: %lf\n", vrms);

        uint16_t variance = calculateVariance(adc_samples_buffer, VRMS_SAMPLE_SIZE);
        PRINTF("variance is: %d\n", variance);

        vrms_buffer[(vrms_buffer_count++) % VRMS_BUFFER_SIZE] = vrms;

        if (current_time.sec == 0)
        {
            // TODO: CONTROL THRESHOLD
            if (vrms >= (float)getVRMSThresholdValue())
            {
                setThresholdPIN();
                writeThresholdRecord(vrms, variance);
            }

            if (current_time.min % load_profile_record_period == 0)
            {
                PRINTF("ADC READ TASK: minute is multiple of %d. write flash block is running...\n", load_profile_record_period);

                if (vrms_buffer_count > VRMS_BUFFER_SIZE)
                {
                    vrms_buffer_count = VRMS_BUFFER_SIZE;
                }

                vrmsSetMinMaxMean(vrms_buffer, vrms_buffer_count);
                PRINTF("ADC READ TASK: calculated VRMS values.\n");

                SPIWriteToFlash();
                PRINTF("ADC READ TASK: writing flash memory process is completed.\n");

                memset(vrms_buffer, 0, VRMS_BUFFER_SIZE * sizeof(float));
                vrms_buffer_count = 0;
                PRINTF("ADC READ TASK: buffer content is deleted\n");
            }
        }
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

        PRINTF("WRITE DEBUG TASK: The Time is:%s \r\n", datetime_str);
    }
}

void vADCSampleTask()
{
    TickType_t startTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(2);
    startTime = xTaskGetTickCount();
    uint16_t adc_sample;
    while (1)
    {
        adc_sample = adc_read(); // ADC'den örnek al

        bool is_added = addToFIFO(&adc_fifo, adc_sample);

        if (!is_added)
        {
            removeFirstElementAddNewElement(&adc_fifo, adc_sample);
        }

        vTaskDelayUntil(&startTime, xFrequency); // 2 ms bekle
    }
}

// RESET TASK: This task sends a pulse to reset PIN.
void vResetTask()
{
    while (1)
    {
        gpio_put(RESET_PULSE_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_put(RESET_PULSE_PIN, 0);
        vTaskDelay(INTERVAL_MS);
    }
}

int main()
{
    stdio_init_all();
    sleep_ms(500);

    // UART INIT
    uart_init(UART0_ID, BAUD_RATE);
    initUART();
    gpio_set_function(UART0_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART0_RX_PIN, GPIO_FUNC_UART);
    sleep_ms(100);

    // RESET INIT
    gpio_init(RESET_PULSE_PIN);
    gpio_set_dir(RESET_PULSE_PIN, GPIO_OUT);
    sleep_ms(100);

    // THRESHOLD GPIO INIT
    gpio_init(THRESHOLD_PIN);
    gpio_set_dir(THRESHOLD_PIN, GPIO_OUT);
    sleep_ms(100);
    gpio_put(THRESHOLD_PIN, 0);

    // ADC INIT
    adc_init();
    adc_gpio_init(ADC_READ_PIN);
    adc_gpio_init(ADC_BIAS_PIN);
    adc_select_input(ADC_SELECT_INPUT);
    adc_set_clkdiv(clkdiv);
    sleep_ms(100);

    // I2C Init
    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(RTC_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(RTC_I2C_SCL_PIN, GPIO_FUNC_I2C);
    sleep_ms(100);

#if WITHOUT_BOOTLOADER
    addSerialNumber();
    sleep_ms(100);
#endif

    // sector content control
    checkSectorContent();
    checkThresholdContent();
    sleep_ms(100);

    // // Reset Record Settings
    // resetFlashSettings();

    // FLASH CONTENTS
    getFlashContents();
    sleep_ms(100);

    // RTC Init
    rtc_init();
    sleep_ms(100);

    // Get PT7C4338's Time information and set RP2040's RTC module
    getTimePt7c4338(&current_time);
    sleep_ms(100);

    // If current time which is get from Chip has invalid value, adjust the value.
    if (current_time.dotw < 0 || current_time.dotw > 6)
    {
        current_time.dotw = 2;
    }

    // set and get datetimes from RP2040 RTC's
    bool is_time_set = rtc_set_datetime(&current_time);
    sleep_ms(100);
    bool is_time_get = rtc_get_datetime(&current_time);
    sleep_ms(100);

    // if time is get correctly, set string datetime.
    if (is_time_get)
    {
        datetime_to_str(datetime_str, sizeof(datetime_buffer), &current_time);
    }
    else
    {
        PRINTF("Time is not GET. Please check the time setting.\n");
    }

    initADCFIFO(&adc_fifo);

    // set when program started
    setProgramStartDate(&current_time);
    sleep_ms(100);

    // if time is set correctly, start the processes.
    if (is_time_set)
    {
        PRINTF("Time is set. Starting tasks...\n");

        xTaskCreate(vADCReadTask, "ADCReadTask", 1024, NULL, 3, &xADCHandle);
        xTaskCreate(vUARTTask, "UARTTask", UART_TASK_STACK_SIZE, NULL, UART_TASK_PRIORITY, NULL);
        xTaskCreate(vWriteDebugTask, "WriteDebugTask", 256, NULL, 5, NULL);
        xTaskCreate(vResetTask, "ResetTask", 256, NULL, 1, NULL);
        xTaskCreate(vADCSampleTask, "ADCSampleTask", 512, NULL, 2, NULL);

        vTaskStartScheduler();
    }
    else
    {
        PRINTF("Time is not SET. Please check the time setting.\n");
        watchdog_reboot(0, 0, 0);
    }

    while (true)
        ;
}
