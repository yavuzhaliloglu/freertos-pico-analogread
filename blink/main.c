#include "header/bcc.h"

#include "FreeRTOS.h"
#include "hardware/adc.h"
#include "hardware/structs/uart.h"
#include "hardware/structs/watchdog.h"
#include "hardware/watchdog.h"
#include "message_buffer.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "task.h"
#include <string.h>

#include "header/adc.h"
#include "header/fifo.h"
#include "header/mutex.h"
#include "header/print.h"
#include "header/project_globals.h"
#include "header/rtc.h"
#include "header/spiflash.h"
#include "header/uart.h"

#define LINE_FEED '\n' // The delimiter
#define ETX_CHAR 0x03  // The delimiter
MessageBufferHandle_t xUARTMessageBuffer;

// ISR içindeki static tanımları buraya global static olarak alıyoruz
static uint8_t temp_rx_buf[RX_BUFFER_SIZE];
static volatile size_t rx_index = 0;
static volatile bool waiting_for_bcc = false;

// Bu fonksiyonu ekle: Yazılım bufferını temizler
void reset_uart_software_buffer() {
    rx_index = 0;
    waiting_for_bcc = false;
    memset(temp_rx_buf, 0, RX_BUFFER_SIZE);
}

void __not_in_flash_func(uart_receive_interrupt_handler)() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // check for uart is readable, if uart is full, empty the fifo
    uart_hw_t *uart_hw = uart_get_hw(UART0_ID);

    // Check for UART errors (Overrun, Break, Parity, Framing)
    if (uart_hw->rsr & (UART_UARTRSR_OE_BITS | UART_UARTRSR_BE_BITS | UART_UARTRSR_PE_BITS | UART_UARTRSR_FE_BITS)) {
        // Clear all errors by writing to the Error Clear Register (which shares the same offset as RSR)
        uart_hw->rsr = (UART_UARTRSR_OE_BITS | UART_UARTRSR_BE_BITS | UART_UARTRSR_PE_BITS | UART_UARTRSR_FE_BITS);

        // If Overrun Error (OE) occurred, the FIFO was full and we lost data.
        // We should flush the FIFO to start fresh and avoid processing corrupted/partial data.
        while (uart_is_readable(UART0_ID)) {
            (void)uart_getc(UART0_ID);
        }
        rx_index = 0; // Reset software buffer
        waiting_for_bcc = false;
        return;
    }

    while (uart_is_readable(UART0_ID)) {
        uint8_t ch = uart_getc(UART0_ID);

        PRINTF("CHR: %02X\n", ch);

        // 1. Store character
        if (rx_index < RX_BUFFER_SIZE - 1) {
            temp_rx_buf[rx_index++] = ch;
        }

        // 2. Handle BCC if we were waiting for it
        if (waiting_for_bcc) {
            waiting_for_bcc = false;
            // Message complete with BCC
            temp_rx_buf[rx_index] = '\0'; // Null terminate
            xMessageBufferSendFromISR(
                xUARTMessageBuffer,
                temp_rx_buf,
                rx_index,
                &xHigherPriorityTaskWoken);
            rx_index = 0;
            continue;
        }

        // 3. Check for End of Message
        if (ch == LINE_FEED) {
            // Message complete with LF
            temp_rx_buf[rx_index] = '\0'; // Null terminate
            xMessageBufferSendFromISR(
                xUARTMessageBuffer,
                temp_rx_buf,
                rx_index,
                &xHigherPriorityTaskWoken);
            rx_index = 0;
        } else if (ch == ETX_CHAR) {
            // Next char is BCC, wait for it in next interrupt or loop iteration
            waiting_for_bcc = true;
        }
    }

    // 4. Perform the context switch if the Message Buffer woke the task
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// UART Initialization
uint8_t initUART() {
    uint set_brate = 0;
    set_brate = uart_init(UART0_ID, BAUD_RATE);
    if (set_brate == 0) {
        PRINTF("UART INIT ERROR!\n");
        return 0;
    }

    uart_set_format(UART0_ID, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART0_ID, true);
    int UART_IRQ = UART0_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(UART_IRQ, uart_receive_interrupt_handler);
    irq_set_enabled(UART_IRQ, true);
    uart_set_translate_crlf(UART0_ID, true);
    uart_set_irq_enables(UART0_ID, true, false); // Enable RX interrupt only

    return 1;
}

void vUARTTask() {
    uint8_t rx_buffer[RX_BUFFER_SIZE];
    size_t received_bytes;
    char identify_response_buf[IDENTIFICATION_RESPONSE_BUFFER_SIZE];
    size_t identify_response_len = 0;
    uint8_t message_retry_count = 0;
    uint8_t hex_baud_rate = 0;
    int8_t requested_mode = -1;
    // this buffer stores start time for load profile data
    uint8_t reading_state_start_time[14] = {0};
    // this buffer stores end time for load profile data
    uint8_t reading_state_end_time[14] = {0};

    while (1) {
        // wait for identification message
        received_bytes = xMessageBufferReceive(
            xUARTMessageBuffer,
            rx_buffer,
            sizeof(rx_buffer),
            portMAX_DELAY);

        if (received_bytes > 0) {
            // got identification message, control serial number and answer it
            PRINTF("---> %.*s\n", received_bytes, rx_buffer);

            vTaskDelay(pdMS_TO_TICKS(250));
            if (control_serial_number(rx_buffer, received_bytes) == true) {
                identify_response_len = create_identify_response_message(identify_response_buf, sizeof(identify_response_buf));

                if (identify_response_len >= sizeof(identify_response_buf)) {
                    PRINTF("Identification response buffer overflow!\n");
                    sendErrorMessage((char *)"IDRESPONSEBUFOVERFLOW");
                    continue;
                }

                // send message three times, waiting for response each time
                while (message_retry_count < MAX_MESSAGE_RETRY_COUNT) {
                    uart_puts(UART0_ID, identify_response_buf);
                    PRINTF("<--- %s", identify_response_buf);
                    uart_tx_wait_blocking(UART0_ID);

                    received_bytes = xMessageBufferReceive(
                        xUARTMessageBuffer,
                        rx_buffer,
                        sizeof(rx_buffer),
                        pdMS_TO_TICKS(1500));

                    if (received_bytes > 0) {
                        PRINTF("---> %.*s\n", received_bytes, rx_buffer);
                        break;
                    } else {
                        PRINTF("No message received after identification within timeout.\n");
                        message_retry_count++;
                    }
                }

                if (message_retry_count >= MAX_MESSAGE_RETRY_COUNT) {
                    PRINTF("Max message retry count reached. Aborting identification process.\n");
                    message_retry_count = 0;
                    continue;
                }

                // do operations
                vTaskDelay(pdMS_TO_TICKS(250));
                message_retry_count = 0;
                hex_baud_rate = exract_baud_rate_and_mode_from_message(rx_buffer, received_bytes, &requested_mode);
                set_device_baud_rate(hex_baud_rate);

                if (requested_mode == REQUEST_MODE_LONG_READ || requested_mode == REQUEST_MODE_SHORT_READ) {
                    PRINTF("Request is readout\n");
                    send_readout_message();
                    set_init_baud_rate();
                    continue;
                } else if (requested_mode == REQUEST_MODE_PROGRAMMING) {
                    PRINTF("Request is programming mode\n");
                    send_programming_acknowledgement();
                } else {
                    PRINTF("Request mode is invalid, ignoring message.\n");
                    set_init_baud_rate();
                    continue;
                }

                while (1) {
                    // wait for next message
                    received_bytes = xMessageBufferReceive(
                        xUARTMessageBuffer,
                        rx_buffer,
                        sizeof(rx_buffer),
                        pdMS_TO_TICKS(30000));

                    if (received_bytes <= 0 || is_message_break_command(rx_buffer)) {
                        PRINTF("No message received within timeout, ending programming mode.\n");

                        int UART_IRQ = UART0_ID == uart0 ? UART0_IRQ : UART1_IRQ;
                        irq_set_enabled(UART_IRQ, false);
                        set_init_baud_rate();

                        while (uart_is_readable(UART0_ID)) {
                            (void)uart_getc(UART0_ID);
                        }

                        reset_uart_software_buffer();
                        uart_get_hw(UART0_ID)->rsr = (UART_UARTRSR_OE_BITS | UART_UARTRSR_BE_BITS | UART_UARTRSR_PE_BITS | UART_UARTRSR_FE_BITS);
                        irq_set_enabled(UART_IRQ, true);
                        break;
                    }

                    PRINTF("---> %.*s\n", received_bytes, rx_buffer);

                    switch (checkListeningData(rx_buffer, received_bytes)) {
                        case DataError:
                            sendErrorMessage((char *)"DATAERROR");
                            break;

                        case BCCError:
                            sendErrorMessage((char *)"BCCERROR");
                            break;

                        case Password:
                            passwordHandler(rx_buffer);
                            break;

#if CONF_LOAD_PROFILE_ENABLED
                        case Reading:
                            parseLoadProfileDates(rx_buffer, received_bytes, reading_state_start_time, reading_state_end_time);
                            searchDataInFlash(rx_buffer, reading_state_start_time, reading_state_end_time, Reading);
                            break;
#endif
#if CONF_TIME_SET_ENABLED
                        case TimeSet:
                            setTimeFromUART(rx_buffer);
                            break;
#endif
#if CONF_DATE_SET_ENABLED
                        case DateSet:
                            setDateFromUART(rx_buffer);
                            break;
#endif
#if CONF_PRODUCTION_INFO_ENABLED
                        case ProductionInfo:
                            sendProductionInfo();
                            break;
#endif
#if CONF_THRESHOLD_SET_ENABLED
                        case SetThreshold:
                            setThresholdValue(rx_buffer);
                            break;
#endif
#if CONF_THRESHOLD_GET_ENABLED
                        case GetThreshold:
                            parseThresholdRequestDates(rx_buffer, reading_state_start_time, reading_state_end_time);
                            getThresholdRecord(reading_state_start_time, reading_state_end_time, GetThreshold);
                            break;
#endif
#if CONF_THRESHOLD_PIN_ENABLED
                        case ThresholdPin:
                            resetThresholdPIN();
                            break;
#endif
#if CONF_SUDDEN_AMPLITUDE_CHANGE_ENABLED
                        case GetSuddenAmplitudeChange:
                            parseACRequestDate(rx_buffer, reading_state_start_time, reading_state_end_time);
                            getSuddenAmplitudeChangeRecords(reading_state_start_time, reading_state_end_time, GetSuddenAmplitudeChange);
                            break;
#endif
#if CONF_TIME_READ_ENABLED
                        case ReadTime:
                            readTime();
                            break;
#endif
#if CONF_DATE_SET_ENABLED
                        case ReadDate:
                            readDate();
                            break;
#endif
#if CONF_SERIAL_NUMBER_READ_ENABLED
                        case ReadSerialNumber:
                            readSerialNumber();
                            break;
#endif
#if CONF_VRMS_MAX_READ_ENABLED
                        case ReadLastVRMSMax:
                            sendLastVRMSXValue(ReadLastVRMSMax);
                            break;
#endif
#if CONF_VRMS_MIN_READ_ENABLED
                        case ReadLastVRMSMin:
                            sendLastVRMSXValue(ReadLastVRMSMin);
                            break;
#endif
#if CONF_VRMS_MEAN_READ_ENABLED
                        case ReadLastVRMSMean:
                            sendLastVRMSXValue(ReadLastVRMSMean);
                            break;
#endif
#if CONF_RESET_DATES_READ_ENABLED
                        case ReadResetDates:
                            sendResetDates();
                            break;
#endif
#if CONF_THRESHOLD_OBIS_ENABLED
                        case GetThresholdObis:
                            sendThresholdObis();
                            break;
#endif
                        default:
                            sendErrorMessage((char *)"UNSUPPORTEDOPERATION");
                            break;
                    }
                }
            }
        } else {
            PRINTF("SN is invalid, ignoring message.\n");
            continue;
        }
    }
}

// ADC CONVERTER TASK: This task read ADC PIN to calculate VRMS value and writes a record to flash memory according to current time.
void vADCReadTask() {
    // Set the parameters for this task.
    struct AmplitudeChangeTimerCallbackParameters ac_data = {0};
    uint8_t amplitude_change_detect_flag = 0;
    // this is a buffer that keeps samples in ADC FIFO in ADC Input 1 to calculate VRMS value
    uint16_t adc_samples_buffer[VRMS_SAMPLE_SIZE];
    float vrms_values_per_second[VRMS_SAMPLE_SIZE / SAMPLE_SIZE_PER_VRMS_CALC];
    uint16_t vrms_buffer_count = 0;
    // vrms buffer values
    float vrms_buffer[VRMS_BUFFER_SIZE] = {0};

    while (1) {
        // delay until next cycle
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (xSemaphoreTake(xFIFOMutex, pdMS_TO_TICKS(250)) == pdTRUE) {
            getLastNElementsToBuffer(&adc_fifo, adc_samples_buffer, VRMS_SAMPLE_SIZE);
            xSemaphoreGive(xFIFOMutex);
        }

        float vrms = calculateVRMS(adc_samples_buffer, VRMS_SAMPLE_SIZE, bias_voltage);
        PRINTF("vrms is: %lf\r\n", vrms);

        uint16_t variance = calculateVariance(adc_samples_buffer, VRMS_SAMPLE_SIZE);
        // PRINTF("variance is: %d\n", variance);

        calculateVRMSValuesPerSecond(vrms_values_per_second, adc_samples_buffer, VRMS_SAMPLE_SIZE, SAMPLE_SIZE_PER_VRMS_CALC, bias_voltage);

        vrms_buffer[(vrms_buffer_count++) % VRMS_BUFFER_SIZE] = vrms;

#if CONF_THRESHOLD_PIN_ENABLED || CONF_THRESHOLD_GET_ENABLED
        if (vrms >= (float)getVRMSThresholdValue()) {
#if CONF_THRESHOLD_PIN_ENABLED
            setThresholdPIN();
#endif
#if CONF_THRESHOLD_GET_ENABLED
            writeThresholdRecord(vrms, variance);
#endif
        }
#endif

#if CONF_SUDDEN_AMPLITUDE_CHANGE_ENABLED
        if (detectSuddenAmplitudeChangeWithDerivative(vrms_values_per_second, VRMS_SAMPLE_SIZE / SAMPLE_SIZE_PER_VRMS_CALC) || amplitude_change_detect_flag) {
            PRINTF("ADC READ TASK: sudden amplitude change detected with Derivate method.\r\n");
            if (amplitude_change_detect_flag) {
                writeSuddenAmplitudeChangeRecordToFlash(&ac_data);
                amplitude_change_detect_flag = 0;
            } else {
                // setAmplitudeChangeParameters(&ac_data, vrms_values_per_second, variance, ADC_FIFO_SIZE, sizeof(vrms_values_per_second));
                amplitude_change_detect_flag = 1;
            }
        }
#endif

        if (current_time.sec == 0) {
            if (current_time.min % load_profile_record_period == 0) {
                PRINTF("ADC READ TASK: minute is multiple of %d. write flash block is running...\r\n", load_profile_record_period);

                if (vrms_buffer_count > VRMS_BUFFER_SIZE) {
                    vrms_buffer_count = VRMS_BUFFER_SIZE;
                }

                VRMS_VALUES_RECORD vrms_values = vrmsSetMinMaxMean(vrms_buffer, vrms_buffer_count);
                PRINTF("ADC READ TASK: calculated VRMS values.\r\n");

                SPIWriteToFlash(&vrms_values);
                PRINTF("ADC READ TASK: writing flash memory process is completed.\r\n");

                memset(vrms_buffer, 0, VRMS_BUFFER_SIZE * sizeof(float));
                vrms_buffer_count = 0;
                PRINTF("ADC READ TASK: buffer content is deleted\r\n");
            }
        }

        watchdog_update();
    }
}

// DEBUG TASK
void vGetRTCTask() {
    TickType_t startTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000);
    startTime = xTaskGetTickCount();

    while (true) {
        vTaskDelayUntil(&startTime, xFrequency);

        rtc_get_datetime(&current_time);
        datetime_to_str(datetime_str, sizeof(datetime_buffer), &current_time);
        PRINTF("---------------------------------------------------------------------------------------------------------\n");
        PRINTF("WRITE DEBUG TASK: The Time is:%s\r\n", datetime_str);
    }
}

void vPowerLedBlinkTask() {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        gpio_put(POWER_LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
        gpio_put(POWER_LED_PIN, 1);
    }
}

void vADCSampleTask() {
    TickType_t startTime;
    const TickType_t xFrequency = 1;
    uint16_t adc_sample;
    uint16_t bias_sample;

    uint16_t bias_buffer[BIAS_SAMPLE_SIZE] = {0};
    uint16_t bias_buffer_count = 0;

    startTime = xTaskGetTickCount();
    while (1) {
        adc_sample = adc_read();

        bool is_added = addToFIFO(&adc_fifo, adc_sample);

        if (!is_added) {
            removeFirstElementAddNewElement(&adc_fifo, adc_sample);
        }

        bias_sample = adc_read();
        bias_buffer[(bias_buffer_count++) % BIAS_SAMPLE_SIZE] = bias_sample;

        if (bias_buffer_count == BIAS_SAMPLE_SIZE) {
            bias_voltage = getMean(bias_buffer, BIAS_SAMPLE_SIZE);
            PRINTF("bias voltage is: %lf\r\n", bias_voltage);
            bias_buffer_count = 0;
            xTaskNotifyGive(xADCHandle);
        }

        vTaskDelayUntil(&startTime, xFrequency);
    }
}

// RESET TASK: This task sends a pulse to reset PIN.
void vResetTask() {
    while (1) {
        getTimePt7c4338(&current_time);
        rtc_set_datetime(&current_time);
        gpio_put(RESET_PULSE_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_put(RESET_PULSE_PIN, 0);
        vTaskDelay(INTERVAL_MS);
    }
}

int main() {
    // POWER LED INIT
    gpio_init(POWER_LED_PIN);
    gpio_set_dir(18, GPIO_OUT);
    gpio_put(POWER_LED_PIN, 1);

    stdio_init_all();
    sleep_ms(2000);

    gpio_set_function(UART0_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART0_RX_PIN, GPIO_FUNC_UART);
    // UART INIT
    if (!initUART()) {
        watchdog_reboot(0, 0, 0);
    }

    // RESET INIT
    gpio_init(RESET_PULSE_PIN);
    gpio_set_dir(RESET_PULSE_PIN, GPIO_OUT);

    // THRESHOLD GPIO INIT
    gpio_init(THRESHOLD_PIN);
    gpio_set_dir(THRESHOLD_PIN, GPIO_OUT);
    gpio_put(THRESHOLD_PIN, 0);

    // ADC INIT
    adc_init();
    adc_gpio_init(ADC_READ_PIN);
    adc_gpio_init(ADC_BIAS_PIN);
    adc_set_round_robin(0x03);

    // I2C Init
    if (!initI2C()) {
        watchdog_reboot(0, 0, 0);
    }
    gpio_set_function(RTC_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(RTC_I2C_SCL_PIN, GPIO_FUNC_I2C);

#if WITHOUT_BOOTLOADER
    addSerialNumber();
#endif

    // sector content control
    checkSectorContent();
    checkThresholdContent();

    // FLASH CONTENTS
    getFlashContents();

    // RTC Init
    rtc_init();

    // Enable Write Protection for this device's firmware (first 256KB of flash memory)
    read_flash_status_registers();
    send_write_enable_command();
    sleep_ms(10);
    send_write_protect_command();
    sleep_ms(10);

    // Get PT7C4338's Time information and set RP2040's RTC module
    if (!getTimePt7c4338(&current_time)) {
        watchdog_reboot(0, 0, 0);
    }

    // If current time which is get from Chip has invalid value, adjust the value.
    if (current_time.dotw < 0 || current_time.dotw > 6) {
        current_time.dotw = 2;
    }

    // set and get datetimes from RP2040 RTC's
    bool is_time_set = rtc_set_datetime(&current_time);
    sleep_ms(100);
    bool is_time_get = rtc_get_datetime(&current_time);

    // if time is get correctly, set string datetime.
    if (is_time_get) {
        datetime_to_str(datetime_str, sizeof(datetime_buffer), &current_time);
    } else {
        PRINTF("Time is not GET. Please check the time setting.\n");
    }

    initADCFIFO(&adc_fifo);

    // set when program started
    setProgramStartDate(&current_time);

    if (!setMutexes()) {
        PRINTF("Failed to set mutexes!\n");
        watchdog_reboot(0, 0, 0);
    }

    watchdog_enable(WATCHDOG_TIMEOUT_MS, 0);

    xUARTMessageBuffer = xMessageBufferCreate(RX_BUFFER_SIZE);
    // if time is set correctly, start the processes.
    if (is_time_set) {
        PRINTF("Time is set. Starting tasks...\n");

        xTaskCreate(vADCReadTask, "ADCReadTask", ADC_READ_TASK_STACK_SIZE, NULL, 5, &xADCHandle);
        xTaskCreate(vADCSampleTask, "ADCSampleTask", ADC_SAMPLE_TASK_STACK_SIZE, NULL, 6, &xADCSampleHandle);

        xTaskCreate(vUARTTask, "UARTTask", UART_TASK_STACK_SIZE, NULL, 4, &xUARTHandle);
        xTaskCreate(vGetRTCTask, "WriteDebugTask", WRITE_DEBUG_TASK_STACK_SIZE, NULL, 5, &xGetRTCHandle);

        xTaskCreate(vResetTask, "ResetTask", RESET_TASK_STACK_SIZE, NULL, 7, &xResetHandle);
        xTaskCreate(vPowerLedBlinkTask, "PowerLedBlinkTask", POWER_BLINK_STACK_SIZE, NULL, 1, NULL);

        vTaskCoreAffinitySet(xADCHandle, 1 << 1);
        vTaskCoreAffinitySet(xADCSampleHandle, 1 << 1);

        vTaskCoreAffinitySet(xUARTHandle, 1 << 0);
        vTaskCoreAffinitySet(xGetRTCHandle, 1 << 0);
        vTaskCoreAffinitySet(xResetHandle, 1 << 0);

        vTaskStartScheduler();
    } else {
        PRINTF("Time is not SET. Please check the time setting.\n");
        watchdog_reboot(0, 0, 0);
    }

    while (true)
        ;
}
