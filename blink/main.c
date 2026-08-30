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
#include "header/threshold_event.h"
#include "header/uart.h"

static uint8_t temp_rx_buf[RX_BUFFER_SIZE];
static volatile size_t rx_index = 0;
static volatile bool waiting_for_bcc = false;
MessageBufferHandle_t xUARTMessageBuffer;
volatile uint8_t task_flags = 0;

void reset_uart_software_buffer() {
    rx_index = 0;
    waiting_for_bcc = false;
    memset(temp_rx_buf, 0, RX_BUFFER_SIZE);
}

void __not_in_flash_func(uart_receive_interrupt_handler)() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    uart_hw_t *uart_hw = uart_get_hw(UART0_ID);

    if (uart_hw->rsr & (UART_UARTRSR_OE_BITS | UART_UARTRSR_BE_BITS | UART_UARTRSR_PE_BITS | UART_UARTRSR_FE_BITS)) {
        uart_hw->rsr = (UART_UARTRSR_OE_BITS | UART_UARTRSR_BE_BITS | UART_UARTRSR_PE_BITS | UART_UARTRSR_FE_BITS);

        while (uart_is_readable(UART0_ID)) {
            (void)uart_getc(UART0_ID);
        }
        rx_index = 0;
        waiting_for_bcc = false;
        return;
    }

    if (!uart_is_readable(UART0_ID)) {
        led_blink_pattern(LED_ERROR_CODE_UART_NOT_READABLE, false);
    }

    while (uart_is_readable(UART0_ID)) {
        uint8_t ch = uart_getc(UART0_ID);

        if (rx_index < RX_BUFFER_SIZE - 1) {
            temp_rx_buf[rx_index++] = ch;
        } else {
            led_blink_pattern(LED_ERROR_CODE_RX_BUFFER_OVERFLOW_ISR, false);
            rx_index = 0;
            waiting_for_bcc = false;
            return;
        }

        if (waiting_for_bcc) {
            waiting_for_bcc = false;
            temp_rx_buf[rx_index] = '\0';
            xMessageBufferSendFromISR(
                xUARTMessageBuffer,
                temp_rx_buf,
                rx_index,
                &xHigherPriorityTaskWoken);
            rx_index = 0;
            continue;
        }

        if (ch == LINE_FEED) {
            temp_rx_buf[rx_index] = '\0';
            xMessageBufferSendFromISR(
                xUARTMessageBuffer,
                temp_rx_buf,
                rx_index,
                &xHigherPriorityTaskWoken);
            rx_index = 0;
        } else if (ch == ETX_CHAR) {
            waiting_for_bcc = true;
        }
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// UART Initialization
uint8_t initUART() {
    gpio_set_function(UART0_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART0_RX_PIN, GPIO_FUNC_UART);

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

#if !CONF_THRESHOLD_PIN_ENABLED
void vStatusLedTask() {
    uint16_t step_index = 0;
    uint16_t tick_count = 0;
    int last_pattern_id = -1;
    const TickType_t xFrequency = pdMS_TO_TICKS(2);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (current_pattern_id != last_pattern_id) {
            last_pattern_id = current_pattern_id;
            step_index = 0;
            tick_count = 0;
            gpio_put(STATUS_LED_PIN, 1);
        }

        const LedPattern *p = &patterns[current_pattern_id];

        if (tick_count >= p->sequence[step_index]) {
            tick_count = 0;
            step_index++;
            if (step_index >= p->length) {
                if (play_once) {
                    play_once = false;
                    current_pattern_id = 0;
                }
                step_index = 0;
            }

            gpio_put(STATUS_LED_PIN, (step_index % 2) == 0 ? 1 : 0);
        }

        tick_count++;
    }
}
#endif

void vUARTTask() {
    uint8_t rx_buffer[RX_BUFFER_SIZE];
    size_t received_bytes;
    char identify_response_buf[IDENTIFICATION_RESPONSE_BUFFER_SIZE];
    size_t identify_response_len = 0;
    uint8_t message_retry_count = 0;
    uint8_t hex_baud_rate = 0;
    int8_t requested_mode = -1;

    while (1) {
        // 2s receive timeout (intentionally < watchdog 5s check period)
        // avoids resonance race where both tasks wake on the same tick.
        received_bytes = xMessageBufferReceive(
            xUARTMessageBuffer,
            rx_buffer,
            sizeof(rx_buffer),
            pdMS_TO_TICKS(2000));

        // Task is alive AND UART IRQ is enabled — both required for healthy UART pipeline
        if (irq_is_enabled(UART_IRQ_NUM(UART0_ID))) {
            taskENTER_CRITICAL();
            task_health_flags |= WDT_FLAG_UART;
            taskEXIT_CRITICAL();
        }

        if (received_bytes > 0) {
            PRINTF("---> %.*s\n", received_bytes, rx_buffer);

            vTaskDelay(pdMS_TO_TICKS(250));
            if (control_serial_number(rx_buffer, received_bytes) == true) {
                identify_response_len = create_identify_response_message(identify_response_buf, sizeof(identify_response_buf));

                if (identify_response_len >= sizeof(identify_response_buf)) {
                    PRINTF("Identification response buffer overflow!\n");
                    sendErrorMessage((char *)"IDRESPONSEBUFOVERFLOW");
                    continue;
                }

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
                    led_blink_pattern(LED_ERROR_CODE_MESSAGE_TIMEOUT, false);
                    continue;
                }

                vTaskDelay(pdMS_TO_TICKS(250));
                message_retry_count = 0;
                hex_baud_rate = exract_baud_rate_and_mode_from_message(rx_buffer, received_bytes, &requested_mode);
                set_device_baud_rate(hex_baud_rate);

                if (requested_mode == REQUEST_MODE_LONG_READ || requested_mode == REQUEST_MODE_SHORT_READ) {
                    PRINTF("Request is readout\n");
                    send_readout_message(requested_mode);
                    set_init_baud_rate();
                    continue;
                } else if (requested_mode == REQUEST_MODE_PROGRAMMING) {
                    PRINTF("Request is programming mode\n");
                    send_programming_acknowledgement();
                } else {
                    PRINTF("Request mode is invalid, ignoring message.\n");
                    set_init_baud_rate();
                    led_blink_pattern(LED_ERROR_CODE_INVALID_REQUEST_MODE, false);
                    continue;
                }

                // IEC62056-21: total 30s silence ends programming mode.
                // Implemented as 3s receive chunks + cumulative tracking, so
                // the watchdog flag stays fresh under the 8s HW WDT limit.
                TickType_t inner_idle_start = xTaskGetTickCount();
                while (1) {
                    received_bytes = xMessageBufferReceive(
                        xUARTMessageBuffer,
                        rx_buffer,
                        sizeof(rx_buffer),
                        pdMS_TO_TICKS(3000));

                    // Heartbeat: task alive AND UART IRQ functional
                    if (irq_is_enabled(UART_IRQ_NUM(UART0_ID))) {
                        taskENTER_CRITICAL();
                        task_health_flags |= WDT_FLAG_UART;
                        taskEXIT_CRITICAL();
                    }

                    if (received_bytes == 0) {
                        // No message in this 3s chunk — keep waiting unless
                        // 30s cumulative silence elapsed (IEC end-of-session).
                        if ((xTaskGetTickCount() - inner_idle_start) < pdMS_TO_TICKS(30000)) {
                            continue;
                        }
                        // 30s silence → fall through to exit logic
                    } else {
                        // Activity → reset cumulative idle timer
                        inner_idle_start = xTaskGetTickCount();
                    }

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
                            send_load_profile_records(rx_buffer);
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
#if CONF_THRESHOLD_ENABLED
                        case SetThreshold:
                            setThresholdValue(rx_buffer);
                            break;
#endif
#if CONF_THRESHOLD_PIN_ENABLED
                        case ThresholdPin:
                            resetThresholdPIN();
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
            } else {
                PRINTF("SN is invalid, ignoring message.\n");
                led_blink_pattern(LED_ERROR_CODE_INVALID_SERIAL_NUMBER, true);
            }
        }
        // 2s timeout in idle is the normal heartbeat tick — no log here.
    }
}

#if CONF_THRESHOLD_ENABLED || CONF_THRESHOLD_PIN_ENABLED
// Esik degeri protokolde VOLT (tamsayi) olarak tutulur; ic hesap santivolttur.
static uint16_t thresholdVoltsToCv(uint16_t volts) {
    uint32_t cv = (uint32_t)volts * 100u;

    return (cv > 65535u) ? 65535u : (uint16_t)cv;
}
#endif

#if CONF_THRESHOLD_ENABLED
// --- Esik olay mantigi -------------------------------------------------------
// Saniyelik VRMS degerleri bir dakikalik pencerede toplanir, pencerenin MEDYANI
// karar istatistigi olarak kullanilir (ortalama tek bir sicramadan etkilenir,
// medyan etkilenmez). Flash'a saniyelik ornek degil, OLAY yazilir.

static th_state_t th_state;
static uint16_t th_window[TH_WINDOW_MAX_SAMPLES];
static uint16_t th_window_count = 0;
static int8_t th_last_min = -1;

static uint16_t thresholdReleaseCv(uint16_t enter_cv) {
    return (enter_cv > TH_HYSTERESIS_CV) ? (uint16_t)(enter_cv - TH_HYSTERESIS_CV) : 0u;
}

// RTC kurulmamisken kaydedilen bir olay "en son ne zaman ariza vardi" sorusuna
// 2000 yilindan bir tarih dondurur ve listenin tamamini guvenilmez yapar.
static bool isRTCTimeValid(void) {
    return current_time.year >= TH_RTC_MIN_VALID_YEAR && current_time.year <= 99 &&
           current_time.month >= 1 && current_time.month <= 12 &&
           current_time.day >= 1 && current_time.day <= 31;
}

static void thWriteRecord(const th_record_t *rec, const char *what) {
    struct ThresholdData data;

    setDateToCharArray(rec->time.year, data.year);
    setDateToCharArray(rec->time.month, data.month);
    setDateToCharArray(rec->time.day, data.day);
    setDateToCharArray(rec->time.hour, data.hour);
    setDateToCharArray(rec->time.min, data.min);
    setDateToCharArray(rec->time.sec, data.sec);
    data.vrms = rec->vrms_cv;
    data.duration = rec->duration;

    PRINTF("THRESHOLD EVENT: %s %02d-%02d-%02d %02d:%02d:%02d vrms=%u.%02u V sure=%u dk slot=%u/%u\r\n",
           what, rec->time.year, rec->time.month, rec->time.day,
           rec->time.hour, rec->time.min, rec->time.sec,
           (unsigned)(rec->vrms_cv / 100u), (unsigned)(rec->vrms_cv % 100u),
           (unsigned)rec->duration,
           (unsigned)getThresholdWriteIndex(), (unsigned)TH_RECORD_SLOT_COUNT);

    writeThresholdRecord(&data);
}

static void thProcessWindow(uint16_t threshold_cv) {
    th_record_t rec;
    th_time_t now;
    uint16_t median_cv = thMedianCv(th_window, th_window_count);

    // Esik UART'tan degistirilmis olabilir. Suren bir olayi bozmadan guncelle.
    if (th_state.enter_cv != threshold_cv) {
        th_state.enter_cv = threshold_cv;
        th_state.release_cv = thresholdReleaseCv(threshold_cv);
    }

    now.year = current_time.year;
    now.month = current_time.month;
    now.day = current_time.day;
    now.hour = current_time.hour;
    now.min = current_time.min;
    now.sec = current_time.sec;

    th_action_t action = thEventWindow(&th_state, median_cv, &now, &rec);

    PRINTF("THRESHOLD: pencere medyani=%u.%02u V (%u ornek) esik=%u.%02u V%s\r\n",
           (unsigned)(median_cv / 100u), (unsigned)(median_cv % 100u),
           (unsigned)th_window_count,
           (unsigned)(threshold_cv / 100u), (unsigned)(threshold_cv % 100u),
           th_state.active ? " [OLAY AKTIF]" : "");

    if (action == TH_ACTION_NONE) {
        return;
    }

    if (!isRTCTimeValid()) {
        PRINTF("THRESHOLD EVENT: RTC kurulmamis (yil=%d), kayit atlandi\r\n", current_time.year);
        return;
    }

    switch (action) {
    case TH_ACTION_EVENT_STARTED:
        thWriteRecord(&rec, "BASLADI");
        break;
    case TH_ACTION_EVENT_ONGOING:
        thWriteRecord(&rec, "SURUYOR");
        break;
    case TH_ACTION_EVENT_ENDED:
        thWriteRecord(&rec, "BITTI  ");
        break;
    default:
        break;
    }
}
#endif

void vADCReadTask() {
#if CONF_SUDDEN_AMPLITUDE_CHANGE_ENABLED
    struct AmplitudeChangeTimerCallbackParameters ac_data = {0};
    uint8_t amplitude_change_detect_flag = 0;
#endif
    uint16_t adc_samples_buffer[VRMS_SAMPLE_SIZE];
    float vrms_values_per_second[VRMS_SAMPLE_SIZE / SAMPLE_SIZE_PER_VRMS_CALC];
    uint16_t vrms_buffer_count = 0;
    float vrms_buffer[VRMS_BUFFER_SIZE] = {0};

#if CONF_THRESHOLD_ENABLED
    uint16_t initial_threshold_cv = thresholdVoltsToCv(getVRMSThresholdValue());
    thEventInit(&th_state, initial_threshold_cv, thresholdReleaseCv(initial_threshold_cv),
                TH_ENTER_WINDOWS, TH_EXIT_WINDOWS, TH_HEARTBEAT_WINDOWS);
    PRINTF("THRESHOLD: olay mantigi kuruldu. giris=%u.%02u V cikis=%u.%02u V "
           "(giris %d pencere, cikis %d pencere, ara kayit %d dk)\r\n",
           (unsigned)(th_state.enter_cv / 100u), (unsigned)(th_state.enter_cv % 100u),
           (unsigned)(th_state.release_cv / 100u), (unsigned)(th_state.release_cv % 100u),
           TH_ENTER_WINDOWS, TH_EXIT_WINDOWS, TH_HEARTBEAT_WINDOWS);
#endif

    while (1) {
        uint32_t ulNotificationValue = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000));

        if (ulNotificationValue == 0) {
            PRINTF("ADC READ TASK: No notification received from ADC SAMPLE TASK within timeout.\r\n");
            continue;
        }

        taskENTER_CRITICAL();
        task_health_flags |= WDT_FLAG_ADC_READ;
        taskEXIT_CRITICAL();

        if (xSemaphoreTake(xFIFOMutex, pdMS_TO_TICKS(250)) == pdTRUE) {
            getLastNElementsToBuffer(&adc_fifo, adc_samples_buffer, VRMS_SAMPLE_SIZE);
            xSemaphoreGive(xFIFOMutex);
        } else {
            PRINTF("ADC READ TASK: FIFO MUTEX CANNOT BE TAKEN!\r\n");
            led_blink_pattern(LED_ERROR_CODE_FIFO_MUTEX_NOT_TAKEN, false);
            if (current_time.sec == 0 && current_time.min % load_profile_record_period == 0) {
                memset(vrms_buffer, 0, VRMS_BUFFER_SIZE * sizeof(float));
                vrms_buffer_count = 0;
                PRINTF("ADC READ TASK: buffer content is deleted\r\n");
            }
            continue;
        }

        float vrms = calculateVRMS(adc_samples_buffer, VRMS_SAMPLE_SIZE, bias_voltage);
        PRINTF("vrms is: %lf\r\n", vrms);

        float main_mean = getMean(adc_samples_buffer, VRMS_SAMPLE_SIZE);
#if CONF_BIAS_SAMPLING_ENABLED
        float ch_diff = main_mean - bias_voltage;
        PRINTF("TESHIS: CH0_ort=%.1f BIAS_ort=%.1f fark=%.1f sayim (%.3f V hat)\r\n",
               main_mean, bias_voltage, ch_diff,
               ch_diff * (3.28f / (1 << 12)) * VRMS_MULTIPLICATION_VALUE);
#else
        PRINTF("TESHIS: CH0_ort=%.1f (bias ornekleme kapali)\r\n", main_mean);
#endif

        calculateVRMSValuesPerSecond(vrms_values_per_second, adc_samples_buffer, VRMS_SAMPLE_SIZE, SAMPLE_SIZE_PER_VRMS_CALC, bias_voltage);

        vrms_buffer[(vrms_buffer_count++) % VRMS_BUFFER_SIZE] = vrms;

#if CONF_THRESHOLD_PIN_ENABLED || CONF_THRESHOLD_ENABLED
        uint16_t vrms_cv = thVoltsToCv(vrms);
        uint16_t threshold_cv = thresholdVoltsToCv(getVRMSThresholdValue());

#if CONF_THRESHOLD_PIN_ENABLED
        // Pin anlik gostergedir, saniyelik kalir; flash'a yazilan sey artik olaydir.
        if (vrms_cv >= threshold_cv) {
            setThresholdPIN();
        }
#endif

#if CONF_THRESHOLD_ENABLED
        if (th_window_count < TH_WINDOW_MAX_SAMPLES) {
            th_window[th_window_count++] = vrms_cv;
        }

        // Pencere RTC dakikasina hizalidir. sec == 0 yerine dakika DEGISIMINE
        // bakiyoruz; gorev jitter'i bir saniyeyi kacirsa bile pencere kapanir.
        if (current_time.min != th_last_min) {
            if (th_last_min >= 0 && th_window_count > 0) {
                thProcessWindow(threshold_cv);
            }

            th_last_min = current_time.min;
            th_window_count = 0;
        }
#endif
#endif

#if CONF_SUDDEN_AMPLITUDE_CHANGE_ENABLED
        if (detectSuddenAmplitudeChangeWithDerivative(vrms_values_per_second, VRMS_SAMPLE_SIZE / SAMPLE_SIZE_PER_VRMS_CALC) || amplitude_change_detect_flag) {
            PRINTF("ADC READ TASK: sudden amplitude change detected with Derivate method.\r\n");
            if (amplitude_change_detect_flag) {
                writeSuddenAmplitudeChangeRecordToFlash(&ac_data);
                amplitude_change_detect_flag = 0;
            } else {
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

                uint32_t uptime_ms = to_ms_since_boot(get_absolute_time());
                uint32_t guard_ms = (uint32_t)load_profile_record_period * 60u * 1000u;

                if (uptime_ms + guard_ms < ESTIMATE_RESET_MS) {
                    SPIWriteToFlash(&vrms_values);
                    PRINTF("ADC READ TASK: writing flash memory process is completed.\r\n");
                } else {
                    PRINTF("ADC READ TASK: reset window is close (uptime=%lu ms), load profile skipped.\r\n", uptime_ms);
                }

                memset(vrms_buffer, 0, VRMS_BUFFER_SIZE * sizeof(float));
                vrms_buffer_count = 0;
                PRINTF("ADC READ TASK: buffer content is deleted\r\n");
            }
        }
    }
}

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

void vADCSampleTask() {
    TickType_t startTime;
    const TickType_t xFrequency = 1;
    uint16_t adc_sample;
    uint16_t sample_count = 0;

#if CONF_BIAS_SAMPLING_ENABLED
    uint16_t bias_sample;
    uint16_t bias_buffer[BIAS_SAMPLE_SIZE] = {0};
    uint16_t bias_buffer_count = 0;
#endif

    startTime = xTaskGetTickCount();
    while (1) {
        adc_sample = adc_read();

        bool is_added = addToFIFO(&adc_fifo, adc_sample);

        if (!is_added) {
            removeFirstElementAddNewElement(&adc_fifo, adc_sample);
        }

#if CONF_BIAS_SAMPLING_ENABLED
        // Round-robin acik oldugu icin bu ikinci donusum CH1'e (bias) denk gelir.
        bias_sample = adc_read();
        bias_buffer[(bias_buffer_count++) % BIAS_SAMPLE_SIZE] = bias_sample;

        if (bias_buffer_count == BIAS_SAMPLE_SIZE) {
            bias_voltage = getMean(bias_buffer, BIAS_SAMPLE_SIZE);
            PRINTF("bias voltage is: %lf\r\n", bias_voltage);
            bias_buffer_count = 0;
        }
#endif

        // Pencere sayaci: bias ornekleme kapaliyken de okuma gorevi ayni ritimde
        // (VRMS_SAMPLE_SIZE ornek = 1 s) tetiklenmeli.
        if (++sample_count >= VRMS_SAMPLE_SIZE) {
            sample_count = 0;

            taskENTER_CRITICAL();
            task_health_flags |= WDT_FLAG_ADC_SAMPLE;
            taskEXIT_CRITICAL();

            xTaskNotifyGive(xADCHandle);
        }

        vTaskDelayUntil(&startTime, xFrequency);
    }
}


void vWatchdogTask() {
    const TickType_t xCheckInterval = pdMS_TO_TICKS(WATCHDOG_CHECK_PERIOD_MS); // 2 saniyede bir kontrol et

    while (1) {
        vTaskDelay(xCheckInterval);

        if ((task_health_flags & WDT_ALL_TASKS_OK) == WDT_ALL_TASKS_OK) {
            watchdog_update();

            taskENTER_CRITICAL();
            task_health_flags = 0;
            taskEXIT_CRITICAL();
        } else {
            PRINTF("WDT: System UNHEALTHY! Flags: %02lX (Expected: %02X)\n", task_health_flags, WDT_ALL_TASKS_OK);
        }
    }
}

void init_status_led_or_threshold_pin() {
#if CONF_THRESHOLD_PIN_ENABLED
    gpio_init(THRESHOLD_PIN);
    gpio_set_dir(THRESHOLD_PIN, GPIO_OUT);
    gpio_put(THRESHOLD_PIN, 0);
#else
    gpio_init(STATUS_LED_PIN);
    gpio_set_dir(STATUS_LED_PIN, GPIO_OUT);
    gpio_put(STATUS_LED_PIN, 1);
#endif
}

void init_reset_pin() {
    gpio_init(RESET_PULSE_PIN);
    gpio_set_dir(RESET_PULSE_PIN, GPIO_OUT);
    gpio_put(RESET_PULSE_PIN, 0);
}

void init_adc() {
    adc_init();
    adc_gpio_init(ADC_READ_PIN);
    adc_gpio_init(ADC_BIAS_PIN);

#if CONF_BIAS_SAMPLING_ENABLED
    adc_set_round_robin((1u << ADC_VRMS_SAMPLE_INPUT) | (1u << ADC_BIAS_INPUT));
#else
    // Rotasyonda tek kanal: her donusum CH0. Ornek-tut kondansatoru bias
    // gerilimine hic ugramadigi icin kanallar arasi tasima ortadan kalkar.
    adc_select_input(ADC_VRMS_SAMPLE_INPUT);
    adc_set_round_robin(1u << ADC_VRMS_SAMPLE_INPUT);
#endif
}

int main() {
    init_status_led_or_threshold_pin();

    watchdog_enable(WATCHDOG_TIMEOUT_MS, 0);

    if (!stdio_init_all()) {
        watchdog_reboot(0, 0, 0);
    }
    sleep_ms(2000);

    if (!initUART()) {
        PRINTF("UART Init fail! Restarting...\n");
        watchdog_reboot(0, 0, 0);
    }

    init_reset_pin();
    init_adc();

    if (!initI2C()) {
        watchdog_reboot(0, 0, 0);
    }
    gpio_set_function(RTC_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(RTC_I2C_SCL_PIN, GPIO_FUNC_I2C);

#if WITHOUT_BOOTLOADER
    addSerialNumber();
#endif

    checkSectorContent();
    checkThresholdContent();

    getFlashContents();

    rtc_init();

    read_flash_status_registers();
    send_write_enable_command();
    sleep_ms(10);
    send_write_protect_command();
    sleep_ms(10);

    if (!getTimePt7c4338(&current_time)) {
        watchdog_reboot(0, 0, 0);
    }

    if (current_time.dotw < 0 || current_time.dotw > 6) {
        current_time.dotw = 2;
    }

    bool is_time_set = rtc_set_datetime(&current_time);
    sleep_ms(100);
    bool is_time_get = rtc_get_datetime(&current_time);

    if (is_time_get) {
        datetime_to_str(datetime_str, sizeof(datetime_buffer), &current_time);
    } else {
        PRINTF("Time is not GET. Please check the time setting.\n");
        watchdog_reboot(0, 0, 0);
    }

    initADCFIFO(&adc_fifo);

    setProgramStartDate(&current_time);

    if (!setMutexes()) {
        PRINTF("Failed to set mutexes!\n");
        watchdog_reboot(0, 0, 0);
    }

    xUARTMessageBuffer = xMessageBufferCreate(RX_BUFFER_SIZE);
    if (is_time_set) {
        PRINTF("Time is set. Starting tasks...\n");

        xTaskCreate(vADCReadTask, "ADCReadTask", ADC_READ_TASK_STACK_SIZE, NULL, 5, &xADCHandle);
        xTaskCreate(vADCSampleTask, "ADCSampleTask", ADC_SAMPLE_TASK_STACK_SIZE, NULL, 6, &xADCSampleHandle);

        xTaskCreate(vUARTTask, "UARTTask", UART_TASK_STACK_SIZE, NULL, 4, &xUARTHandle);
        xTaskCreate(vGetRTCTask, "WriteDebugTask", WRITE_DEBUG_TASK_STACK_SIZE, NULL, 5, &xGetRTCHandle);

#if !CONF_THRESHOLD_PIN_ENABLED
        xTaskCreate(vStatusLedTask, "StatusLedTask", configMINIMAL_STACK_SIZE, NULL, 1, &xStatusLedHandle);
#endif
        xTaskCreate(vWatchdogTask, "WatchdogTask", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, &xWatchdogHandle);

        vTaskCoreAffinitySet(xADCHandle, 1 << 1);
        vTaskCoreAffinitySet(xADCSampleHandle, 1 << 1);

        vTaskCoreAffinitySet(xUARTHandle, 1 << 0);
        vTaskCoreAffinitySet(xGetRTCHandle, 1 << 0);
#if !CONF_THRESHOLD_PIN_ENABLED
        vTaskCoreAffinitySet(xStatusLedHandle, 1 << 0);
#endif

        vTaskStartScheduler();
    } else {
        PRINTF("Time is not SET. Please check the time setting.\n");
        watchdog_reboot(0, 0, 0);
    }

    while (true)
        ;
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    (void) pcTaskName;
    (void) xTask;
    led_blink_pattern(LED_ERROR_CODE_STACK_OVERFLOW, true);
    watchdog_reboot(0, 0, 5000);
}
