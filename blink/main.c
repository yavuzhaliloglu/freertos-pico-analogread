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

// UART DEFINES
#define ENTRY_MAGIC 0xb105f00d
#define UART0_ID uart0 // UART0 for RS485
#define BAUD_RATE 300
#define UART0_TX_PIN 0
#define UART0_RX_PIN 1
#define DATA_BITS 7
#define STOP_BITS 1
#define PARITY UART_PARITY_EVEN
#define UART_TASK_PRIORITY 3
#define UART_TASK_STACK_SIZE (1024 * 3)

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
uint8_t time_change_flag;


// UART VARIABLES
enum States
{
    Greeting = 0,
    Setting = 1,
    Listening = 2,
    WriteProgram = 3
};
enum ListeningStates
{
    DataError = -1,
    Reading = 0,
    TimeSet = 1,
    DateSet = 2,
    ReProgram = 3
};
volatile TaskHandle_t xTaskToNotify_UART = NULL;
enum States state = Greeting;
uint16_t max_baud_rate = 9600;
uint8_t rx_buffer[256] = {};
uint8_t rx_buffer_len = 0;
uint8_t reading_state_start_time[10];
uint8_t reading_state_end_time[10];
volatile uint8_t test_flag = 0;

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

void readENUART()
{
    gpio_put(3, 0);
}

void writeENUART()
{
    gpio_put(3, 1);
    vTaskDelay(1);
}



enum ListeningStates checkListeningData(uint8_t *data_buffer, uint8_t size)
{
    uint8_t reprogram[4] = {0x21, 0x21, 0x21, 0x21};
    if (strncmp(reprogram, data_buffer, 4) == 0)
        return ReProgram;

    if (!(bccControl(data_buffer, size)))
    {
        printf("bcc control error.");
        return DataError;
    }

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



void printBufferHex(uint8_t *buf, size_t len)
{
    for (size_t i = 0; i < len; ++i)
    {
        printf("%02X", buf[i]);
        if (i % 16 == 15)
            printf("\n");
        else
            printf(" ");

        if (i % 256 == 0 && i != 0)
        {
            printf("\n\n");
            printf("page %d\n", i / 256);
        }
    }
}

void __not_in_flash_func(rebootDevice)()
{
    uint8_t *old_prg_offset = (uint8_t *)(XIP_BASE);
    uint8_t *new_prg_offset = (uint8_t *)(XIP_BASE + FLASH_REPROGRAM_OFFSET);
    uint8_t *ram_offset = (uint8_t *)(0x200000c0);

    uint32_t ints = save_and_disable_interrupts();
    printf("1");
    flash_range_erase(16 * 1024, 240 * 1024);
    printf("2");

    for (int i = 0; i < ota_block_count; i++)
    {
        memcpy(rpb, new_prg_offset, FLASH_RPB_BLOCK_SIZE);
        flash_range_program(16 * 1024 + (i * FLASH_RPB_BLOCK_SIZE), rpb, FLASH_RPB_BLOCK_SIZE);
        new_prg_offset += FLASH_RPB_BLOCK_SIZE;
        sleep_ms(1);
    }
    restore_interrupts(ints);

    printf("old area:\n");
    printBufferHex(old_prg_offset, 2048);
    printf("new area:\n");
    printBufferHex(new_prg_offset, 2048);

    hw_clear_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_ENABLE_BITS);
    watchdog_hw->scratch[5] = ENTRY_MAGIC;
    watchdog_hw->scratch[6] = ~ENTRY_MAGIC;
    watchdog_reboot(0, 0, 0);
}

void setReProgramSize(uint8_t *data_buffer)
{
    uint8_t lsb_byte = data_buffer[7];

    for (int i = 4; i < 7; i++)
    {
        uint8_t lsb = lsb_byte & 0x01;
        data_buffer[i] = (data_buffer[i] << 1);
        data_buffer[i] += lsb;
        lsb_byte = (lsb_byte >> 1);

        reprogram_size += data_buffer[i];
        if (i != 6)
        {
            reprogram_size = (reprogram_size << 8);
        }
    }
    uart_putc(UART0_ID, 0x06);
    printf("reprogram size: %u\n", reprogram_size);
}

void setBCC(uint8_t *buffer, uint8_t size, uint8_t xor)
{
    for (int i = 0; i < size; i++)
        xor ^= buffer[i];

    printf("xor result in function is: %02X", xor);

    buffer[size - 1] = xor;
}

void resetRxBuffer()
{
    memset(rx_buffer, 0, 256);
    rx_buffer_len = 0;
    printf("reset Buffer func!\n");
}

void resetState()
{
    state = Greeting;
    memset(rx_buffer, 0, 256);
    setProgramBaudRate(0);
    rx_buffer_len = 0;
    printf("reset state func!\n");
}

void greetingStateHandler(uint8_t *buffer, uint8_t size)
{

    if (((buffer[0] == 0x2F) && (buffer[1] == 0x3F) && (buffer[size - 3] == 0x21) && (buffer[size - 2] == 0x0D) && (buffer[size - 1] == 0x0A)) || ((buffer[0] == 0x2F) && (buffer[1] == 0x3F) && (buffer[2] == 0x41) && (buffer[3] == 0x4C) && (buffer[4] == 0x50) && (buffer[5] == 0x36) && (buffer[6] == 0x30) && (buffer[size - 3] == 0x21) && (buffer[size - 2] == 0x0D) && (buffer[size - 1] == 0x0A)))
    {
        printf("greeting entered.\n");
        uint8_t program_baud_rate = getProgramBaudRate(max_baud_rate);
        char greeting_uart_buffer[21] = {0};

        snprintf(greeting_uart_buffer, 20, "/ALP%d<1>MAVIALPV2\r\n", program_baud_rate);

        writeENUART();
        uart_puts(UART0_ID, greeting_uart_buffer);
        readENUART();

        state = Setting;
        printf("state is setting.\n");
    }
}

void settingStateHandler(uint8_t *buffer, uint8_t size)
{
    printf("setting state entered.\n");

    writeENUART();
    uint8_t load_profile[3] = {0x31, 0x0D, 0x0A};
    uint8_t meter_read[3] = {0x30, 0x0D, 0x0A};
    uint8_t default_control[2] = {0x06, 0x30};

    if ((strncmp(buffer, default_control, 2) == 0) && (size == 6))
    {
        printf("default entered.\n");

        uint8_t modem_baud_rate;
        uint8_t i;
        uint8_t program_baud_rate = getProgramBaudRate(max_baud_rate);

        for (i = 0; i < size; i++)
        {
            if (buffer[i] == '0')
            {
                modem_baud_rate = buffer[i + 1] - '0';
                if (modem_baud_rate > 5 && modem_baud_rate < 0)
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
                    printf("modem baud rate = %d\n", modem_baud_rate);
                    setProgramBaudRate(modem_baud_rate);
                }
                break;
            }
        }
        if (i == size)
        {
            uart_putc(UART0_ID, 0x15);
        }

        // Load Profile ([ACK]0Z1[CR][LF])
        if (strncmp(load_profile, (buffer + 3), 3) == 0)
        {
            uint8_t ack_buff[17] = {0};
            snprintf(ack_buff, 16, "%cP0%c(60616161)%c", 0x01, 0x02, 0x03);
            setBCC(ack_buff, 16, 0x01);

            uart_puts(UART0_ID, ack_buff);
            printf("ack sent.\n");

            printf("state is listening.\n");
            state = Listening;
        }

        // Read Out ([ACK]0Z0[CR][LF])
        if (strncmp(meter_read, (buffer + 3), 3) == 0)
        {
            printf("meter read entered.\n");
            uint8_t mread_data_buff[55] = {0};

            snprintf(mread_data_buff, 55, "%c0.0.0(60616161)\r\n0.9.1(%02d:%02d:%02d)\r\n0.9.2(%02d-%02d-%02d)\r\n!%c", 0x02, current_time.hour, current_time.min, current_time.sec, current_time.year, current_time.month, current_time.day, 0X03);
            uint8_t bcc = bccCreate(mread_data_buff, 55, 0x02);
            mread_data_buff[54] = bcc;

            for (int i = 0; i < 55; i++)
            {
                printf("%02X ", mread_data_buff[i]);
                if (i % 18 == 0 && i != 0)
                    printf("\n");
                vTaskDelay(1);
            }

            uart_puts(UART0_ID, mread_data_buff);
        }
    }
    readENUART();
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

    rtc_get_datetime(&current_time);
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

    rtc_get_datetime(&current_time);
    setTimePt7c4338(I2C_PORT, I2C_ADDRESS, (uint8_t)current_time.sec, (uint8_t)current_time.min, (uint8_t)current_time.hour, (uint8_t)current_time.dotw, day, month, year);
    getTimePt7c4338(&current_time);
    rtc_set_datetime(&current_time);
}

bool controlRXBuffer(uint8_t *buffer, uint8_t len)
{
    uint8_t reading[8] = {0x01, 0x52, 0x32, 0x02, 0x50, 0x2E, 0x30, 0x31};
    uint8_t reading_all[11] = {0x01, 0x52, 0x32, 0x02, 0x50, 0x2E, 0x30, 0x31, 0x28, 0x3B, 0x29};
    uint8_t time[9] = {0x01, 0x52, 0x32, 0x02, 0x30, 0x2E, 0x39, 0x2E, 0x31};
    uint8_t date[9] = {0x01, 0x52, 0x32, 0x02, 0x30, 0x2E, 0x39, 0x2E, 0x32};
    uint8_t reprogram[4] = {0x21, 0x21, 0x21, 0x21};

    uint8_t reprogram_len = 8;
    uint8_t reading_len = 33;
    uint8_t reading_all_len = 13;
    uint8_t time_len = 19;
    uint8_t date_len = 19;

    if ((len == reprogram_len) && (strncmp(buffer, reprogram, 4) == 0))
        return 1;
    if (((len == reading_len) && (strncmp(buffer, reading, 8) == 0)) || ((len == reading_all_len) && (strncmp(buffer, reading_all, 11) == 0)))
        return 1;
    if ((len == time_len) && (strncmp(buffer, time, 9) == 0))
        return 1;
    if ((len == date_len) && (strncmp(buffer, date, 9) == 0))
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
    flash_range_erase(FLASH_REPROGRAM_OFFSET, 256 * 1024);
    restore_interrupts(ints);
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
    datetime_t dt_start = {0};
    datetime_t dt_end = {0};
    int32_t start_index = -1;
    int32_t end_index = -1;
    char load_profile_line[32] = {0};
    uint8_t *flash_start_content = (uint8_t *)(XIP_BASE + FLASH_DATA_OFFSET);

    printf("rx buffer len: %d\n", rx_buffer_len);

    if (rx_buffer_len == 14)
    {
        for (int i = 0; i < FLASH_TOTAL_RECORDS; i += 16)
        {
            if ((start_index != -1) && (flash_start_content[i] == 0xFF || flash_start_content[i] == 0x00))
            {
                printf("end index entered.\n");
                arrayToDatetime(&end, &flash_start_content[i - 16]);
                end_index = i - 16;
                break;
            }

            if (flash_start_content[i] == 0xFF || flash_start_content[i] == 0x00)
            {
                printf("empty rec entered.\n");
                continue;
            }

            if (start_index == -1)
            {
                printf("start index entered.\n");
                arrayToDatetime(&start, &flash_start_content[i]);
                start_index = i;
            }
        }
    }
    else
    {
        arrayToDatetime(&start, reading_state_start_time);
        arrayToDatetime(&end, reading_state_end_time);

        if (datetimeComp(&start, &end) > 0)
            return;

        for (uint32_t i = 0; i < FLASH_TOTAL_RECORDS; i += 16)
        {
            datetime_t recurrent_time = {0};

            if (flash_start_content[i] == 0xFF || flash_start_content[i] == 0x00)
                continue;

            arrayToDatetime(&recurrent_time, &flash_start_content[i]);

            if (datetimeComp(&recurrent_time, &start) >= 0)
            {
                if (start_index == -1 || (datetimeComp(&recurrent_time, &dt_start) < 0))
                {
                    start_index = i;
                    datetimeCopy(&recurrent_time, &dt_start);
                }
            }

            if (datetimeComp(&recurrent_time, &end) <= 0)
            {
                if (end_index == -1 || datetimeComp(&recurrent_time, &dt_end) > 0)
                {
                    end_index = i;
                    datetimeCopy(&recurrent_time, &dt_end);
                }
            }
        }
    }

    writeENUART();
    if (start_index >= 0 && end_index >= 0)
    {
        uint8_t xor_result = 0x02;
        uint32_t start_addr = start_index;
        uint8_t first_flag = 0;
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
                printf("1\n");

                if (!first_flag)
                {
                    printf("2\n");
                    snprintf(load_profile_line, 31, "%c(%s%s%s%s%s)(%03d,%03d,%03d)\r\n%c", 0x02, year, month, day, hour, minute, min, max, mean, 0x03);
                    first_flag = 1;
                    xor_result = bccCreate(load_profile_line, 32, xor_result);
                    load_profile_line[30] = xor_result;
                }
                else
                {
                    printf("3\n");
                    snprintf(load_profile_line, 30, "(%s%s%s%s%s)(%03d,%03d,%03d)\r\n\r%c", year, month, day, hour, minute, min, max, mean, 0x03);
                    xor_result = bccCreate(load_profile_line, 32, xor_result);
                    load_profile_line[29] = xor_result;
                }
            }
            else
            {
                printf("4\n");
                if (!first_flag)
                {
                    printf("5\n");
                    snprintf(load_profile_line, 29, "%c(%s%s%s%s%s)(%03d,%03d,%03d)\r\n", 0x02, year, month, day, hour, minute, min, max, mean);
                    xor_result = bccCreate(load_profile_line, 29, xor_result);
                    first_flag = 1;
                }
                else
                {
                    printf("6\n");
                    snprintf(load_profile_line, 28, "(%s%s%s%s%s)(%03d,%03d,%03d)\r\n", year, month, day, hour, minute, min, max, mean);
                    xor_result = bccCreate(load_profile_line, 28, xor_result);
                }
            }

            printf("data sent\n");
            uart_puts(UART0_ID, load_profile_line);

            sleep_ms(15);

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
        printf("data not found\n");
        uart_putc(UART0_ID, 0x15);
    }
    readENUART();
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

                char test[3];
                test[0] = rx_char;
                test[1] = '\n';
                test[2] = '\0';

                printf("%s", test);

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
                    writeENUART();

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
                    readENUART();
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

#if !DEBUG
        if (adc_remaining_time > 0)
        {
            vTaskDelay(pdMS_TO_TICKS(60000) - adc_remaining_time);
            adc_remaining_time = 0;
        }
#endif
        startTime = xTaskGetTickCount();

        rtc_get_datetime(&current_time);
        datetime_to_str(datetime_str, sizeof(datetime_buffer), &current_time);
        printf("Alarm Fired At %s\n", datetime_str);

        adcCapture(sample_buffer, VRMS_SAMPLE);

#if DEBUG
        char deneme[40] = {0};
        writeENUART();
        for (uint8_t i = 0; i < 150; i++)
        {
            snprintf(deneme, 20, "sample: %d\n", sample_buffer[i]);
            deneme[21] = '\0';

            uart_puts(UART0_ID, deneme);
        }
        readENUART();
        printf("\n");
#endif

        float mean = 2050 * conversion_factor / 1000;

#if DEBUG
        snprintf(deneme, 30, "mean: %f\n", mean);
        deneme[31] = '\0';
        writeENUART();
        uart_puts(UART0_ID, deneme);
        readENUART();
#endif

        for (uint16_t i = 0; i < VRMS_SAMPLE; i++)
        {
            double production = (double)(sample_buffer[i] * conversion_factor) / 1000;
            vrms_accumulator += pow((production - mean), 2);
        }

#if DEBUG
        snprintf(deneme, 34, "vrmsAc: %f\n", vrms_accumulator);
        deneme[35] = '\0';
        writeENUART();
        uart_puts(UART0_ID, deneme);
        readENUART();
#endif
        vrms = sqrt(vrms_accumulator / VRMS_SAMPLE);
        vrms = vrms * 75;

#if DEBUG
        char x_array[40] = {0};
        snprintf(x_array, 31, "VRMS: %d\n\n", (uint16_t)vrms);
        x_array[32] = '\0';
        writeENUART();
        uart_puts(UART0_ID, x_array);
        readENUART();
#endif

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
    adc_gpio_init(27);
    adc_select_input(1);
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