
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/flash.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <string.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>

#include "header/project_globals.h"
#include "header/print.h"
#include "header/spiflash.h"
#include "header/adc.h"
#include "header/bcc.h"
#include "header/threshold_event.h"

// Halka tampon aritmetigi kayit boyutunun 16 bayt kalmasina dayaniyor.
// Okuma tarafi (uart.c) alanlari SABIT OFFSET ile cozuyor; derleyici araya
// dolgu koyarsa flash yerlesimi sessizce bozulur. O yuzden burada zorluyoruz.
_Static_assert(sizeof(struct ThresholdData) == FLASH_RECORD_SIZE,
               "ThresholdData FLASH_RECORD_SIZE ile ayni boyutta olmali");
_Static_assert(offsetof(struct ThresholdData, vrms) == 12,
               "vrms alani 12. bayttan baslamali (uart.c sabit offset kullaniyor)");
_Static_assert(offsetof(struct ThresholdData, duration) == 14,
               "duration alani 14. bayttan baslamali (uart.c sabit offset kullaniyor)");

uint16_t calculateVariance(uint16_t *buffer, uint16_t size)
{
    uint64_t total = 0;
    uint64_t mean;
    uint64_t variance_total = 0;

    for (uint16_t i = 0; i < size; i++)
    {
        total += buffer[i];
    }

    mean = total / size;

    for (uint16_t i = 0; i < size; i++)
    {
        int32_t mult = buffer[i] - mean;
        variance_total += mult * mult;
    }

    return (uint16_t)(variance_total / (size - 1));
}

// Sifir cizgisi olarak BIAS kanali DEGIL, pencerenin kendi ortalamasi kullanilir.
// BIAS (ADC girisi 1) ile sinyal (ADC girisi 0) fiziksel olarak farkli iki kanal;
// aralarindaki her DC farki VRMS_MULTIPLICATION_VALUE ile carpilip sahte gerilim
// olarak okumaya giriyordu. Kendi ortalamasini cikarinca DC nereden gelirse gelsin
// denklemden dusuyor. bias_voltage parametresi sadece geriye donuk uyumluluk icin
// duruyor, hesaba karismiyor (teshis amacli loglanir).
float calculateVRMS(uint16_t *buffer, size_t size, float bias_voltage)
{
    (void)bias_voltage;

    float conversion_factor = (3.28f / (1 << 12));
    double sum = 0.0;
    double acc = 0.0;

    if (size == 0)
    {
        return 0.0f;
    }

    // 1. gecis: bu pencerenin ortalamasi (sifir cizgisi)
    for (size_t i = 0; i < size; i++)
    {
        sum += (double)buffer[i];
    }
    double mean = sum / (double)size;

    // 2. gecis: ayni orneklerin o ortalamadan sapmasi -> saf AC
    for (size_t i = 0; i < size; i++)
    {
        double diff = (double)buffer[i] - mean;
        acc += diff * diff;
    }

    float vrms = (float)sqrt(acc / (double)size);
    return vrms * conversion_factor * VRMS_MULTIPLICATION_VALUE;
}

float getMean(uint16_t *buffer, size_t size)
{
    float total = 0;

    for (size_t i = 0; i < size; i++)
        total += (float)buffer[i];

    if (size == 0)
    {
        return 0;
    }
    else
    {
        return (total / size);
    }
}

// Halkadaki bir sonraki yazma konumunu (mutlak slot indeksi) hesaplar.
uint16_t getThresholdWriteIndex(void)
{
    const uint8_t *sector = (const uint8_t *)(XIP_BASE + FLASH_THRESHOLD_RECORDS_ADDR + (th_sector_data * FLASH_SECTOR_SIZE));
    uint16_t offset = thFindFreeOffset(sector, FLASH_SECTOR_SIZE, FLASH_RECORD_SIZE);
    th_write_pos_t pos = thNextWritePos(th_sector_data, offset, FLASH_SECTOR_SIZE, FLASH_RECORD_SIZE, TH_RECORD_SECTOR_COUNT);

    return (uint16_t)(((uint32_t)pos.sector * TH_RECORDS_PER_SECTOR) + pos.slot_in_sector);
}

// Write threshold data to flash.
// Kayit alani bir HALKA tampondur: son sektor dolunca 0. sektore donulur, o
// sektor silinip uzerine yazilmaya devam edilir (en eski kayitlar dusr).
void __not_in_flash_func(writeThresholdRecord)(const struct ThresholdData *record)
{
    PRINTF("writing threshold record\r\n");

    if (record == NULL)
    {
        return;
    }

    // initialize the variables
    const uint8_t *flash_threshold_recs = (const uint8_t *)(XIP_BASE + FLASH_THRESHOLD_RECORDS_ADDR + (th_sector_data * FLASH_SECTOR_SIZE));

    if (xSemaphoreTake(xFlashMutex, pdMS_TO_TICKS(250)) == pdTRUE)
    {
        PRINTF("WRITETHRESHOLDRECORD: memcpy mutex received\r\n");
        memcpy(th_flash_buf, flash_threshold_recs, FLASH_SECTOR_SIZE);
        xSemaphoreGive(xFlashMutex);
    }
    else
    {
        PRINTF("WRITETHRESHOLDRECORD: memcpy mutex error\r\n");
        led_blink_pattern(LED_ERROR_CODE_FLASH_MUTEX_NOT_TAKEN, false);
        return;
    }

    // Kaydin alanlari cagiran tarafta (olay mantiginda) doldurulmustur; burada
    // sadece halkadaki yerine konur.
    uint16_t free_offset = thFindFreeOffset((const uint8_t *)th_flash_buf, FLASH_SECTOR_SIZE, FLASH_RECORD_SIZE);
    th_write_pos_t pos = thNextWritePos(th_sector_data, free_offset, FLASH_SECTOR_SIZE, FLASH_RECORD_SIZE, TH_RECORD_SECTOR_COUNT);

    if (pos.sector_changed)
    {
        PRINTF("WRITETHRESHOLDRECORD: sector %d dolu, %d. sektore geciliyor\r\n", th_sector_data, pos.sector);

        th_sector_data = pos.sector;

        // Yeni sektorun RAM kopyasini silinmis hale (0xFF) getir. Bu sektor
        // asagida silinip bastan yazildigi icin icindeki en eski kayitlar duser
        // -- halkanin basa donmesi tam olarak budur.
        memset(th_flash_buf, 0xFF, FLASH_SECTOR_SIZE);
        updateThresholdSector(th_sector_data);
    }

    th_flash_buf[pos.slot_in_sector] = *record;

    PRINTF("WRITETHRESHOLDRECORD: kayit sektor %d slot %d/%d konumuna yazildi\r\n",
           th_sector_data, pos.slot_in_sector, (int)TH_RECORDS_PER_SECTOR);

    // write buffer in flash
    if (xSemaphoreTake(xFlashMutex, pdMS_TO_TICKS(250)) == pdTRUE)
    {
        PRINTF("WRITETHRESHOLDRECORD: write flash mutex received\r\n");
        uint32_t ints = save_and_disable_interrupts();
        flash_range_erase(FLASH_THRESHOLD_RECORDS_ADDR + (th_sector_data * FLASH_SECTOR_SIZE), FLASH_SECTOR_SIZE);
        flash_range_program(FLASH_THRESHOLD_RECORDS_ADDR + (th_sector_data * FLASH_SECTOR_SIZE), (uint8_t *)th_flash_buf, FLASH_SECTOR_SIZE);
        restore_interrupts(ints);
        PRINTF("WRITETHRESHOLDDATA: threshold record written to flash.\r\n");
        xSemaphoreGive(xFlashMutex);
    }
    else
    {
        PRINTF("MUTEX CANNOT RECEIVED!\r\n");
        led_blink_pattern(LED_ERROR_CODE_FLASH_MUTEX_NOT_TAKEN, false);
        return;
    }
}

uint8_t detectSuddenAmplitudeChangeWithDerivative(float *sample_buf, size_t buffer_size)
{
    for (uint16_t i = 1; i < buffer_size; i++)
    {
        float derivative = sample_buf[i] - sample_buf[i - 1];

        if (fabs(derivative) > AMPLITUDE_THRESHOLD)
        {
            PRINTF("Sudden amplitude change detected at index %d: %f\r\n", i, fabs(derivative));
            return 1;
        }
    }

    return 0;
}

void calculateVRMSValuesPerSecond(float *vrms_buffer, uint16_t *sample_buf, size_t buffer_size, size_t sample_size_per_vrms_calc, float bias_voltage)
{
    for (uint16_t i = 0; i < buffer_size; i += sample_size_per_vrms_calc)
    {
        float vrms = calculateVRMS(sample_buf + i, sample_size_per_vrms_calc, bias_voltage);
        vrms_buffer[i / sample_size_per_vrms_calc] = vrms;
    }

    PRINTF("VRMS VALUES PER SECOND:");
    printBufferFloat(vrms_buffer, buffer_size / sample_size_per_vrms_calc);
}

void setAmplitudeChangeParameters(struct AmplitudeChangeTimerCallbackParameters *ac_data, float *vrms_values_buffer, uint16_t variance, size_t adc_fifo_size, size_t vrms_values_buffer_size_bytes)
{
    memcpy(ac_data->vrms_values_buffer, vrms_values_buffer, vrms_values_buffer_size_bytes);
    ac_data->vrms_values_buffer_size_bytes = vrms_values_buffer_size_bytes;
    ac_data->variance = variance;
    ac_data->adc_fifo_size = adc_fifo_size;
}
