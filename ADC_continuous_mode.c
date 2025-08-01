#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_continuous.h"
#include "esp_system.h"

#define TAG "ECG_ADC"

// ADC configuration
#define ADC_UNIT            ADC_UNIT_1
#define ADC_CHANNEL         ADC_CHANNEL_8   // GPIO9
#define ADC_ATTEN           ADC_ATTEN_DB_12
#define ADC_BIT_WIDTH       ADC_BITWIDTH_12
#define SAMPLE_FREQ_HZ      1000

// Buffer sizes
#define ADC_FRAME_SIZE      128   // Must be multiple of sizeof(adc_digi_output_data_t)
#define ADC_STORE_SIZE      1024  // Must be >= FRAME_SIZE

adc_continuous_handle_t adc_handle = NULL;

void app_main() {
    // Mostrar causa de reinicio previa
    esp_reset_reason_t reason = esp_reset_reason();
    ESP_LOGI(TAG, "Reset reason: %d", reason);

    // Mostrar configuración usada
    ESP_LOGI(TAG, "Init ADC DMA: store_buf_size=%d, frame_size=%d",
             ADC_STORE_SIZE, ADC_FRAME_SIZE);

    // === Configurar ADC ===
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = ADC_STORE_SIZE,
        .conv_frame_size = ADC_FRAME_SIZE,
    };

    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_handle));

    adc_digi_pattern_config_t pattern = {
        .atten = ADC_ATTEN,
        .channel = ADC_CHANNEL,
        .unit = 0,  // ADC_UNIT_1 → unit 0
        .bit_width = ADC_BIT_WIDTH,
    };

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 1000,  // mínimo seguro para DMA
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
        .pattern_num = 1,
        .adc_pattern = &pattern,
    };

    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &dig_cfg));
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));

    // === Búfer de lectura ===
    uint8_t result[ADC_STORE_SIZE];
    adc_digi_output_data_t *sample;

    while (true) {
        uint32_t bytes_read = 0;
        esp_err_t ret = adc_continuous_read(adc_handle, result, ADC_STORE_SIZE, &bytes_read, 0); // no bloqueante

        if (ret == ESP_OK && bytes_read > 0) {
            for (int i = 0; i < bytes_read; i += sizeof(adc_digi_output_data_t)) {
                sample = (adc_digi_output_data_t*)&result[i];

                // Validación de canal y unidad
                if (sample->type2.unit == 0 && sample->type2.channel == ADC_CHANNEL) {
                    uint16_t raw = sample->type2.data;
                    float voltage = (raw * 3.3f) / 4095.0f;
                    printf("%.3f\n", voltage); // Compatible con Serial Studio
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1)); // ceder CPU
    }
}
