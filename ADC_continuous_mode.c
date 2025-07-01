#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_continuous.h"

#define TAG "ECG_ADC"
#define ADC_CHANNEL    ADC_CHANNEL_8        // GPIO9
#define ADC_UNIT       ADC_UNIT_1
#define ADC_ATTEN      ADC_ATTEN_DB_11
#define SAMPLE_FREQ_HZ 500
#define BUF_SIZE       512

adc_continuous_handle_t adc_handle = NULL;

void app_main() {
    // --- Configuración del ADC continuo ---
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = BUF_SIZE,
        .conv_frame_size = 64,
    };

    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = SAMPLE_FREQ_HZ,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };

    adc_digi_pattern_config_t pattern = {
        .atten = ADC_ATTEN,
        .channel = ADC_CHANNEL,
        .unit = 0,
        .bit_width = ADC_BITWIDTH_12
    };

    dig_cfg.pattern_num = 1;
    dig_cfg.adc_pattern = &pattern;

    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &dig_cfg));
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));

    uint8_t result[BUF_SIZE];
    adc_digi_output_data_t *p;
    int ret;

    while (1) {
        ret = adc_continuous_read(adc_handle, result, BUF_SIZE, NULL, 1000);
        if (ret == ESP_OK) {
            for (int i = 0; i < BUF_SIZE; i += sizeof(adc_digi_output_data_t)) {
                p = (adc_digi_output_data_t *)&result[i];
                if (p->type1.channel == ADC_CHANNEL) {
                    uint16_t raw = p->type1.data;
                    float voltage = (raw * 3.3f) / 4095.0f;
                    printf("%.3f\n", voltage); // Compatible con Serial Studio
                }
            }
        }
    }
}
