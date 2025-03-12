#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"

// Configuración del ADC
#define ADC1_CHAN0 ADC_CHANNEL_8  // GPIO9 para la salida ECG
#define ADC_ATTEN ADC_ATTEN_DB_11 // Atenuación para 0-3.3V

adc_oneshot_unit_handle_t adc1_handle;
static const char *TAG = "ECG_Sensor";

esp_err_t config_ADC();
esp_err_t read_ECG();

void app_main() {
    config_ADC();

    while (true) {
        read_ECG();
        vTaskDelay(pdMS_TO_TICKS(2)); // ~500 Hz
    }
}

esp_err_t config_ADC() {
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&init_config1, &adc1_handle);

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN,
    };
    adc_oneshot_config_channel(adc1_handle, ADC1_CHAN0, &config);

    return ESP_OK;
}

esp_err_t read_ECG() {
    int adc_raw;
    float voltage;

    adc_oneshot_read(adc1_handle, ADC1_CHAN0, &adc_raw);
    voltage = (adc_raw * 3.3 / 4095.0); // Conversión ADC a Voltios

    // Enviar datos por el puerto USB CDC
    printf("/*%2.3f*/", voltage);

    ESP_LOGI(TAG, "Raw: %d, Voltage: %2.3f V", adc_raw, voltage);

    return ESP_OK;
}
