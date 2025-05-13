#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"

// Configuración del ADC
#define ADC1_CHAN0 ADC_CHANNEL_8  // GPIO9 para la salida ECG
#define ADC_ATTEN ADC_ATTEN_DB_11 // Atenuación para 0-3.3V
#define ECG_THRESHOLD 1.5          // Umbral para detectar el pico R en volts
#define MIN_RR_INTERVAL_MS 300     // Ignorar si dos picos están muy juntos (<300ms)

adc_oneshot_unit_handle_t adc1_handle;
static const char *TAG = "ECG_Sensor";

int bpm = 0;
int64_t last_peak_time = 0;
bool above_threshold = false;

esp_err_t config_ADC();
void read_ECG_task(void *pvParameters);

void app_main() {
    config_ADC();
    xTaskCreate(read_ECG_task, "read_ECG_task", 4096, NULL, 5, NULL);
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

void read_ECG_task(void *pvParameters) {
    int adc_raw;
    float voltage;

    while (1) {
        adc_oneshot_read(adc1_handle, ADC1_CHAN0, &adc_raw);
        voltage = (adc_raw * 3.3 / 4095.0); // Conversión a volts

        int64_t now = esp_timer_get_time() / 1000; // Tiempo en ms

        // Detección de pico R (borde de subida)
        if (!above_threshold && voltage > ECG_THRESHOLD) {
            above_threshold = true;
            if (last_peak_time > 0 && (now - last_peak_time) > MIN_RR_INTERVAL_MS) {
                bpm = 60000 / (now - last_peak_time);
            }
            last_peak_time = now;
        }

        // Borde de bajada
        if (above_threshold && voltage < ECG_THRESHOLD - 0.1) {
            above_threshold = false;
        }

        // Enviar al USB CDC: formato compatible con Serial Studio
        printf("%.3f,%d\n", voltage, bpm); // ECG, BPM

        vTaskDelay(pdMS_TO_TICKS(2)); // 500 Hz
    }
}
