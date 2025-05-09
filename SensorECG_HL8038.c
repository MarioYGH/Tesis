#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_timer.h"

// GPIOs usados
#define PULSE_GPIO GPIO_NUM_4       // Entrada digital de latido
#define ECG_ADC_CH ADC_CHANNEL_8    // GPIO9 

adc_oneshot_unit_handle_t adc_handle;
volatile uint32_t bpm = 0;
volatile int64_t last_time_us = 0;

static const char *TAG = "HL8038";

// ISR para contar pulsos
static void IRAM_ATTR pulse_isr_handler(void* arg) {
    int64_t now = esp_timer_get_time(); // tiempo en microsegundos
    if (last_time_us > 0) {
        int64_t diff = now - last_time_us;
        if (diff > 300000) { // al menos 200 BPM (300 ms)
            bpm = 60000000 / diff;
        }
    }
    last_time_us = now;
}

// Configurar ADC para ECG
void config_adc() {
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&init_cfg, &adc_handle);

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    adc_oneshot_config_channel(adc_handle, ECG_ADC_CH, &chan_cfg);
}

// Configurar GPIO de latido (pulso)
void config_pulse_gpio() {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pin_bit_mask = (1ULL << PULSE_GPIO),
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PULSE_GPIO, pulse_isr_handler, NULL);
}

void app_main(void) {
    config_adc();
    config_pulse_gpio();

    while (1) {
        int ecg_raw;
        adc_oneshot_read(adc_handle, ECG_ADC_CH, &ecg_raw);
        float voltage = (ecg_raw * 3.3 / 4095.0);

        // Enviar datos a Serial Studio (formato compatible)
        printf("%.3f,%ld\n", voltage, bpm);

        vTaskDelay(pdMS_TO_TICKS(2)); // 500 Hz
    }
}
