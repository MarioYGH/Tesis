

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"

// CONFIGURACIÓN DE PINES
#define DOUT_ADC_CHANNEL   ADC_CHANNEL_8     // GPIO9
#define PULSE_GPIO         GPIO_NUM_4        // Señal digital de latido
#define ADC_ATTEN          ADC_ATTEN_DB_12   // Para rango de hasta 3.3V
#define ADC_WIDTH          ADC_BITWIDTH_12

// Variables globales
adc_oneshot_unit_handle_t adc1_handle;
volatile int beat_count = 0;

// ISR: Se ejecuta cada vez que llega un latido
static void IRAM_ATTR pulse_isr_handler(void *arg) {
    beat_count++;
}

// CONFIGURACIÓN ADC PARA DOUT (ECG)
void config_adc() {
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1
    };
    adc_oneshot_new_unit(&init_cfg, &adc1_handle);

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_WIDTH,
        .atten = ADC_ATTEN
    };
    adc_oneshot_config_channel(adc1_handle, DOUT_ADC_CHANNEL, &chan_cfg);
}

// CONFIGURACIÓN GPIO PARA PULSE
void config_pulse_gpio() {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PULSE_GPIO),
        .pull_up_en = 1,
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(PULSE_GPIO, pulse_isr_handler, NULL);
}

void app_main(void) {
    config_adc();
    config_pulse_gpio();

    int adc_raw;
    float voltage;
    int bpm = 0;
    int last_beat_count = 0;

    // Temporizador para calcular BPM cada 1 segundo
    int64_t last_time = esp_timer_get_time(); // En microsegundos

    while (1) {
        // Leer señal ECG analógica (DOUT)
        adc_oneshot_read(adc1_handle, DOUT_ADC_CHANNEL, &adc_raw);
        voltage = adc_raw * 3.3f / 4095.0f;

        // Calcular BPM cada 1 segundo
        int64_t now = esp_timer_get_time();
        if ((now - last_time) >= 1000000) {
            bpm = (beat_count - last_beat_count) * 60;
            last_beat_count = beat_count;
            last_time = now;
        }

        // Imprimir datos por puerto USB CDC
        printf("%.3f, %d\n", voltage, bpm);

        vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz (100 ms)
    }
}
