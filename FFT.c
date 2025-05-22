#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"

// === Configuración del ADC ===
#define ADC1_CHAN0 ADC_CHANNEL_8  // GPIO9 (ECG)
#define ADC_ATTEN ADC_ATTEN_DB_11 // Atenuación 0-3.3V
#define ECG_THRESHOLD 1.5         // Umbral para pico R
#define MIN_RR_INTERVAL_MS 300    // Intervalo mínimo para BPM

// === FFT ===
#define N 128                     // Tamaño FFT (potencia de 2)
#define FS 500.0                  // Frecuencia de muestreo (Hz)
#define PI 3.14159265358979323846

typedef float real;
typedef struct { real Re; real Im; } complex;

char line_output[1024];  // Para la línea completa

adc_oneshot_unit_handle_t adc1_handle;
static const char *TAG = "ECG_Sensor";

// === Variables de ECG y FFT ===
int bpm = 0;
int64_t last_peak_time = 0;
bool above_threshold = false;

esp_err_t config_ADC();
void read_ECG_task(void *pvParameters);

// === FFT sin librerías ===
void fft(complex *v, int n, complex *tmp) {
    if (n > 1) {
        int k, m;
        complex z, w, *vo, *ve;
        ve = tmp;
        vo = tmp + n/2;
        for (k = 0; k < n/2; k++) {
            ve[k] = v[2*k];
            vo[k] = v[2*k+1];
        }
        fft(ve, n/2, v);
        fft(vo, n/2, v);
        for (m = 0; m < n/2; m++) {
            w.Re = cos(2*PI*m/(double)n);
            w.Im = -sin(2*PI*m/(double)n);
            z.Re = w.Re*vo[m].Re - w.Im*vo[m].Im;
            z.Im = w.Re*vo[m].Im + w.Im*vo[m].Re;
            v[m].Re       = ve[m].Re + z.Re;
            v[m].Im       = ve[m].Im + z.Im;
            v[m+n/2].Re   = ve[m].Re - z.Re;
            v[m+n/2].Im   = ve[m].Im - z.Im;
        }
    }
}

void app_main() {
    config_ADC();
    xTaskCreate(read_ECG_task, "read_ECG_task", 8192, NULL, 5, NULL);
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

    float buffer[N];
    int index = 0;
    complex fft_input[N];
    complex fft_scratch[N];

    while (1) {
        adc_oneshot_read(adc1_handle, ADC1_CHAN0, &adc_raw);
        voltage = (adc_raw * 3.3 / 4095.0);

        // Calcular BPM (detección de pico R)
        int64_t now = esp_timer_get_time() / 1000;
        if (!above_threshold && voltage > ECG_THRESHOLD) {
            above_threshold = true;
            if (last_peak_time > 0 && (now - last_peak_time) > MIN_RR_INTERVAL_MS) {
                bpm = 60000 / (now - last_peak_time);
            }
            last_peak_time = now;
        }
        if (above_threshold && voltage < ECG_THRESHOLD - 0.1) {
            above_threshold = false;
        }

        // Enviar señal cruda al USB (Serial Studio compatible)
        // Construir línea completa: ECG, BPM, mag[0]...mag[N/2 - 1]
        int len = snprintf(line_output, sizeof(line_output), "%.3f,%d", voltage, bpm);

        for (int k = 0; k < N/2; k++) {
            float mag = sqrt(fft_input[k].Re * fft_input[k].Re + fft_input[k].Im * fft_input[k].Im);
            len += snprintf(line_output + len, sizeof(line_output) - len, ",%.4f", mag);
        }

        printf("%s\n", line_output);


        // === Acumular en buffer ===
        buffer[index++] = voltage;

        if (index >= N) {
            // Rellenar vector complejo
            for (int i = 0; i < N; i++) {
                fft_input[i].Re = buffer[i];
                fft_input[i].Im = 0.0;
            }

            // Aplicar FFT
            fft(fft_input, N, fft_scratch);

            index = 0; // Reiniciar buffer
        }

        vTaskDelay(pdMS_TO_TICKS(2)); // 2 ms -> 500 Hz
    }
}
