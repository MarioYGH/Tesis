#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_dsp.h"
#include "esp_heap_caps.h"

#define ADC1_CHAN0 ADC_CHANNEL_8  // GPIO9
#define ADC_ATTEN ADC_ATTEN_DB_12
#define FS 500.0
#define NUM_MUESTRAS 16
#define FFT_SIZE 32

static float beta_data[NUM_MUESTRAS];
static int beta_index = 0;
adc_oneshot_unit_handle_t adc1_handle;
SemaphoreHandle_t semaforo_fft;

esp_err_t config_ADC() {
    adc_oneshot_unit_init_cfg_t init_config1 = {.unit_id = ADC_UNIT_1};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC1_CHAN0, &config));
    return ESP_OK;
}

float* generar_fir(float fc, int N) {
    float *h = (float *)malloc(N * sizeof(float));
    if (!h) return NULL;
    for (int k = 0; k < N; k++) {
        float n = k - (N - 1) / 2.0;
        float sinc_val = (fabs(n) < 1e-6) ? 1.0 : sinf(2 * M_PI * fc * n) / (M_PI * n);
        float hamming = 0.54 - 0.46 * cosf(2 * M_PI * k / (N - 1));
        h[k] = 2 * fc * sinc_val * hamming;
    }
    return h;
}

void aplicar_filtro_en_frecuencia(float *fft_data, float *h_fir, int N) {
    for (int k = 0; k < N; k++) {
        float xr = fft_data[2 * k], xi = fft_data[2 * k + 1];
        float hr = h_fir[2 * k], hi = h_fir[2 * k + 1];
        float re = xr * hr - xi * hi;
        float im = xr * hi + xi * hr;
        fft_data[2 * k]     = re;
        fft_data[2 * k + 1] = im;
    }
}

void procesar_ECG() {
    static __attribute__((aligned(16))) float input[FFT_SIZE * 2] = {0};

    for (int i = 0; i < FFT_SIZE; i++) {
        if (i < NUM_MUESTRAS) {
            input[2 * i] = beta_data[i];
            input[2 * i + 1] = 0.0f;
        } else {
            input[2 * i] = 0.0f;
            input[2 * i + 1] = 0.0f;
        }
    }

    dsps_fft2r_fc32(input, FFT_SIZE);
    dsps_bit_rev2r_fc32(input, FFT_SIZE);

    float *h1 = generar_fir(20.0 / (FS / 2), NUM_MUESTRAS);
    float *h2 = generar_fir(150.0 / (FS / 2), NUM_MUESTRAS);
    float *bp1 = (float *)heap_caps_malloc(FFT_SIZE * 2 * sizeof(float), MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    if (!bp1) { printf("Error memoria bp1\n"); return; }

    for (int i = 0; i < NUM_MUESTRAS; i++) {
        bp1[2 * i] = h2[i] - h1[i];
        bp1[2 * i + 1] = 0.0f;
    }
    for (int i = NUM_MUESTRAS; i < FFT_SIZE; i++) {
        bp1[2 * i] = 0.0f;
        bp1[2 * i + 1] = 0.0f;
    }
    free(h1); free(h2);

    dsps_fft2r_fc32(bp1, FFT_SIZE);
    dsps_bit_rev2r_fc32(bp1, FFT_SIZE);
    aplicar_filtro_en_frecuencia(input, bp1, FFT_SIZE);
    free(bp1);

    float *h3 = generar_fir(0.5 / (FS / 2), NUM_MUESTRAS);
    float *h4 = generar_fir(3.0 / (FS / 2), NUM_MUESTRAS);
    float *bp2 = (float *)heap_caps_malloc(FFT_SIZE * 2 * sizeof(float), MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    if (!bp2) { printf("Error memoria bp2\n"); return; }

    for (int i = 0; i < NUM_MUESTRAS; i++) {
        bp2[2 * i] = h4[i] - h3[i];
        bp2[2 * i + 1] = 0.0f;
    }
    for (int i = NUM_MUESTRAS; i < FFT_SIZE; i++) {
        bp2[2 * i] = 0.0f;
        bp2[2 * i + 1] = 0.0f;
    }
    free(h3); free(h4);

    dsps_fft2r_fc32(bp2, FFT_SIZE);
    dsps_bit_rev2r_fc32(bp2, FFT_SIZE);
    aplicar_filtro_en_frecuencia(input, bp2, FFT_SIZE);
    free(bp2);

    float max_mag = 0.0f;
    float freq_max = 0.0f;
    for (int i = 0; i < FFT_SIZE / 2; i++) {
        float re = input[2 * i];
        float im = input[2 * i + 1];
        float mag = sqrtf(re * re + im * im);
        if (mag > max_mag) {
            max_mag = mag;
            freq_max = i * FS / FFT_SIZE;
        }
    }

    float bpm = freq_max * 60.0f;
    printf(", %.2f Hz, %.1f BPM\n", freq_max, bpm);
}

void read_task(void *pvParameters) {
    const TickType_t delay = pdMS_TO_TICKS(2);  // 500 Hz
    while (1) {
        int adc_raw;
        float voltage;

        adc_oneshot_read(adc1_handle, ADC1_CHAN0, &adc_raw);
        voltage = (adc_raw * 3.3 / 4095.0);
        printf("%.3f", voltage);

        beta_data[beta_index++] = voltage;
        if (beta_index >= NUM_MUESTRAS) {
            beta_index = 0;
            xSemaphoreGive(semaforo_fft);
        }
        vTaskDelay(delay);
    }
}

void fft_task(void *pvParameters) {
    while (1) {
        if (xSemaphoreTake(semaforo_fft, portMAX_DELAY)) {
            procesar_ECG();
        }
    }
}

void app_main() {
    config_ADC();
    ESP_ERROR_CHECK(dsps_fft2r_init_fc32(NULL, FFT_SIZE));
    semaforo_fft = xSemaphoreCreateBinary();

    xTaskCreatePinnedToCore(read_task, "ReadECG", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(fft_task, "FFTProc", 4096, NULL, 1, NULL, 0);
}
