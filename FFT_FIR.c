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
#define NUM_MUESTRAS 1024
#define FFT_SIZE 2048
#define GANANCIA_VISUAL 1000.0f

static float buffer[NUM_MUESTRAS];  // Buffer circular
static int write_index = 0;
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
        int idx = (write_index + i) % NUM_MUESTRAS;
        input[2 * i] = buffer[idx];
        input[2 * i + 1] = 0.0f;
    }

    dsps_fft2r_fc32(input, FFT_SIZE);
    dsps_bit_rev2r_fc32(input, FFT_SIZE);

    float *h1 = generar_fir(20.0 / (FS / 2), NUM_MUESTRAS);
    float *h2 = generar_fir(150.0 / (FS / 2), NUM_MUESTRAS);
    float *bp1 = (float *)heap_caps_malloc(FFT_SIZE * 2 * sizeof(float), MALLOC_CAP_8BIT);
    if (!bp1) { printf("Error memoria bp1\n"); return; }

    for (int i = 0; i < FFT_SIZE; i++) {
        bp1[2 * i] = (i < NUM_MUESTRAS) ? (h2[i] - h1[i]) : 0.0f;
        bp1[2 * i + 1] = 0.0f;
    }
    free(h1); free(h2);
    dsps_fft2r_fc32(bp1, FFT_SIZE);
    dsps_bit_rev2r_fc32(bp1, FFT_SIZE);
    aplicar_filtro_en_frecuencia(input, bp1, FFT_SIZE);
    free(bp1);

    float *h3 = generar_fir(0.5 / (FS / 2), NUM_MUESTRAS);
    float *h4 = generar_fir(3.0 / (FS / 2), NUM_MUESTRAS);
    float *bp2 = (float *)heap_caps_malloc(FFT_SIZE * 2 * sizeof(float), MALLOC_CAP_8BIT);
    if (!bp2) { printf("Error memoria bp2\n"); return; }

    for (int i = 0; i < FFT_SIZE; i++) {
        bp2[2 * i] = (i < NUM_MUESTRAS) ? (h4[i] - h3[i]) : 0.0f;
        bp2[2 * i + 1] = 0.0f;
    }
    free(h3); free(h4);
    dsps_fft2r_fc32(bp2, FFT_SIZE);
    dsps_bit_rev2r_fc32(bp2, FFT_SIZE);
    aplicar_filtro_en_frecuencia(input, bp2, FFT_SIZE);
    free(bp2);

    for (int i = 0; i < FFT_SIZE; i++) {
        input[2 * i + 1] *= -1.0f;
    }
    dsps_fft2r_fc32(input, FFT_SIZE);
    dsps_bit_rev2r_fc32(input, FFT_SIZE);

    for (int i = 0; i < FFT_SIZE; i += 2) {
        float valor_filtrado = input[2 * i] / FFT_SIZE;
        printf("%.4f\r\n", valor_filtrado * GANANCIA_VISUAL);
        fflush(stdout);
    }
}

void read_task(void *pvParameters) {
    const TickType_t delay = pdMS_TO_TICKS(2);
    while (1) {
        int adc_raw;
        float voltage;

        adc_oneshot_read(adc1_handle, ADC1_CHAN0, &adc_raw);
        voltage = (adc_raw * 3.3 / 4095.0);
        buffer[write_index] = voltage;
        write_index = (write_index + 1) % NUM_MUESTRAS;

        if (write_index % 128 == 0) {
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

    xTaskCreatePinnedToCore(read_task, "ReadECG", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(fft_task, "FFTProc", 8192, NULL, 1, NULL, 0);
}
