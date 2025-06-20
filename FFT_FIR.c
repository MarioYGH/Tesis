#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_dsp.h"

#define ADC1_CHAN0 ADC_CHANNEL_8  // GPIO9
#define ADC_ATTEN ADC_ATTEN_DB_12
#define FS 500.0
#define NUM_MUESTRAS 16
#define FFT_SIZE 32  // Potencia de 2 >= NUM_MUESTRAS

static float beta_data[NUM_MUESTRAS];
static int beta_index = 0;
static bool ready_for_fft = false;
adc_oneshot_unit_handle_t adc1_handle;

esp_err_t config_ADC() {
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC1_CHAN0, &config));
    return ESP_OK;
}

void read_ECG(void *pvParameters) {
    int adc_raw;
    float voltage;

    adc_oneshot_read(adc1_handle, ADC1_CHAN0, &adc_raw);
    voltage = (adc_raw * 3.3 / 4095.0);
    printf("%.3f", voltage);

    if (!ready_for_fft) {
        beta_data[beta_index++] = voltage;
        if (beta_index >= NUM_MUESTRAS) {
            ready_for_fft = true;
            beta_index = 0;
        }
    }
    vTaskDelete(NULL);
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
    float input[FFT_SIZE * 2] = {0};  // Interleaved Re, Im
    memcpy(input, beta_data, sizeof(float) * NUM_MUESTRAS);

    for (int i = NUM_MUESTRAS; i < FFT_SIZE; i++) {
        input[2 * i] = 0.0f;
        input[2 * i + 1] = 0.0f;
    }

    dsps_fft2r_fc32(input, FFT_SIZE);
    dsps_bit_rev2r_fc32(input, FFT_SIZE);
    dsps_cplx2real_fc32(input, FFT_SIZE);

    float *h1 = generar_fir(20.0 / (FS / 2), NUM_MUESTRAS);
    float *h2 = generar_fir(150.0 / (FS / 2), NUM_MUESTRAS);
    float *bp1 = (float *)calloc(FFT_SIZE * 2, sizeof(float));
    for (int i = 0; i < NUM_MUESTRAS; i++) {
        bp1[2 * i] = h2[i] - h1[i];
        bp1[2 * i + 1] = 0.0f;
    }
    free(h1); free(h2);
    dsps_fft2r_fc32(bp1, FFT_SIZE);
    dsps_bit_rev2r_fc32(bp1, FFT_SIZE);
    dsps_cplx2real_fc32(bp1, FFT_SIZE);
    aplicar_filtro_en_frecuencia(input, bp1, FFT_SIZE);

    float *h3 = generar_fir(0.5 / (FS / 2), NUM_MUESTRAS);
    float *h4 = generar_fir(3.0 / (FS / 2), NUM_MUESTRAS);
    float *bp2 = (float *)calloc(FFT_SIZE * 2, sizeof(float));
    for (int i = 0; i < NUM_MUESTRAS; i++) {
        bp2[2 * i] = h4[i] - h3[i];
        bp2[2 * i + 1] = 0.0f;
    }
    free(h3); free(h4);
    dsps_fft2r_fc32(bp2, FFT_SIZE);
    dsps_bit_rev2r_fc32(bp2, FFT_SIZE);
    dsps_cplx2real_fc32(bp2, FFT_SIZE);
    aplicar_filtro_en_frecuencia(input, bp2, FFT_SIZE);

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

    free(bp1);
    free(bp2);
}

void FFT_task(void *pvParameters) {
    if (!ready_for_fft) {
        vTaskDelete(NULL);
        return;
    }
    ready_for_fft = false;
    procesar_ECG();
    vTaskDelete(NULL);
}

void app_main() {
    ESP_ERROR_CHECK(dsps_fft2r_init_fc32(NULL, FFT_SIZE));
    config_ADC();

    while (true) {
        xTaskCreatePinnedToCore(read_ECG, "ECG", 2048, NULL, 1, NULL, 1);
        xTaskCreatePinnedToCore(FFT_task, "FFT", 4096, NULL, 1, NULL, 0);
        vTaskDelay(pdMS_TO_TICKS(2));  // ~500 Hz
    }
}
