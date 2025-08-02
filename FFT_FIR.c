#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_adc/adc_continuous.h"
#include "esp_dsp.h"
#include "esp_heap_caps.h"

#define FS 1000.0
#define NUM_MUESTRAS 1024
#define FFT_SIZE 2048
#define GANANCIA_VISUAL 1000.0f

#define ADC_UNIT        ADC_UNIT_1
#define ADC_CHANNEL     ADC_CHANNEL_8  // GPIO9
#define ADC_ATTEN       ADC_ATTEN_DB_12
#define ADC_BIT_WIDTH   ADC_BITWIDTH_12
#define SAMPLE_FREQ_HZ  ((uint32_t)FS)
#define FRAME_SIZE      256
#define STORE_SIZE      1024

static float buffer[NUM_MUESTRAS];  // buffer circular
static int write_index = 0;

adc_continuous_handle_t adc_handle = NULL;
SemaphoreHandle_t semaforo_fft;

// === Inicialización de ADC DMA ===
void config_ADC_continuous() {
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = STORE_SIZE,
        .conv_frame_size = FRAME_SIZE,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_handle));

    adc_digi_pattern_config_t pattern = {
        .atten = ADC_ATTEN,
        .channel = ADC_CHANNEL,
        .unit = 0,
        .bit_width = ADC_BIT_WIDTH,
    };

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = SAMPLE_FREQ_HZ,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
        .pattern_num = 1,
        .adc_pattern = &pattern,
    };

    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &dig_cfg));
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
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
    if (!bp1) return;

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
    if (!bp2) return;

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

    for (int i = 0; i < FFT_SIZE; i++) {
        int idx = (write_index + i) % NUM_MUESTRAS;
        float original = buffer[idx];  // señal cruda original
        float filtrada = input[2 * i] / FFT_SIZE;

        // Salida para Serial Studio: Canal 1 (original), Canal 2 (filtrada)
        printf("%.4f,%.4f\r\n", original, filtrada * GANANCIA_VISUAL);

        fflush(stdout);
    }

}

void read_task_continuous(void *pvParameters) {
    uint8_t result[FRAME_SIZE];
    adc_digi_output_data_t *sample;

    while (1) {
        uint32_t bytes_read = 0;
        esp_err_t ret = adc_continuous_read(adc_handle, result, FRAME_SIZE, &bytes_read, 1000);
        if (ret == ESP_OK && bytes_read > 0) {
            for (int i = 0; i < bytes_read; i += sizeof(adc_digi_output_data_t)) {
                sample = (adc_digi_output_data_t*)&result[i];
                if (sample->type2.unit == 0 && sample->type2.channel == ADC_CHANNEL) {
                    uint16_t raw = sample->type2.data;
                    float voltage = (raw * 3.3f) / 4095.0f;
                    buffer[write_index] = voltage;
                    write_index = (write_index + 1) % NUM_MUESTRAS;

                    if (write_index % 128 == 0) {
                        xSemaphoreGive(semaforo_fft);
                    }
                }
            }
        }
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
    config_ADC_continuous();
    ESP_ERROR_CHECK(dsps_fft2r_init_fc32(NULL, FFT_SIZE));
    semaforo_fft = xSemaphoreCreateBinary();

    xTaskCreatePinnedToCore(read_task_continuous, "ReadECG_DMA", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(fft_task, "FFTProc", 8192, NULL, 1, NULL, 0);
}
