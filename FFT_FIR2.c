// Versión optimizada: imprime SOLO la señal filtrada.
// Binario opcional por stdout para reducir overhead de texto.
//
// Notas:
// - OUTPUT_BINARY = 0 -> imprime texto (una muestra por línea).
// - OUTPUT_BINARY = 1 -> emite float32 binario por stdout (fwrite).
//
// Si usas ESP32-S3 con consola USB Serial/JTAG por defecto,
// printf/fwrite salen por el mismo puerto virtual.

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
#include "esp_system.h"

#define FS 1000.0f
#define NUM_MUESTRAS 1024
#define FFT_SIZE 2048
#define TAPAS 513
#define GANANCIA_VISUAL 3.0f

// ---- Opciones de salida ----
#define OUTPUT_BINARY 0       // 0: texto (printf), 1: binario (fwrite float32)
#define PRINT_DECIMALS 4      // decimales en modo texto
#define PRINT_EVERY 4         // imprime 1 de cada N muestras (en texto o binario)

// ---- ADC config ----
#define ADC_UNIT        ADC_UNIT_1
#define ADC_CHANNEL     ADC_CHANNEL_8
#define ADC_ATTEN       ADC_ATTEN_DB_12
#define ADC_BIT_WIDTH   ADC_BITWIDTH_12
#define SAMPLE_FREQ_HZ  ((uint32_t)FS)
#define FRAME_SIZE      256
#define STORE_SIZE      1024

static float buffer[NUM_MUESTRAS];
static int write_index = 0;

adc_continuous_handle_t adc_handle = NULL;
SemaphoreHandle_t semaforo_fft;
static float *H_bp = NULL; // Filtro en frecuencia

static inline void salida_filtrada(float v) {
#if OUTPUT_BINARY
    // Emitir como float32 binario por stdout
    fwrite(&v, sizeof(float), 1, stdout);
#else
    // Emitir como texto
    // Nota: evitar printf dentro de ISR; aquí es contexto de tarea.
    printf("%.*f\r\n", PRINT_DECIMALS, v);
#endif
}

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
        float n = k - (N - 1) / 2.0f;
        float sinc_val = (fabsf(n) < 1e-6f) ? 1.0f : sinf(2 * M_PI * fc * n) / (M_PI * n);
        float hamming = 0.54f - 0.46f * cosf(2 * M_PI * k / (N - 1));
        h[k] = 2 * fc * sinc_val * hamming;
    }
    return h;
}

void precomputar_filtro_bp() {
    float *lp_lo = generar_fir(0.5f  / (FS / 2), TAPAS);
    float *lp_hi = generar_fir(40.0f / (FS / 2), TAPAS);
    H_bp = heap_caps_calloc(FFT_SIZE * 2, sizeof(float), MALLOC_CAP_8BIT);

    for (int i = 0; i < FFT_SIZE; i++) {
        H_bp[2*i]   = (i < TAPAS) ? (lp_hi[i] - lp_lo[i]) : 0.0f;
        H_bp[2*i+1] = 0.0f;
    }
    free(lp_lo); free(lp_hi);
    dsps_fft2r_fc32(H_bp, FFT_SIZE);
    dsps_bit_rev2r_fc32(H_bp, FFT_SIZE);
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

    // Copiar últimas muestras y zero-padding
    for (int i = 0; i < NUM_MUESTRAS; i++) {
        int idx = (write_index - NUM_MUESTRAS + i + NUM_MUESTRAS) % NUM_MUESTRAS;
        input[2*i]   = buffer[idx];
        input[2*i+1] = 0.0f;
    }
    for (int i = NUM_MUESTRAS; i < FFT_SIZE; i++) {
        input[2*i]   = 0.0f;
        input[2*i+1] = 0.0f;
    }

    // Sustracción de DC
    float mean = 0;
    for (int i = 0; i < NUM_MUESTRAS; i++) mean += input[2*i];
    mean /= NUM_MUESTRAS;
    for (int i = 0; i < NUM_MUESTRAS; i++) input[2*i] -= mean;

    // FFT
    dsps_fft2r_fc32(input, FFT_SIZE);
    dsps_bit_rev2r_fc32(input, FFT_SIZE);

    // Aplicar pasa-banda 0.5–40 Hz
    aplicar_filtro_en_frecuencia(input, H_bp, FFT_SIZE);

    // IFFT (conjugado)
    for (int i = 0; i < FFT_SIZE; i++) input[2*i+1] *= -1.0f;
    dsps_fft2r_fc32(input, FFT_SIZE);
    dsps_bit_rev2r_fc32(input, FFT_SIZE);

    // Salida: SOLO filtrada (opcionalmente diezmada para reducir ancho de banda)
    // Nota: input[2*i] / FFT_SIZE -> dominio temporal (real)
#if OUTPUT_BINARY
    // Binario: emite 1 de cada PRINT_EVERY muestras
    for (int i = 0; i < NUM_MUESTRAS; i++) {
        if ((i % PRINT_EVERY) == 0) {
            float filtrada = (input[2*i] / FFT_SIZE) * GANANCIA_VISUAL;
            salida_filtrada(filtrada);
        }
    }
    // En binario, evita flush frecuente; si lo necesitas para tiempo real:
    // fflush(stdout);
#else
    // Texto: emite 1 de cada PRINT_EVERY muestras como línea
    for (int i = 0; i < NUM_MUESTRAS; i++) {
        if ((i % PRINT_EVERY) == 0) {
            float filtrada = (input[2*i] / FFT_SIZE) * GANANCIA_VISUAL;
            salida_filtrada(filtrada);
        }
    }
#endif
}

void read_task_continuous(void *pvParameters) {
    uint8_t result[FRAME_SIZE];
    adc_digi_output_data_t *sample;
    static bool pendiente_fft = false;

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

                    if (!pendiente_fft && (write_index % 128 == 0)) {
                        xSemaphoreGive(semaforo_fft);
                        pendiente_fft = true;
                    }
                    if (write_index % 128 != 0) pendiente_fft = false;
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
    // Opcional: baja el nivel de logs para no interferir con la salida
    esp_log_level_set("*", ESP_LOG_ERROR);

    ESP_LOGW("REBOOT", "Razon de reinicio: %d", esp_reset_reason());

    config_ADC_continuous();
    ESP_ERROR_CHECK(dsps_fft2r_init_fc32(NULL, FFT_SIZE));
    semaforo_fft = xSemaphoreCreateBinary();
    precomputar_filtro_bp();

    xTaskCreatePinnedToCore(read_task_continuous, "ReadECG_DMA", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(fft_task, "FFTProc", 8192, NULL, 3, NULL, 0);
}
