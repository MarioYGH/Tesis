// Versión optimizada: imprime SOLO la señal filtrada.
// Binario opcional por stdout para reducir overhead de texto.
//
// Notas:
// - OUTPUT_BINARY = 0 -> imprime texto (una muestra por línea).
// - OUTPUT_BINARY = 1 -> emite float32 binario por stdout (fwrite).
//
// Si usas ESP32-S3 con consola USB Serial/JTAG por defecto,
// printf/fwrite salen por el mismo puerto virtual.

// ============================================================================
//  Adquisición continua de ECG en ESP32-S3 + filtrado en frecuencia (0.5–40 Hz)
//  y salida SOLO de la señal filtrada por stdout (texto o binario).
//
//  Flujo general (por qué así):
//  1) ADC continuo por DMA llena frames pequeños de muestras sin bloquear la CPU.
//  2) Guardamos las muestras en un buffer circular (ventana temporal) de tamaño N.
//  3) Cuando hay suficientes nuevas muestras (cada 128), disparamos el procesamiento.
//  4) Procesamiento por bloques:
//     - Copiamos la última ventana de N=1024 muestras, hacemos zero-padding a 2048.
//     - Quitamos DC (media) para centrar la señal en 0 (mejora la respuesta del filtro).
//     - FFT -> multiplicamos por la respuesta en frecuencia de un FIR pasa‑banda 0.5–40 Hz
//       precomputada (H_bp) -> IFFT (volvemos a tiempo).
//     - Emitimos SOLO la señal filtrada (texto o float binario), opcionalmente diezmada.
//  5) Precomputamos el filtro una vez: diseñamos su respuesta al impulso (h_bp) por
//     diferencia de low-pass (h_lp(fc_high) - h_lp(fc_low)), y la pasamos a frecuencia
//     con FFT. Así el filtrado en tiempo real es solo un producto punto a punto en freq.
//
//  Ventajas de este enfoque:
//  - La adquisición DMA evita pérdidas de muestras.
//  - El filtrado en frecuencia (FFT) es eficiente para filtros largos (513 taps).
//  - Al imprimir SOLO la señal filtrada reducimos carga de CPU y ancho de banda UART.
// ============================================================================

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

// ------------------------ Parámetros DSP/ventana -----------------------------
#define FS            1000.0f   // Frecuencia de muestreo en Hz (1 kHz)
#define NUM_MUESTRAS  1024      // Tamaño de ventana temporal (N) para procesar
#define FFT_SIZE      2048      // Tamaño de FFT (>= N + TAPAS - 1 para evitar alias en conv.)
#define TAPAS         513       // Taps del FIR (respuesta al impulso del filtro)
#define GANANCIA_VISUAL 3.0f    // Escalado para visualizar mejor la señal filtrada

// --- Salida: texto vs binario ---
// En ESP32-S3 stdout suele ir por USB Serial/JTAG. Tanto printf como fwrite salen por ahí.
#define OUTPUT_BINARY   0       // 0: texto (printf), 1: binario (float32 via fwrite)
#define PRINT_DECIMALS  4       // Decimales al imprimir en modo texto
#define PRINT_EVERY     4       // Emitir 1 de cada N muestras (reduce ancho de banda)

// ------------------------ Config ADC continuo (DMA) --------------------------
#define ADC_UNIT        ADC_UNIT_1
#define ADC_CHANNEL     ADC_CHANNEL_8    // Ajusta al canal físico que estés usando
#define ADC_ATTEN       ADC_ATTEN_DB_12  // Atenuación (rango hasta ~3.3 V aprox.)
#define ADC_BIT_WIDTH   ADC_BITWIDTH_12  // 12 bits (0..4095)
#define SAMPLE_FREQ_HZ  ((uint32_t)FS)
#define FRAME_SIZE      256              // Tamaño de “frame” DMA; pequeño => latencia baja
#define STORE_SIZE      1024             // Profundidad del buffer interno del driver

// ------------------------ Buffers y sincronización ---------------------------
static float buffer[NUM_MUESTRAS];  // Buffer circular para la ventana temporal
static int write_index = 0;         // Índice circular de escritura en 'buffer'

adc_continuous_handle_t adc_handle = NULL;
SemaphoreHandle_t semaforo_fft;     // Semáforo para disparar el bloque de procesamiento
static float *H_bp = NULL;          // Respuesta en frecuencia del filtro (compleja)

// ------------------------ Salida unificada (texto o binaria) -----------------
// Emitimos en un único punto para evitar código duplicado.
// Por qué: printf línea a línea es costoso; el binario reduce overhead si lo necesitas.
static inline void salida_filtrada(float v) {
#if OUTPUT_BINARY
    // BINARIO: emite el float32 tal cual (little-endian) por stdout.
    // Ventaja: 4 bytes/muestra, sin parsing. Ideal para graficar rápido en PC.
    fwrite(&v, sizeof(float), 1, stdout);
#else
    // TEXTO: una línea por muestra; más legible, mayor overhead.
    printf("%.*f\r\n", PRINT_DECIMALS, v);
#endif
}

// ------------------------ Configuración de ADC continuo ----------------------
// Por qué así: new_handle -> config -> start. 'pattern' define canal/aten/
// y el driver nos entrega estructuras adc_digi_output_data_t a través de DMA.
void config_ADC_continuous() {
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = STORE_SIZE,
        .conv_frame_size    = FRAME_SIZE,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_handle));

    // Patrón de conversión: un solo canal (unit=0 = ADC1), con la atenuación y resolución dadas.
    adc_digi_pattern_config_t pattern = {
        .atten     = ADC_ATTEN,
        .channel   = ADC_CHANNEL,
        .unit      = 0,               // 0 => ADC1
        .bit_width = ADC_BIT_WIDTH,
    };

    // Config global del modo continuo (frecuencia de muestreo y formato).
    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = SAMPLE_FREQ_HZ,
        .conv_mode      = ADC_CONV_SINGLE_UNIT_1,
        .format         = ADC_DIGI_OUTPUT_FORMAT_TYPE2, // struct type2 facilita leer canal y dato
        .pattern_num    = 1,
        .adc_pattern    = &pattern,
    };

    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &dig_cfg));
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
}

// ------------------------ Diseño FIR (en tiempo) -----------------------------
// generar_fir(fc_norm, N): Devuelve h_lp (low-pass) con ventana Hamming.
// Por qué: Ideal sinc recortada + ventana Hamming reduce lóbulos (mejor atenuación lateral).
// Nota: fc se pasa NORMALIZADA a [0..1] respecto a (FS/2). Ej: 40 Hz -> 40/(FS/2).
float* generar_fir(float fc, int N) {
    float *h = (float *)malloc(N * sizeof(float));
    if (!h) return NULL;

    for (int k = 0; k < N; k++) {
        float n = k - (N - 1) / 2.0f; // centro del filtro para fase lineal
        // sinc(2*pi*fc*n)/(pi*n) con caso n=0
        float sinc_val = (fabsf(n) < 1e-6f) ? 1.0f : sinf(2 * M_PI * fc * n) / (M_PI * n);
        float hamming  = 0.54f - 0.46f * cosf(2 * M_PI * k / (N - 1));
        // Low-pass ideal recortada y enventanada
        h[k] = 2 * fc * sinc_val * hamming;
    }
    return h;
}

// ------------------------ Precomputo del filtro en frecuencia ----------------
// Por qué: multiplicar en frecuencia es O(N), más rápido en ejecución que
// convolucionar en tiempo con 513 taps para cada bloque.
//
// Band-pass por diferencia de low-pass: h_bp = h_lp(fc_high) - h_lp(fc_low)
// => Paso entre (fc_low, fc_high). Aquí: fc_low=0.5 Hz, fc_high=40 Hz.
void precomputar_filtro_bp() {
    // fc normalizadas respecto a (FS/2):
    float *lp_lo = generar_fir(0.5f  / (FS / 2), TAPAS);  // LP con corte 0.5 Hz
    float *lp_hi = generar_fir(40.0f / (FS / 2), TAPAS);  // LP con corte 40 Hz

    // Buffer complejo para H(ω) del filtro (intercalado Re,Im) tamaño FFT_SIZE
    H_bp = heap_caps_calloc(FFT_SIZE * 2, sizeof(float), MALLOC_CAP_8BIT);
    // Colocamos la respuesta al impulso h_bp (tiempo) en la parte real y zereo el resto.
    for (int i = 0; i < FFT_SIZE; i++) {
        H_bp[2*i]   = (i < TAPAS) ? (lp_hi[i] - lp_lo[i]) : 0.0f;  // h_bp[n] = h_lp(40) - h_lp(0.5)
        H_bp[2*i+1] = 0.0f;
    }
    free(lp_lo); free(lp_hi);

    // FFT para pasar h_bp[n] -> H_bp[k]
    dsps_fft2r_fc32(H_bp, FFT_SIZE);
    dsps_bit_rev2r_fc32(H_bp, FFT_SIZE);
}

// ------------------------ Multiplicación compleja punto a punto --------------
// Por qué: Y[k] = X[k] * H[k] => en tiempo es la convolución x[n] * h[n].
void aplicar_filtro_en_frecuencia(float *fft_data, float *h_fir, int N) {
    for (int k = 0; k < N; k++) {
        float xr = fft_data[2 * k],     xi = fft_data[2 * k + 1];
        float hr = h_fir[2 * k],        hi = h_fir[2 * k + 1];
        float re = xr * hr - xi * hi;   // (a+jb)(c+jd) = (ac - bd) + j(ad + bc)
        float im = xr * hi + xi * hr;
        fft_data[2 * k]     = re;
        fft_data[2 * k + 1] = im;
    }
}

// ------------------------ Bloque de procesamiento (cada disparo) -------------
// Toma la última ventana de N muestras, filtra y emite SOLO la señal filtrada.
// Detalles importantes:
// - Zero-padding hasta FFT_SIZE para que la convolución sea lineal (no circular) en
//   los primeros N puntos si FFT_SIZE >= N + (TAPAS-1). Aquí 2048 >= 1024+512.
// - Restamos la media para eliminar offset DC del ECG y mejorar SNR del filtro.
// - IFFT con conjugado (truco de esp-dsp) + dividir por FFT_SIZE para normalizar.
// - PRINT_EVERY reduce la tasa de salida si necesitas menos muestras hacia el host.
void procesar_ECG() {
    static __attribute__((aligned(16))) float input[FFT_SIZE * 2] = {0}; // complejo intercalado

    // 1) Copiar la última ventana de N muestras desde el buffer circular.
    //    (solo parte real; parte imaginaria = 0)
    for (int i = 0; i < NUM_MUESTRAS; i++) {
        int idx = (write_index - NUM_MUESTRAS + i + NUM_MUESTRAS) % NUM_MUESTRAS;
        input[2*i]   = buffer[idx]; // Re
        input[2*i+1] = 0.0f;        // Im
    }
    // Zero-padding hasta FFT_SIZE
    for (int i = NUM_MUESTRAS; i < FFT_SIZE; i++) {
        input[2*i]   = 0.0f;
        input[2*i+1] = 0.0f;
    }

    // 2) Sustracción de DC (centrar señal)
    float mean = 0;
    for (int i = 0; i < NUM_MUESTRAS; i++) mean += input[2*i];
    mean /= NUM_MUESTRAS;
    for (int i = 0; i < NUM_MUESTRAS; i++) input[2*i] -= mean;

    // 3) FFT de la ventana
    dsps_fft2r_fc32(input, FFT_SIZE);
    dsps_bit_rev2r_fc32(input, FFT_SIZE);

    // 4) Filtrado en frecuencia: Y[k] = X[k] * H[k]
    aplicar_filtro_en_frecuencia(input, H_bp, FFT_SIZE);

    // 5) IFFT (usando el truco: conjugar parte imaginaria antes y después)
    for (int i = 0; i < FFT_SIZE; i++) input[2*i+1] *= -1.0f;  // conj(X)
    dsps_fft2r_fc32(input, FFT_SIZE);                          // FFT(conj(X)) = conj(IFFT(X)) * N
    dsps_bit_rev2r_fc32(input, FFT_SIZE);
    // Nota: Para obtener IFFT real, tomamos parte real y dividimos por FFT_SIZE.

    // 6) Emitir SOLO la señal filtrada (opcionalmente diezmada)
    //    Guardamos 1 de cada PRINT_EVERY muestras para bajar el ancho de banda.
    for (int i = 0; i < NUM_MUESTRAS; i++) {
        if ((i % PRINT_EVERY) == 0) {
            float filtrada = (input[2*i] / FFT_SIZE) * GANANCIA_VISUAL; // normalizar y escalar
            salida_filtrada(filtrada);
        }
    }
    // En binario puedes decidir si haces fflush(stdout) aquí según necesidades de latencia.
}

// ------------------------ Tarea de lectura ADC (productora) ------------------
// Por qué: leer en bucle frames DMA y llenar el buffer circular a 1 kHz.
// Además, cuando write_index cae en múltiplos de 128, disparamos el procesamiento.
// Usamos 'pendiente_fft' para no dar el semáforo repetidamente mientras el índice
// se mantiene en el mismo múltiplo (antirebote lógico).
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

                // Validamos que venga del ADC1 canal esperado.
                if (sample->type2.unit == 0 && sample->type2.channel == ADC_CHANNEL) {
                    // Dato crudo 12-bit (0..4095). Convertimos a "voltaje" aprox.
                    // Nota: La linealidad/ganancia real del ADC depende de atenuación y cal.
                    uint16_t raw = sample->type2.data;
                    float voltage = (raw * 3.3f) / 4095.0f;

                    // Escribimos en buffer circular
                    buffer[write_index] = voltage;
                    write_index = (write_index + 1) % NUM_MUESTRAS;

                    // Disparo de procesamiento cada 128 nuevas muestras
                    if (!pendiente_fft && (write_index % 128 == 0)) {
                        xSemaphoreGive(semaforo_fft);
                        pendiente_fft = true; // evita re-disparar mientras no cambiemos de múltiplo
                    }
                    if (write_index % 128 != 0) pendiente_fft = false;
                }
            }
        }
        // Si ret != ESP_OK o bytes_read==0, simplemente seguimos; el driver reintenta.
    }
}

// ------------------------ Tarea de FFT (consumidora) -------------------------
// Toma el semáforo y procesa el bloque completo (ver procesar_ECG).
void fft_task(void *pvParameters) {
    while (1) {
        if (xSemaphoreTake(semaforo_fft, portMAX_DELAY)) {
            procesar_ECG();
        }
    }
}

// ------------------------ app_main: orquestación -----------------------------
// Pasos clave:
// 1) Bajar el nivel de logs global para que no "ensucien" stdout (útil si graficas).
// 2) Configurar y arrancar ADC continuo.
// 3) Inicializar librería FFT de esp-dsp y precomputar H(ω) del filtro una vez.
// 4) Lanzar tareas: lectura (core 1, baja prioridad) y procesamiento (core 0, mayor).
void app_main() {
    esp_log_level_set("*", ESP_LOG_ERROR);  // Evita que logs interfieran con la salida

    ESP_LOGW("REBOOT", "Razon de reinicio: %d", esp_reset_reason());

    config_ADC_continuous();

    // Inicializar tablas internas de esp-dsp para FFT float32
    ESP_ERROR_CHECK(dsps_fft2r_init_fc32(NULL, FFT_SIZE));

    semaforo_fft = xSemaphoreCreateBinary();

    // Precomputar respuesta en frecuencia del filtro una sola vez
    precomputar_filtro_bp();

    // Tareas: productor (lectura DMA) y consumidor (FFT+filtro+salida)
    xTaskCreatePinnedToCore(read_task_continuous, "ReadECG_DMA", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(fft_task,           "FFTProc",     8192, NULL, 3, NULL, 0);
}

