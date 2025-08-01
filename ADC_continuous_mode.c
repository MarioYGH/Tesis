#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_continuous.h"
#include "esp_system.h"

#define TAG "ECG_ADC"

// === Parámetros de configuración del ADC ===

// Unidad de ADC utilizada: el ESP32-S3 cuenta con dos unidades ADC (0 y 1). Aquí se emplea la unidad 1.
#define ADC_UNIT            ADC_UNIT_1

// Canal del ADC: se utiliza el canal 8 de la unidad 1, correspondiente al pin GPIO9,
// que es la salida recomendada para el sensor AD8232 en la tarjeta seleccionada.
#define ADC_CHANNEL         ADC_CHANNEL_8

// Atenuación seleccionada: ADC_ATTEN_DB_12 corresponde a una atenuación de 11 dB reales,
// permitiendo medir señales analógicas desde 0 V hasta aproximadamente 3.3 V sin saturación.
// Esta atenuación reemplaza al valor obsoleto `ADC_ATTEN_DB_11`.
#define ADC_ATTEN           ADC_ATTEN_DB_12

// Resolución del ADC: se configura con 12 bits (4096 niveles), suficiente para capturar la señal ECG
// con detalle, sin generar un volumen excesivo de datos.
#define ADC_BIT_WIDTH       ADC_BITWIDTH_12

// Frecuencia de muestreo: se fija en 1000 Hz, suficiente para señales ECG (que típicamente requieren ≥ 250 Hz).
// Esta frecuencia garantiza que el sistema no pierda información clínica relevante, y es compatible con DMA.
#define SAMPLE_FREQ_HZ      1000

// === Configuración de buffers ===

// Tamaño del frame de conversión: cantidad de datos leídos por llamada al driver.
// Debe ser múltiplo del tamaño de una muestra (adc_digi_output_data_t, usualmente 4 bytes).
#define ADC_FRAME_SIZE      128

// Tamaño del buffer DMA de almacenamiento interno.
// Mientras más grande sea, más tolerante será el sistema a retrasos en el procesamiento.
#define ADC_STORE_SIZE      1024

adc_continuous_handle_t adc_handle = NULL;

void app_main() {
    // === Diagnóstico de reinicio previo (útil para desarrollo y pruebas) ===
    esp_reset_reason_t reason = esp_reset_reason();
    ESP_LOGI(TAG, "Reset reason: %d", reason);

    // Mostrar la configuración actual del ADC para propósitos de depuración o registro
    ESP_LOGI(TAG, "Init ADC DMA: store_buf_size=%d, frame_size=%d",
             ADC_STORE_SIZE, ADC_FRAME_SIZE);

    // === Paso 1: Crear el handle del ADC en modo continuo (DMA) ===
    // Se usa adc_continuous para permitir muestreo autónomo y sin bloqueo.
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = ADC_STORE_SIZE,   // Tamaño máximo del buffer interno del ADC
        .conv_frame_size = ADC_FRAME_SIZE,      // Tamaño de los bloques de conversión
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_handle));

    // === Paso 2: Configurar el patrón de muestreo ===
    // Define el canal, atenuación y resolución a usar.
    adc_digi_pattern_config_t pattern = {
        .atten = ADC_ATTEN,
        .channel = ADC_CHANNEL,
        .unit = 0,  // Unidad 0 corresponde a ADC_UNIT_1 en este contexto
        .bit_width = ADC_BIT_WIDTH,
    };

    // === Paso 3: Configurar el modo digital del ADC continuo ===
    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = SAMPLE_FREQ_HZ,                 // Frecuencia real de muestreo
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,              // Solo se usa la unidad ADC1
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,           // Formato de salida para ESP-IDF v5.4+
        .pattern_num = 1,                                 // Solo se configura un patrón de canal
        .adc_pattern = &pattern,
    };

    // Aplicar la configuración al ADC
    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &dig_cfg));
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));

    // === Paso 4: Inicializar buffer de lectura externo ===
    // Este arreglo recibe los datos directamente desde el DMA del ADC.
    uint8_t result[ADC_STORE_SIZE];
    adc_digi_output_data_t *sample;

    while (true) {
        uint32_t bytes_read = 0;

        // === Paso 5: Leer muestras del ADC (modo no bloqueante) ===
        // Se intentan leer hasta ADC_STORE_SIZE bytes de datos muestreados.
        esp_err_t ret = adc_continuous_read(adc_handle, result, ADC_STORE_SIZE, &bytes_read, 0);

        // Verificar si hay datos válidos
        if (ret == ESP_OK && bytes_read > 0) {
            // Procesar cada muestra contenida en el buffer
            for (int i = 0; i < bytes_read; i += sizeof(adc_digi_output_data_t)) {
                sample = (adc_digi_output_data_t*)&result[i];

                // Validar que la muestra proviene del canal esperado y de la unidad correcta
                if (sample->type2.unit == 0 && sample->type2.channel == ADC_CHANNEL) {
                    uint16_t raw = sample->type2.data;  // Valor crudo de 12 bits

                    // Conversión a voltaje en escala de 0 a 3.3V
                    float voltage = (raw * 3.3f) / 4095.0f;

                    // Imprimir valor en formato compatible con Serial Studio
                    printf("%.3f\n", voltage);
                }
            }
        }

        // === Paso 6: Ceder el CPU brevemente ===
        // Esto previene bloqueos por el watchdog y permite ejecución de otras tareas del sistema
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
