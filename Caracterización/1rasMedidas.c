/* Aquí se tomaron las medidas para caracterizar el ADC del ESP32, ya se implementa la libreriá esp_cali que reduce el error pasando de tener un error de entre 60 - 70 mV a tener un error entre 10 - 20 mV
Se usara el código para obtener más muestras y comparar con un multimetro las lecturas del ADC del ESP para registrar ambas y posteriormente sacar la desviasión que existe y hacer una regresión polinomial
para reducir aún más el error de lectura*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

// ADC One-Shot + Calibración (ESP-IDF v5.x)
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// =================== CONFIGURACIÓN ===================
// Ajusta estos valores según tu conexión y rango deseado
#define ADC_UNIT_ID         ADC_UNIT_1
#define ADC_CHANNEL_ID      ADC_CHANNEL_8     // Usa el canal que estés empleando
#define ADC_BITWIDTH        ADC_BITWIDTH_12   // 0..4095
#define ADC_ATTEN_LEVEL     ADC_ATTEN_DB_12   // ~hasta ~3.3V aprox en S3
#define MULTISAMPLE_COUNT   64                // Número de lecturas para promediar
#define PRINT_PERIOD_MS     300               // Cada cuánto imprimir (ms)

// =====================================================

static const char *TAG = "ADC_CAL_CHECK";

static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_cali_handle_t cal_handle = NULL;
static bool cal_enabled = false;

// Intenta crear calibración con curve fitting (S3) y si no, line fitting (fallback)
static bool init_adc_calibration(adc_unit_t unit, adc_channel_t chan,
                                 adc_atten_t atten, adc_bitwidth_t bitwidth)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cfg_curve = {
        .unit_id  = unit,
        .chan     = chan,
        .atten    = atten,
        .bitwidth = bitwidth,
    };
    if (adc_cali_create_scheme_curve_fitting(&cfg_curve, &cal_handle) == ESP_OK) {
        ESP_LOGI(TAG, "Calibración: curve fitting habilitada");
        return true;
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cfg_line = {
        .unit_id  = unit,
        .atten    = atten,
        .bitwidth = bitwidth,
    };
    if (adc_cali_create_scheme_line_fitting(&cfg_line, &cal_handle) == ESP_OK) {
        ESP_LOGW(TAG, "Calibración: line fitting habilitada (fallback)");
        return true;
    }
#endif

    ESP_LOGW(TAG, "Calibración NO disponible (usaremos conversión aproximada)");
    return false;
}

static inline float raw_to_volts_fallback(int raw)
{
    // Fallback sencillo: regla de tres. Ajusta 3.3f si conoces mejor tu Vref real.
    return (raw * 3.3f) / 4095.0f;
}

void app_main(void)
{
    // Bajar verbosidad de logs para no ensuciar CSV
    esp_log_level_set("*", ESP_LOG_ERROR);
    ESP_LOGI(TAG, "Iniciando prueba de ADC + Calibración");

    // 1) Crear unidad ADC One-Shot
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_ID,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc_handle));

    // 2) Configurar canal (bitwidth + atenuación)
    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH,
        .atten    = ADC_ATTEN_LEVEL,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_ID, &chan_cfg));

    // 3) Inicializar calibración (curve fitting en S3; fallback si no)
    cal_enabled = init_adc_calibration(ADC_UNIT_ID, ADC_CHANNEL_ID, ADC_ATTEN_LEVEL, ADC_BITWIDTH);

    // 4) Encabezado CSV
    // campos: raw_avg, mv_cal, V_cal, V_fallback, diff_cal_minus_fallback_mV
    printf("raw_avg,mv_cal,V_cal,V_fallback,diff_cal_minus_fallback_mV\n");

    while (1) {
        // 5) Multimuestreo
        long raw_sum = 0;
        for (int i = 0; i < MULTISAMPLE_COUNT; i++) {
            int raw = 0;
            esp_err_t err = adc_oneshot_read(adc_handle, ADC_CHANNEL_ID, &raw);
            if (err == ESP_OK) {
                raw_sum += raw;
            }
        }
        int raw_avg = (int)(raw_sum / (MULTISAMPLE_COUNT > 0 ? MULTISAMPLE_COUNT : 1));

        // 6) Conversión calibrada (si disponible) a mV y V
        int mv_cal = -1;
        float V_cal = -1.0f;
        if (cal_enabled && cal_handle) {
            if (adc_cali_raw_to_voltage(cal_handle, raw_avg, &mv_cal) == ESP_OK) {
                V_cal = mv_cal / 1000.0f;
            } else {
                // Si hubo error puntual, usa fallback
                float Vfb = raw_to_volts_fallback(raw_avg);
                mv_cal = (int)(Vfb * 1000.0f);
                V_cal = Vfb;
            }
        } else {
            // Sin calibración: solo fallback
            float Vfb = raw_to_volts_fallback(raw_avg);
            mv_cal = (int)(Vfb * 1000.0f);
            V_cal = Vfb;
        }

        // 7) Fallback separado para comparar (ideal sin calibración)
        float V_fallback = raw_to_volts_fallback(raw_avg);
        float diff_mV = (V_cal - V_fallback) * 1000.0f;

        // 8) Imprimir CSV (fácil de pegar en Excel o plotear)
        printf("%d,%d,%.4f,%.4f,%.1f\n",
               raw_avg, mv_cal, V_cal, V_fallback, diff_mV);

        vTaskDelay(pdMS_TO_TICKS(PRINT_PERIOD_MS));
    }

    // (Opcional) Deinit si tu app fuera a salir:
    // if (cal_enabled && cal_handle) {
    // #if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    //     adc_cali_delete_scheme_curve_fitting(cal_handle);
    // #elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    //     adc_cali_delete_scheme_line_fitting(cal_handle);
    // #endif
    // }
    // adc_oneshot_del_unit(adc_handle);
}
