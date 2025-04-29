#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/uart.h"
#include <string.h>

// Definir el canal ADC
#define ADC1_CHAN0 ADC_CHANNEL_4  // GPIO32
#define ADC_ATTEN ADC_ATTEN_DB_12 // Atenuación para 0-3.3V

// UART Configuración
#define UART_PORT_NUM UART_NUM_1
#define TXD_PIN GPIO_NUM_1
#define RXD_PIN GPIO_NUM_3
#define TX_BUF_SIZE 1024

adc_oneshot_unit_handle_t adc1_handle;
static const char *TAG = "Muscle_Sensor";

esp_err_t config_ADC();
esp_err_t config_UART();
esp_err_t read_EMG();

void app_main() {
    config_ADC();
    config_UART();

    while (true) {
        read_EMG();
        vTaskDelay(pdMS_TO_TICKS(2)); // 500 Hz
    }
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

esp_err_t config_UART() {
    // Cierra el UART si ya estaba instalado
    uart_driver_delete(UART_PORT_NUM);  // No retorna error si no estaba instalado

    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, TX_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    return ESP_OK;
}

esp_err_t read_EMG() {
    int adc_raw;
    float voltage;
    char data_str[50];

    adc_oneshot_read(adc1_handle, ADC1_CHAN0, &adc_raw);
    voltage = (adc_raw * 3.3 / 4095.0); // Conversión ADC a Voltaje

    // Enviar por UART en formato /* V */
    sprintf(data_str, "/* %2.3f V */\n", voltage);
    uart_write_bytes(UART_PORT_NUM, data_str, strlen(data_str));

    ESP_LOGI(TAG, "Raw: %d, Voltage: %2.3f V", adc_raw, voltage);

    return ESP_OK;
}
