#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

static const char *TAG = "LIGHT_CTRL";

#define PHOTO_SENSOR_ADC_CHANNEL ADC_CHANNEL_3 // GPIO 4 
#define LIGHT_THRESHOLD_HIGH     600           // mV BRIGHT state
#define LIGHT_THRESHOLD_LOW      400           // mV DARK state

typedef enum {
    STATE_DARK,
    STATE_BRIGHT
} LightState_t;

// Функція обчислення стану з урахуванням гістерезису
LightState_t get_light_state(int voltage_mv, LightState_t current_state)
{
    if (current_state == STATE_BRIGHT) {
        return (voltage_mv > LIGHT_THRESHOLD_LOW) ? STATE_BRIGHT : STATE_DARK;
    }
    return (voltage_mv >= LIGHT_THRESHOLD_HIGH) ? STATE_BRIGHT : STATE_DARK;
}

void app_main(void) {

    // Ініціалізація ADC1 для GPIO 4
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT, 
        .atten    = ADC_ATTEN_DB_12,      
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, PHOTO_SENSOR_ADC_CHANNEL, &config));

    // Налаштування калібрування
    adc_cali_handle_t adc1_cali_handle = NULL;
    bool do_calibration = false;
    
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = PHOTO_SENSOR_ADC_CHANNEL,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_handle) == ESP_OK) {
        do_calibration = true;
    }

    LightState_t current_state = STATE_DARK;
    int adc_raw;
    int voltage_mv;

    ESP_LOGI(TAG, "System running. Monitoring photo sensor...");

    while (1) {
        // Зчитування "сирого" значення АЦП
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, PHOTO_SENSOR_ADC_CHANNEL, &adc_raw));

        // Конвертація в мілівольти
        if (do_calibration) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw, &voltage_mv));
        } else {
            voltage_mv = (adc_raw * 3300) / 4095;
        }

        // Визначення нового стану за допомогою логіки гістерезису
        LightState_t detected_state = get_light_state(voltage_mv, current_state);

        ESP_LOGI(TAG, "ADC raw=%d, voltage=%d mV, state=%s.",
                 adc_raw,
                 voltage_mv,
                 detected_state == STATE_BRIGHT ? "BRIGHT" : "DARK");

        // Обробка зміни стану
        if (detected_state != current_state) {
            ESP_LOGI(TAG, "State changed from %s to %s.",
                     current_state == STATE_BRIGHT ? "BRIGHT" : "DARK",
                     detected_state == STATE_BRIGHT ? "BRIGHT" : "DARK");
            
            current_state = detected_state;
        }

        vTaskDelay(pdMS_TO_TICKS(250));
    }
    if (do_calibration) {
        adc_cali_delete_scheme_curve_fitting(adc1_cali_handle);
    }
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
}