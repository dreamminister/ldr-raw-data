#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

static const char *TAG = "LIGHT_CTRL";

#define PHOTO_SENSOR_ADC_CHANNEL ADC_CHANNEL_3 
#define LED_GPIO                 GPIO_NUM_2    
#define LIGHT_THRESHOLD_HIGH     600           
#define LIGHT_THRESHOLD_LOW      400           
#define SMA_WINDOW_SIZE          10            

typedef enum {
    STATE_DARK,
    STATE_BRIGHT
} LightState_t;

typedef struct {
    int buffer[SMA_WINDOW_SIZE];
    int head;
    int count;
    int sum;
} SMA_Filter_t;

int sma_update(SMA_Filter_t *filter, int new_value) {
    if (filter->count == SMA_WINDOW_SIZE) {
        filter->sum -= filter->buffer[filter->head];
    } else {
        filter->count++;
    }
    filter->buffer[filter->head] = new_value;
    filter->sum += new_value;
    filter->head = (filter->head + 1) % SMA_WINDOW_SIZE;
    return filter->sum / filter->count;
}

LightState_t get_light_state(int voltage_mv, LightState_t current_state) {
    if (current_state == STATE_BRIGHT) {
        return (voltage_mv > LIGHT_THRESHOLD_LOW) ? STATE_BRIGHT : STATE_DARK;
    }
    return (voltage_mv >= LIGHT_THRESHOLD_HIGH) ? STATE_BRIGHT : STATE_DARK;
}

void app_main(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);
    gpio_set_level(LED_GPIO, 1); 

    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = { .unit_id = ADC_UNIT_1 };
    adc_oneshot_new_unit(&init_config1, &adc1_handle);

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT, 
        .atten    = ADC_ATTEN_DB_12,      
    };
    adc_oneshot_config_channel(adc1_handle, PHOTO_SENSOR_ADC_CHANNEL, &config);

    adc_cali_handle_t adc1_cali_handle = NULL;
    bool do_calibration = false;
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = PHOTO_SENSOR_ADC_CHANNEL,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_handle) == ESP_OK) {
        do_calibration = true;
    }

    SMA_Filter_t filter = {0}; 
    LightState_t current_state = STATE_DARK;
    int adc_raw, voltage_mv;

    while (1) {
        adc_oneshot_read(adc1_handle, PHOTO_SENSOR_ADC_CHANNEL, &adc_raw);

        if (do_calibration) {
            adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw, &voltage_mv);
        } else {
            voltage_mv = (adc_raw * 3300) / 4095;
        }

        int filtered_mv = sma_update(&filter, voltage_mv);
        LightState_t detected_state = get_light_state(filtered_mv, current_state);

        if (detected_state != current_state) {
            current_state = detected_state;
            gpio_set_level(LED_GPIO, current_state == STATE_DARK ? 1 : 0);
            ESP_LOGI(TAG, "State: %s (%d mV)", current_state == STATE_BRIGHT ? "BRIGHT" : "DARK", filtered_mv);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}