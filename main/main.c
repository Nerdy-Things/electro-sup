#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "driver/adc.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (18) // GPIO_18 where the ESC is connected
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_16_BIT // 16-bit resolution
#define LEDC_FREQUENCY          (50) // 50 Hz frequency (20 ms period)

#define HALL_SENSOR_ADC_CHANNEL ADC1_CHANNEL_5  // GPIO33

#define DUTY_CYCLE_MIN 3276.0   // Corresponds to 0% duty cycle
#define DUTY_CYCLE_MAX 6553.0   // Corresponds to 100% duty cycle
#define DUTY_THRESHOLD 200.0   // DUTY_CYCLE_MIN + DUTY_THRESHOLD - means that with that cycle the motor will have enough power under water to rotate

// Voltage Thresholds
#define VOLTAGE_MIN 1.64        // Minimum voltage corresponding to DUTY_CYCLE_MIN
#define VOLTAGE_MAX 2.1  

void init_pwm(void) 
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY, 
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, 
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
} 

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    init_pwm();

    // Configure ADC width and attenuation
    adc1_config_width(ADC_WIDTH_BIT_12);  // 12-bit width
    adc1_config_channel_atten(HALL_SENSOR_ADC_CHANNEL, ADC_ATTEN_DB_11);

    float maxRange = DUTY_CYCLE_MIN + (DUTY_CYCLE_MAX - DUTY_CYCLE_MIN) / 2;

    float duty_cycle_range = maxRange - DUTY_CYCLE_MIN;
    float voltage_range = VOLTAGE_MAX - VOLTAGE_MIN;

    while (1) {
        // Read the ADC value (range: 0 to 4095 for 12-bit resolution)
        int adc_reading = adc1_get_raw(HALL_SENSOR_ADC_CHANNEL);

        // Convert the ADC value to a voltage
        // Assuming a 3.3V reference and 12-bit resolution
        float voltage = adc_reading * (3.3 / 4095.0);
        
        float duty = DUTY_CYCLE_MIN;
        if (voltage < VOLTAGE_MIN) {
            duty = DUTY_CYCLE_MIN;
        } else if (voltage > VOLTAGE_MAX) {
            duty = maxRange;
        } else {
            float adjusted_voltage = voltage - VOLTAGE_MIN;
            float voltage_ratio = adjusted_voltage / voltage_range;
            duty = DUTY_CYCLE_MIN + DUTY_THRESHOLD + (duty_cycle_range * voltage_ratio);
        }

        // MAKE sure that the Duty cycle is in the range.
        if (duty < DUTY_CYCLE_MIN) {
            duty = DUTY_CYCLE_MIN;
        } else if (duty > maxRange) {
            duty = maxRange;
        }

        //----> ESC Range setup code block
        // if (voltage > 1.7) {
        //     ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, DUTY_CYCLE_MIN));
        //     ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));            
        // } else {
        //     ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, DUTY_CYCLE_MAX));
        //     ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));   
        // }
        //<---- END ESC Range setup code block

        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

        // printf("ADC Reading: %d, Voltage: %.2f V Duty: %.2f \n", adc_reading, voltage, duty);
        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}