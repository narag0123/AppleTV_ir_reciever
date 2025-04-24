// motor.c
#include "motor.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

#define MOTOR_GPIO 25
#define TAG "MOTOR"

void motor_init(void) {
    ledc_timer_config_t timer_conf = {
        .duty_resolution = LEDC_TIMER_16_BIT,
        .freq_hz = 50, // 50Hz for servo
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t channel_conf = {
        .channel = LEDC_CHANNEL_0,
        .duty = 0,
        .gpio_num = MOTOR_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER_0
    };
    ledc_channel_config(&channel_conf);
}

void motor_start(void) {
    ESP_LOGI(TAG, "motor_start - rotate to 180°");

    // 180도 (약 2500us)
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 8191);  // (2500 / 20000) * 65536 ≈ 8191
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    vTaskDelay(pdMS_TO_TICKS(1000));  // 1초 대기

    ESP_LOGI(TAG, "motor_return - rotate back to 0°");

    // 0도 (약 500us)
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 1638);  // (500 / 20000) * 65536 ≈ 1638
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    vTaskDelay(pdMS_TO_TICKS(1000));  // 복귀 대기

    // 펄스를 중단해서 전류 차단 (옵션)
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}