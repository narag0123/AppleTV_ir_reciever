// motor.c
#include "motor.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

#define MOTOR_GPIO 25
#define TAG "MOTOR"
#define ANGLE 45

uint32_t angle_to_duty(int angle_deg) {
    int us = 500 + (angle_deg * 2000 / 180);
    return (uint32_t)((us / 20000.0) * 65536);
}

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

static void motor_task(void *arg) {
    ESP_LOGI(TAG, "motor_start - rotate to %d°\n", ANGLE);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, angle_to_duty(ANGLE));  // 180°
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    vTaskDelay(pdMS_TO_TICKS(300));

    ESP_LOGI(TAG, "motor_return - rotate back to 0°");

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, angle_to_duty(0));  // 0°
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    vTaskDelay(pdMS_TO_TICKS(300));

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    vTaskDelete(NULL);  // 태스크 종료
}

void motor_start(void) {
    xTaskCreate(motor_task, "motor_task", 2048, NULL, 5, NULL);
}