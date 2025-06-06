#include "led.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define LED_GPIO 33
#define TAG "LED"

void led_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    ESP_LOGI(TAG, "LED GPIO %d initialized", LED_GPIO);
}

static void led_task(void *arg) {
    gpio_set_level(LED_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(2000));
    gpio_set_level(LED_GPIO, 0);
    vTaskDelete(NULL);
}

void led_blink() {
    xTaskCreate(led_task, "led_task", 1024, NULL, 5, NULL);
}