// main.c
#include <motor.h>

#include "ir.h"
#include "led.h"
#include "esp_log.h"

static const char *TAG = "MAIN";

void app_main(void) {
    motor_init();
    ESP_LOGI(TAG, "Starting application...");
    ir_init();
}
