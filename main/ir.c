// ir.c
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_types.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "ir.h"
#include <motor.h>
#include <led.h>

// ───────────────────────────────────────────────
// 설정 상수 및 상태
// ───────────────────────────────────────────────
#define TAG "IR_RECEIVER"
#define IR_RX_GPIO 32
#define RX_BUF_SIZE 128
#define LED_GPIO 33

#define V_UP_IR    0xFD02FB04
#define V_DOWN_IR  0xFC03FB04
#define MUTE_IR    0xF609FB04

static rmt_channel_handle_t rx_channel = NULL;
static rmt_symbol_word_t rx_buffer[RX_BUF_SIZE];
static QueueHandle_t ir_event_queue = NULL;

// 상태 추적
static volatile bool mute_active = false;
static int mute_press_counter = 0;
static int64_t mute_last_received_time = 0;

// ───────────────────────────────────────────────
// IR 신호 디코딩 함수
// ───────────────────────────────────────────────
static void decode_ir_data(const rmt_symbol_word_t *symbols, size_t count) {
    ESP_LOGI(TAG, "Raw IR signal received. Length = %d symbols", count);

    if (count < 34) {
        // 짧은 신호지만 mute 상태라면 카운트 증가
        if (mute_active) {
            mute_press_counter++;
            mute_last_received_time = esp_timer_get_time();
        }
        return;
    }

    uint32_t data = 0;
    for (int i = 1; i <= 32; i++) {
        if (symbols[i].duration0 > 1500) {
            data |= (1 << (i - 1));
        }
    }

    ESP_LOGI(TAG, "Decoded data: 0x%08X", data);

    if (data == V_UP_IR) {
        ESP_LOGI(TAG, "Volume Up");
    } else if (data == V_DOWN_IR) {
        ESP_LOGI(TAG, "Volume Down");
    } else if (data == MUTE_IR) {
        ESP_LOGI(TAG, "Mute button received");
        if (!mute_active) {
            mute_active = true;
            mute_press_counter = 1;
            mute_last_received_time = esp_timer_get_time();
            ESP_LOGI(TAG, "Mute button pressed");
        } else {
            mute_press_counter++;
            mute_last_received_time = esp_timer_get_time();
        }
    } else {
        ESP_LOGI(TAG, "Unknown Button");
    }
}

// ───────────────────────────────────────────────
// Mute 버튼 상태 감시 태스크
// ───────────────────────────────────────────────
static void mute_watchdog_task(void *arg) {
    while (1) {
        if (mute_active) {
            int64_t now = esp_timer_get_time();
            if ((now - mute_last_received_time) > 1000000) { // 1초 동안 신호 없음
                ESP_LOGI(TAG, "Mute button released");
                ESP_LOGI(TAG, "Held count: %d", mute_press_counter);
                float estimated_time = mute_press_counter / 10.0f; // 1초에 10회 기준
                ESP_LOGI(TAG, "Estimated held duration: %.2f seconds", estimated_time);

                if (estimated_time >= 3.0f) {

                    led_blink(2000); // led 동작
                    motor_start(); // 모터동작
                }

                mute_active = false;
                mute_press_counter = 0;
                mute_last_received_time = 0;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ───────────────────────────────────────────────
// 수신 콜백 함수
// ───────────────────────────────────────────────
static bool ir_rx_callback(rmt_channel_handle_t rx_chan, const rmt_rx_done_event_data_t *edata, void *user_ctx) {
    size_t len = edata->num_symbols;
    if (len > RX_BUF_SIZE) len = RX_BUF_SIZE;

    memcpy(rx_buffer, edata->received_symbols, len * sizeof(rmt_symbol_word_t));
    xQueueSendFromISR(ir_event_queue, &len, NULL);

    rmt_receive_config_t receive_config = {
        .signal_range_min_ns = 1000,
        .signal_range_max_ns = 5000000,
    };
    rmt_receive(rx_channel, rx_buffer, sizeof(rx_buffer), &receive_config);

    return false;
}

// ───────────────────────────────────────────────
// 디코더 태스크
// ───────────────────────────────────────────────
static void ir_decode_task(void *arg) {
    size_t len;
    while (xQueueReceive(ir_event_queue, &len, portMAX_DELAY)) {
        decode_ir_data(rx_buffer, len);
    }
}

// ───────────────────────────────────────────────
// GPIO 초기화
// ───────────────────────────────────────────────
static void configure_gpio() {
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
}

// ───────────────────────────────────────────────
// IR 모듈 초기화 함수 (외부 노출)
// ───────────────────────────────────────────────
void ir_init(void) {
    configure_gpio();
    xTaskCreate(mute_watchdog_task, "mute_watchdog_task", 2048, NULL, 5, NULL);

    rmt_rx_channel_config_t rx_config = {
        .gpio_num = IR_RX_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .mem_block_symbols = RX_BUF_SIZE,
        .resolution_hz = 1000000,
        .flags.with_dma = false
    };
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_config, &rx_channel));

    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = ir_rx_callback,
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel, &cbs, NULL));
    ESP_ERROR_CHECK(rmt_enable(rx_channel));

    rmt_receive_config_t receive_config = {
        .signal_range_min_ns = 1000,
        .signal_range_max_ns = 5000000,
    };
    ESP_ERROR_CHECK(rmt_receive(rx_channel, rx_buffer, sizeof(rx_buffer), &receive_config));

    ir_event_queue = xQueueCreate(4, sizeof(size_t));
    xTaskCreate(ir_decode_task, "ir_decode_task", 4096, NULL, 10, NULL);

    ESP_LOGI(TAG, "IR Receiver started on GPIO %d", IR_RX_GPIO);
}