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

#define TAG "IR_RECEIVER"
#define IR_RX_GPIO 32
#define RX_BUF_SIZE 128
#define LED_GPIO 33

#define V_UP_IR 0xFD02FB04
#define V_DOWN_IR 0xFC03FB04
#define MUTE_IR 0xF609FB04

static rmt_channel_handle_t rx_channel = NULL;
static rmt_symbol_word_t rx_buffer[RX_BUF_SIZE];
static QueueHandle_t ir_event_queue = NULL;

// 버튼 상태 관리
static bool mute_active = false;
static int64_t mute_press_time = 0;
static int64_t mute_last_signal_time = 0;

static void decode_ir_data(const rmt_symbol_word_t *symbols, size_t count) {
    ESP_LOGI(TAG, "Raw IR signal received. Length = %d symbols", count);
    if (count < 34) return;

    uint32_t data = 0;
    for (int i = 1; i <= 32; i++) {
        uint32_t high = symbols[i].duration0;
        if (high > 1500) {
            data |= (1 << (i - 1));
        }
    }

    ESP_LOGI(TAG, "Decoded data: 0x%08X", data);
    int64_t now = esp_timer_get_time() / 1000;  // ms 단위로 변환

    if (data == V_UP_IR) {
        ESP_LOGI(TAG, "Volume Up");
    } else if (data == V_DOWN_IR) {
        ESP_LOGI(TAG, "Volume Down");
    } else if (data == MUTE_IR) {
        ESP_LOGI(TAG, "Mute button received");

    } else {
        ESP_LOGI(TAG, "Unknown Button");
    }
}

static bool ir_rx_callback(rmt_channel_handle_t rx_chan, const rmt_rx_done_event_data_t *edata, void *user_ctx) {
    (void)rx_chan;
    (void)user_ctx;

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

static void ir_decode_task(void *arg) {
    size_t len;
    while (xQueueReceive(ir_event_queue, &len, portMAX_DELAY)) {
        decode_ir_data(rx_buffer, len);
    }
}

gpio_config_t io_conf = {
    .pin_bit_mask = 1ULL << LED_GPIO,
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = 0,
    .pull_down_en = 0,
    .intr_type = GPIO_INTR_DISABLE,
};

void app_main(void) {
    ESP_LOGI(TAG, "Initializing IR receiver...");

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
    gpio_config(&io_conf);
}