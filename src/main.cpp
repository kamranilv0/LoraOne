/* =============================================================================
   BosMag LoRa Transceiver Firmware
   ESP32-S3 + Ebyte E22-900M-30S (SX1262)
   
   Features:
   - Serial-to-LoRa transparent bridge
   - RadioLib RF switch control via setRfSwitchPins
   - LNA boost enabled for improved RX sensitivity
   - Interrupt-driven RX with DIO1
   - Safe RF state management to prevent hardware damage
   ============================================================================= */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "Config.h"
#include "GPIOManager.h"
#include "USBManager.h"
#include "RadioManager.h"
#include "Tasks.h"

static const char *TAG = "BOSMAG";

// ============================================================================
// MAIN APPLICATION
// ============================================================================

extern "C" void app_main() {
    // Print startup banner
    printf("\n");
    printf("╔════════════════════════════════════════════════════════════╗\n");
    printf("║                  BosMag LoRa Transceiver                   ║\n");
    printf("║            ESP32-S3 + Ebyte E22-900M-30S (SX1262)          ║\n");
    printf("║                    with RadioLib                           ║\n");
    printf("╚════════════════════════════════════════════════════════════╝\n");
    printf("\n");

    ESP_LOGI(TAG, "Firmware Version: 1.0.0");
    ESP_LOGI(TAG, "Board: %s", BOARD_NAME);
    ESP_LOGI(TAG, "MCU: %s", MCU_VARIANT);

    // Initialize peripherals
    if (!GPIOManager::init()) {
        ESP_LOGE(TAG, "GPIO initialization failed!");
        return;
    }

    if (!USBManager::init()) {
        ESP_LOGE(TAG, "USB initialization failed!");
        return;
    }

    if (!g_radioManager.init()) {
        ESP_LOGE(TAG, "Radio initialization failed!");
        return;
    }

    // Start RX mode
    g_radioManager.startReceive();

    // Create and start tasks
    tasks_init();

    ESP_LOGI(TAG, "=== BosMag Ready ===");
    ESP_LOGI(TAG, "Serial-to-LoRa bridge active");
    ESP_LOGI(TAG, "Waiting for data...");

    // Main task enters idle state
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}