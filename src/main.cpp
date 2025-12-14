/* =============================================================================
   BosMag LoRa Transceiver Firmware
   ESP32-S3 + Ebyte E22-900M-30S (SX1262)
   
   Modular Architecture:
   - HardwareInit: GPIO, SPI, UART initialization
   - RadioManager: LoRa radio management (singleton)
   - SerialBridge: Bidirectional UART<->LoRa bridge
   - Utils: Common utilities
   ============================================================================= */

#include "Config.h"
#include "HardwareInit.h"
#include "RadioManager.h"
#include "SerialBridge.h"
#include "Utils.h"

#include "esp_log.h"
#include "esp_chip_info.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "BOSMAG";

extern "C" void app_main()
{
    // Print startup banner
    Utils::printBanner();

    ESP_LOGI(TAG, "Firmware Version: 1.0.0");
    ESP_LOGI(TAG, "Board: %s", BOARD_NAME);
    ESP_LOGI(TAG, "MCU: %s", MCU_VARIANT);

    // Initialize all hardware peripherals
    HardwareInit::initAll();

    // Initialize radio manager (singleton)
    RadioManager& radio = RadioManager::getInstance();
    radio.init();
    radio.startReceive();

    // Create and start serial bridge
    SerialBridge bridge(radio);
    bridge.start();

    ESP_LOGI(TAG, "=== BosMag Ready ===");
    ESP_LOGI(TAG, "Serial-to-LoRa bridge active");
    ESP_LOGI(TAG, "Waiting for data...");

    // Main task enters idle state (FreeRTOS handles everything)
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
