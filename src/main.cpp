/* =============================================================================
   BosMag LoRa Transceiver - Main Application
   ESP32-S3 + Ebyte E22-900M-30S (SX1262)
   
   Features:
   - Serial-to-LoRa transparent bridge
   - RadioLib RF switch control via setRfSwitchPins
   - LNA boost enabled for improved RX sensitivity
   - Interrupt-driven RX with DIO1
   - Safe RF state management
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

extern "C" void app_main()
{
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

    // Create radio mutex
    g_radio_mutex = xSemaphoreCreateMutex();
    
    if (g_radio_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create radio mutex");
        return;
    }

    // Initialize peripherals
    gpio_setup();
    spi_setup();
    usb_cdc_init();
    radio_init();

    // Start RX mode
    radio_start_rx();

    // Create tasks
    xTaskCreate(serial_rx_task, "serial_rx", 4096, NULL, 5, NULL);
    xTaskCreate(radio_rx_task, "radio_rx", 4096, NULL, 6, NULL);

    ESP_LOGI(TAG, "=== BosMag Ready ===");
    ESP_LOGI(TAG, "Serial-to-LoRa bridge active");
    ESP_LOGI(TAG, "Waiting for data...");

    // Main loop (idle)
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}