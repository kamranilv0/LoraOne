#include "Tasks.h"
#include "Config.h"
#include "Utils.h"
#include "RadioManager.h"
#include "USBManager.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>

static const char *TAG = "Tasks";

// ============================================================================
// RADIO RX TASK - Blocking receive (most reliable)
// ============================================================================

void radio_rx_task(void *pvParameters) {
    ESP_LOGI(TAG, "Radio RX task started (blocking mode)");
    
    uint8_t rx_buffer[LORA_RX_PAYLOAD_SIZE];
    uint32_t last_check = 0;
    
    // Initial start of RX
    g_radioManager.startReceive();
    
    while (1) {
        // Poll for interrupt flag (set by DIO1 ISR)
        if (g_radioManager.isRxDataAvailable()) {
            ESP_LOGI(TAG, "RX interrupt detected");
            
            // Clear flag immediately
            g_radioManager.clearRxFlag();
            
            // Read the received data
            if (g_radioManager.readReceivedData()) {
                const uint8_t *rx_data = g_radioManager.getRxBuffer();
                size_t rx_length = g_radioManager.getRxLength();
                
                if (rx_length > 0) {
                    // Copy to local buffer
                    memcpy(rx_buffer, rx_data, rx_length);
                    
                    // Get signal metrics
                    float rssi = g_radioManager.getRSSI();
                    float snr = g_radioManager.getSNR();
                    
                    ESP_LOGI(TAG, "LoRa RX: %zu bytes | RSSI: %.1f dBm | SNR: %.1f dB", 
                             rx_length, rssi, snr);
                    log_hex_dump(rx_buffer, rx_length, "LoRa->USB");
                    
                    // Forward to USB
                    int written = USBManager::write(rx_buffer, rx_length, 100);
                    if (written != rx_length) {
                        ESP_LOGW(TAG, "USB write incomplete: %d/%zu bytes", written, rx_length);
                    } else {
                        ESP_LOGI(TAG, "Successfully forwarded %d bytes to USB", written);
                    }
                }
            } else {
                ESP_LOGW(TAG, "Failed to read RX data - restarting RX");
            }
            
            // CRITICAL: Always restart RX after reading a packet
            g_radioManager.startReceive();
        }
        
        // Debug heartbeat
        if ((xTaskGetTickCount() - last_check) > pdMS_TO_TICKS(10000)) {
            ESP_LOGD(TAG, "Radio RX task alive, waiting for packets...");
            last_check = xTaskGetTickCount();
        }
        
        // CRITICAL: Keep delay very short for fast response
        // 1ms is the minimum practical delay
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ============================================================================
// SERIAL RX TASK - Handle USB to LoRa forwarding
// ============================================================================

void serial_rx_task(void *pvParameters) {
    uint8_t rx_buffer[SERIAL_RX_BUFFER_SIZE];
    
    ESP_LOGI(TAG, "Serial RX task started");
    
    while (1) {
        // Read from USB with short timeout
        int length = USBManager::read(rx_buffer, SERIAL_RX_BUFFER_SIZE, 10);
        
        if (length > 0) {
            ESP_LOGI(TAG, "USB RX: %d bytes", length);
            log_hex_dump(rx_buffer, length, "USB->LoRa");
            
            // Transmit via LoRa (blocking call)
            g_radioManager.transmit(rx_buffer, length);
            
            // Restart RX mode after TX completes
            vTaskDelay(pdMS_TO_TICKS(20)); // Brief delay after TX
            g_radioManager.startReceive();
            ESP_LOGI(TAG, "TX complete, resumed RX mode");
        }
        
        // Yield if no data
        if (length == 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

// ============================================================================
// TASK INITIALIZATION
// ============================================================================

void tasks_init() {
    ESP_LOGI(TAG, "Creating tasks...");
    
    // Create serial RX task (USB -> LoRa)
    xTaskCreate(serial_rx_task, "serial_rx", 4096, NULL, 5, NULL);
    
    // Create radio RX task (LoRa -> USB) with HIGHER priority
    // Higher priority ensures packets are read quickly
    xTaskCreate(radio_rx_task, "radio_rx", 4096, NULL, 6, NULL);
    
    ESP_LOGI(TAG, "Tasks created successfully");
}