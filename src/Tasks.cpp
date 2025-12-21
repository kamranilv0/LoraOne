/* =============================================================================
   BosMag LoRa Transceiver - Task Implementations
   ============================================================================= */

#include "Tasks.h"
#include "RadioManager.h"
#include "Config.h"
#include "Utils.h"
#include "driver/usb_serial_jtag.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <RadioLib.h>

static const char *TAG = "TASKS";

// Serial RX Task - reads USB and transmits via LoRa
void serial_rx_task(void *pvParameters)
{
    uint8_t rx_buffer[SERIAL_RX_BUFFER_SIZE];
    
    while (1) {
        int length = usb_serial_jtag_read_bytes(rx_buffer, SERIAL_RX_BUFFER_SIZE, pdMS_TO_TICKS(10));
        
        if (length > 0) {
            ESP_LOGI(TAG, "USB RX: %d bytes", length);
            radio_transmit(rx_buffer, length);
        }
        
        if (length == 0) {
            vTaskDelay(pdMS_TO_TICKS(10)); 
        }
    }
}

// Radio RX Task - handles incoming LoRa packets
void radio_rx_task(void *pvParameters)
{
    while (1) {
        if (g_rx_data_available) {
            g_rx_data_available = false;

            if (xSemaphoreTake(g_radio_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
                // Read received data
                int16_t state = g_radio->readData(g_rx_buffer, LORA_RX_PAYLOAD_SIZE);
                
                if (state == RADIOLIB_ERR_NONE) {
                    // Get actual packet length
                    g_rx_length = g_radio->getPacketLength();
                    
                    // Get RSSI and SNR
                    float rssi = g_radio->getRSSI();
                    float snr = g_radio->getSNR();
                    
                    ESP_LOGI(TAG, "LoRa RX: %zu bytes | RSSI: %.1f dBm | SNR: %.1f dB", 
                                g_rx_length, rssi, snr);
                    log_hex_dump(g_rx_buffer, g_rx_length, "RX Data");

                    // Write to USB
                    if (g_rx_length > 0) {
                        usb_serial_jtag_write_bytes((const void *)g_rx_buffer, g_rx_length, pdMS_TO_TICKS(100));
                        usb_serial_jtag_write_bytes("\r\n", 2, pdMS_TO_TICKS(100));
                    }
                    
                } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
                    ESP_LOGW(TAG, "RX CRC error");
                } else {
                    ESP_LOGE(TAG, "RX failed, code: %d", state);
                }

                // Return to RX mode
                g_radio->startReceive();
                xSemaphoreGive(g_radio_mutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}