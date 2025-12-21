/* =============================================================================
   BosMag LoRa Transceiver - USB CDC Manager Implementation
   ============================================================================= */

#include "USBManager.h"
#include "driver/usb_serial_jtag.h"
#include "esp_vfs_usb_serial_jtag.h"
#include "esp_vfs_dev.h"
#include "esp_log.h"

static const char *TAG = "USB";

void usb_cdc_init()
{
    ESP_LOGI(TAG, "Initializing USB Serial/JTAG...");

    // Configure the driver buffer sizes
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .tx_buffer_size = 1024,
        .rx_buffer_size = 1024,
    };

    // Install the driver
    esp_err_t ret = usb_serial_jtag_driver_install(&usb_serial_jtag_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install USB Serial/JTAG driver");
        return;
    }

    // Connect the Virtual File System to this driver
    esp_vfs_usb_serial_jtag_use_driver();

    ESP_LOGI(TAG, "USB CDC initialized");
}