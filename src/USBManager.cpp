#include "USBManager.h"
#include "Config.h"

#include "driver/usb_serial_jtag.h"
#include "esp_vfs_usb_serial_jtag.h"
#include "esp_vfs_dev.h"
#include "esp_log.h"

static const char *TAG = "USBManager";

// ============================================================================
// USB SERIAL/JTAG INITIALIZATION
// ============================================================================

bool USBManager::init() {
    ESP_LOGI(TAG, "Initializing USB Serial/JTAG...");

    // Configure the driver buffer sizes
    usb_serial_jtag_driver_config_t usb_config = {
        .tx_buffer_size = USB_TX_BUFFER_SIZE,
        .rx_buffer_size = USB_RX_BUFFER_SIZE,
    };

    // Install the driver
    esp_err_t ret = usb_serial_jtag_driver_install(&usb_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install USB Serial/JTAG driver");
        return false;
    }

    // Connect VFS to this driver (makes printf work over USB)
    esp_vfs_usb_serial_jtag_use_driver();

    ESP_LOGI(TAG, "USB CDC initialized");
    return true;
}

// ============================================================================
// READ/WRITE OPERATIONS
// ============================================================================

int USBManager::read(uint8_t *buffer, size_t size, uint32_t timeout_ms) {
    return usb_serial_jtag_read_bytes(buffer, size, pdMS_TO_TICKS(timeout_ms));
}

int USBManager::write(const uint8_t *buffer, size_t size, uint32_t timeout_ms) {
    return usb_serial_jtag_write_bytes((const void *)buffer, size, pdMS_TO_TICKS(timeout_ms));
}