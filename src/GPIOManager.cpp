/* =============================================================================
   BosMag LoRa Transceiver - GPIO Manager Implementation
   ============================================================================= */

#include "GPIOManager.h"
#include "Config.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "GPIO";

void gpio_setup()
{
    ESP_LOGI(TAG, "Configuring GPIO pins...");

    // Configure L_NSS as output (SPI chip select)
    gpio_config_t nss_config = {
        .pin_bit_mask = (1ULL << L_NSS),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&nss_config);
    gpio_set_level((gpio_num_t)L_NSS, 1);  // SPI CS idle HIGH

    // Configure L_RST as output (reset pin)
    gpio_config_t rst_config = {
        .pin_bit_mask = (1ULL << L_RST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&rst_config);
    gpio_set_level((gpio_num_t)L_RST, 1);  // Reset inactive HIGH

    // Configure L_BUSY as input (read-only)
    gpio_config_t busy_config = {
        .pin_bit_mask = (1ULL << L_BUSY),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&busy_config);

    // Configure L_DIO1 as input with interrupt (RX/TX done)
    gpio_config_t dio1_config = {
        .pin_bit_mask = (1ULL << L_DIO1),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,  // Rising edge
    };
    gpio_config(&dio1_config);

    // Configure L_RXEN as output (RF switch - RX path control)
    gpio_config_t rxen_config = {
        .pin_bit_mask = (1ULL << L_RXEN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&rxen_config);
    gpio_set_level((gpio_num_t)L_RXEN, 0);  // Start with LNA disabled

    ESP_LOGI(TAG, "GPIO configuration complete");
}