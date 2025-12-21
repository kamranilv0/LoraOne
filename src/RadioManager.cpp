/* =============================================================================
   BosMag LoRa Transceiver - Radio Manager Implementation
   ============================================================================= */

#include "RadioManager.h"
#include "Config.h"
#include "Utils.h"
#include "driver/spi_master.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <RadioLib.h>
#include <EspHal.h>

static const char *TAG = "RADIO";

// Global state variables
volatile bool g_rx_data_available = false;
volatile bool g_tx_done = false;
uint8_t g_rx_buffer[LORA_RX_PAYLOAD_SIZE];
size_t g_rx_length = 0;
SemaphoreHandle_t g_radio_mutex = NULL;

// RadioLib objects
static Module* g_mod = nullptr;
SX1262* g_radio = nullptr;  // Non-static for Tasks.cpp access
static spi_device_handle_t g_spi_handle = NULL;

// Interrupt callbacks
void IRAM_ATTR setFlag_RxDone()
{
    g_rx_data_available = true;
}

void IRAM_ATTR setFlag_TxDone()
{
    g_tx_done = true;
}

// SPI Setup
void spi_setup()
{
    ESP_LOGI(TAG, "Initializing SPI bus...");
    
    spi_bus_config_t bus_config = {
        .mosi_io_num = L_MOSI,
        .miso_io_num = L_MISO,
        .sclk_io_num = L_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
        .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_GPIO_PINS,
    };

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return;
    }

    spi_device_interface_config_t device_config = {
        .mode = 0,  // SPI mode 0
        .clock_speed_hz = SPI_FREQUENCY,
        .spics_io_num = -1,  // Manual CS control
        .queue_size = 7,
        .pre_cb = NULL,
        .post_cb = NULL,
    };

    ret = spi_bus_add_device(SPI2_HOST, &device_config, &g_spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "SPI bus initialized");
}

// Radio Initialization
void radio_init()
{
    ESP_LOGI(TAG, "Initializing SX1262 radio module with RadioLib...");

    // Create HAL instance
    EspHal* g_hal = new EspHal(L_SCK, L_MISO, L_MOSI);
    
    // Create module instance
    g_mod = new Module(g_hal, L_NSS, L_DIO1, L_RST, L_BUSY);
    
    // Create radio instance
    g_radio = new SX1262(g_mod);
    
    // Hardware reset
    g_radio->reset();
    vTaskDelay(pdMS_TO_TICKS(50));

    // Begin initialization
    ESP_LOGI(TAG, "Configuring SX1262...");
    
    int16_t state = g_radio->begin(
        LORA_FREQUENCY,
        LORA_BANDWIDTH,
        LORA_SPREADING_FACTOR,
        LORA_CODING_RATE,
        LORA_SYNC_WORD,
        LORA_TX_POWER,
        LORA_PREAMBLE_LENGTH,
        TCXO_VOLTAGE,
        LORA_USE_DCDC
    );
    
    g_radio->setRxBoostedGainMode(true);
    g_radio->setCurrentLimit(140.0);

    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "Radio initialization failed, code: %d", state);
        return;
    }

    ESP_LOGI(TAG, "SX1262 initialized successfully");

    // Configure DIO2 as RF switch for TX
    state = g_radio->setDio2AsRfSwitch(true);
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "Failed to set DIO2 as RF switch, code: %d", state);
    } else {
        ESP_LOGI(TAG, "DIO2 configured as RF switch for TX");
    }

    // Configure RF switch pins for RX
    g_radio->setRfSwitchPins(L_RXEN, RADIOLIB_NC);
    ESP_LOGI(TAG, "RF switch pins configured: RX_EN on GPIO %d", L_RXEN);

    // Set explicit header mode
    state = g_radio->explicitHeader();
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGW(TAG, "Failed to set explicit header, code: %d", state);
    } else {
        ESP_LOGI(TAG, "Explicit header mode enabled");
    }

    // Set sync word
    state = g_radio->setSyncWord(LORA_SYNC_WORD);
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGW(TAG, "Failed to set sync word, code: %d", state);
    }

    // Set DIO1 interrupt callback
    g_radio->setDio1Action(setFlag_RxDone);

    // Enable CRC
    state = g_radio->setCRC(true);
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGW(TAG, "Failed to enable CRC, code: %d", state);
    }

    // Log configuration
    ESP_LOGI(TAG, "Radio configuration:");
    ESP_LOGI(TAG, "  Frequency: %.3f MHz", LORA_FREQUENCY);
    ESP_LOGI(TAG, "  Bandwidth: %.1f kHz", LORA_BANDWIDTH);
    ESP_LOGI(TAG, "  SF: %d, CR: 4/%d", LORA_SPREADING_FACTOR, LORA_CODING_RATE);
    ESP_LOGI(TAG, "  TX Power: %d dBm", LORA_TX_POWER);
    ESP_LOGI(TAG, "  Sync Word: 0x%02X", LORA_SYNC_WORD);
    ESP_LOGI(TAG, "  Preamble: %d symbols", LORA_PREAMBLE_LENGTH);
}

// Start RX Mode
void radio_start_rx()
{
    if (g_radio == nullptr) {
        ESP_LOGE(TAG, "Radio not initialized");
        return;
    }

    if (xSemaphoreTake(g_radio_mutex, pdMS_TO_TICKS(1000)) == pdFALSE) {
        ESP_LOGE(TAG, "Radio mutex timeout in start_rx");
        return;
    }

    int16_t state = g_radio->startReceive();
    
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "Failed to start RX, code: %d", state);
        xSemaphoreGive(g_radio_mutex);
        return;
    }

    xSemaphoreGive(g_radio_mutex);
    ESP_LOGI(TAG, "Started listening for LoRa packets...");
}

// Transmit Data
void radio_transmit(const uint8_t *data, size_t length)
{
    if (g_radio == nullptr) {
        ESP_LOGE(TAG, "Radio not initialized");
        return;
    }

    if (length > LORA_TX_PAYLOAD_SIZE) {
        ESP_LOGE(TAG, "Payload too large: %zu > %d", length, LORA_TX_PAYLOAD_SIZE);
        return;
    }

    if (xSemaphoreTake(g_radio_mutex, pdMS_TO_TICKS(1000)) == pdFALSE) {
        ESP_LOGE(TAG, "Radio mutex timeout in transmit");
        return;
    }

    ESP_LOGI(TAG, "Transmitting %zu bytes via LoRa...", length);
    log_hex_dump(data, length, "TX Data");

    // Set TX callback
    g_tx_done = false;
    g_radio->setDio1Action(setFlag_TxDone);

    // Start transmission
    int16_t state = g_radio->startTransmit((uint8_t*)data, length);
    
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "Failed to start TX, code: %d", state);
        xSemaphoreGive(g_radio_mutex);
        return;
    }

    // Wait for TX done
    uint32_t timeout_ms = 5000;
    uint32_t start_time = xTaskGetTickCount();
    
    while (!g_tx_done) {
        if ((xTaskGetTickCount() - start_time) > pdMS_TO_TICKS(timeout_ms)) {
            ESP_LOGE(TAG, "TX timeout!");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (g_tx_done) {
        ESP_LOGI(TAG, "TX complete");
    }

    // Restore RX callback and return to RX
    g_radio->setDio1Action(setFlag_RxDone);
    
    int16_t rx_state = g_radio->startReceive();
    if (rx_state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "Failed to restart RX after TX, code: %d", rx_state);
    }

    xSemaphoreGive(g_radio_mutex);
}