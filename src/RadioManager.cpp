#include "RadioManager.h"
#include "Config.h"
#include "Utils.h"

#include <RadioLib.h>
#include <EspHal.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"

static const char *TAG = "RadioManager";

// RadioLib objects
static Module* g_mod = nullptr;
static SX1262* g_radio = nullptr;
static spi_device_handle_t g_spi_handle = NULL;

// Global instance
RadioManager g_radioManager;

// Static ISR callbacks
static void IRAM_ATTR handleRadioIRQ() {
    // Check the state to decide which logic to run
    if (g_radioManager.getState() == RADIO_STATE_TX) {
        g_radioManager.handleTxDone();
    } else {
        // This covers RADIO_STATE_RX and RADIO_STATE_IDLE
        g_radioManager.handleRxDone();
    }
}

// ============================================================================
// CONSTRUCTOR / DESTRUCTOR
// ============================================================================

RadioManager::RadioManager()
    : m_rx_data_available(false)
    , m_tx_done(false)
    , m_rx_length(0)
    , m_state(RADIO_STATE_IDLE)
    , m_mutex(nullptr)
{
}

RadioManager::~RadioManager() {
    if (m_mutex) {
        vSemaphoreDelete(m_mutex);
    }
    if (g_radio) delete g_radio;
    if (g_mod) delete g_mod;
}

// ============================================================================
// INITIALIZATION
// ============================================================================

bool RadioManager::init() {
    ESP_LOGI(TAG, "Initializing SX1262 radio module with RadioLib...");
    
    // Create mutex
    m_mutex = xSemaphoreCreateMutex();
    if (!m_mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return false;
    }
    
    // Initialize SPI bus for direct register access
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
        return false;
    }

    spi_device_interface_config_t device_config = {
        .mode = 0,
        .clock_speed_hz = SPI_FREQUENCY,
        .spics_io_num = -1,
        .queue_size = 7,
        .pre_cb = NULL,
        .post_cb = NULL,
    };

    ret = spi_bus_add_device(SPI2_HOST, &device_config, &g_spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Install GPIO ISR service BEFORE creating RadioLib objects
    // This prevents RadioLib from trying to install it and failing
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Create HAL and RadioLib objects
    EspHal* hal = new EspHal(L_SCK, L_MISO, L_MOSI);
    g_mod = new Module(hal, L_NSS, L_DIO1, L_RST, L_BUSY);
    g_radio = new SX1262(g_mod);

    // Reset radio
    g_radio->reset();
    vTaskDelay(pdMS_TO_TICKS(50));

    // Begin initialization
    int16_t state = g_radio->begin(
        LORA_FREQUENCY,
        LORA_BANDWIDTH,
        LORA_SPREADING_FACTOR,
        LORA_CODING_RATE,
        LORA_SYNC_WORD,
        LORA_TX_POWER,
        LORA_PREAMBLE_LENGTH,
        TCXO_VOLTAGE
    );

    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "Radio initialization failed, code: %d", state);
        return false;
    }

    ESP_LOGI(TAG, "SX1262 initialized successfully");

    // Configure DIO2 as RF switch for TX (handles TX path automatically)
    state = g_radio->setDio2AsRfSwitch(true);
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "Failed to set DIO2 as RF switch, code: %d", state);
    } else {
        ESP_LOGI(TAG, "DIO2 configured as RF switch for TX path");
    }

    // DO NOT use setRfSwitchPins() - it conflicts with DIO2 RF switch
    // Manual control of L_RXEN for RX path
    ESP_LOGI(TAG, "Manual RF switch control: RX_EN on GPIO %d", L_RXEN);

    // Enable LNA boost
    writeLnaBoostRegister();

    // Set interrupt callback
    g_radio->setDio1Action(handleRadioIRQ);

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

    // Set initial state
    m_state = RADIO_STATE_IDLE; 
    
    return true;
}

// ============================================================================
// LNA BOOST REGISTER WRITE
// ============================================================================

void RadioManager::writeLnaBoostRegister() {
    uint8_t cmd_buffer[5] = {
        0x0D,           // WriteRegister command
        0x08, 0xAC,     // Register address
        0x96,           // Value to write
    };

    // Wait for BUSY pin
    uint32_t timeout = 1000;
    while (gpio_get_level((gpio_num_t)L_BUSY) && timeout--) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (timeout == 0) {
        ESP_LOGE(TAG, "Timeout waiting for BUSY pin before LNA boost write");
        return;
    }

    // SPI transaction
    gpio_set_level((gpio_num_t)L_NSS, 0);
    delay_us(10);

    spi_transaction_t trans = {
        .length = 32,
        .tx_buffer = cmd_buffer,
        .rx_buffer = NULL,
    };

    esp_err_t ret = spi_device_transmit(g_spi_handle, &trans);
    
    delay_us(10);
    gpio_set_level((gpio_num_t)L_NSS, 1);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "LNA boost enabled (register 0x08AC = 0x96)");
    } else {
        ESP_LOGE(TAG, "Failed to write LNA boost register");
    }

    // Wait for completion
    timeout = 1000;
    while (gpio_get_level((gpio_num_t)L_BUSY) && timeout--) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ============================================================================
// RF SWITCH CONTROL
// ============================================================================

void RadioManager::setRxMode() {
    // Enable RX path
    gpio_set_level((gpio_num_t)L_RXEN, 1);
}

void RadioManager::setTxMode() {
    // Disable RX path (TX path controlled by DIO2)
    gpio_set_level((gpio_num_t)L_RXEN, 0);
}

void RadioManager::setIdleMode() {
    // Disable both paths
    gpio_set_level((gpio_num_t)L_RXEN, 0);
}

// ============================================================================
// RX/TX OPERATIONS
// ============================================================================

// ============================================================================
// INTERRUPT HANDLERS - Disabled for continuous mode
// ============================================================================

void RadioManager::handleRxDone() {
    // In continuous mode, we don't use interrupts for RX
    // Interrupt is only used for TX completion
    m_rx_data_available = true;
    ESP_EARLY_LOGI(TAG, "RX interrupt triggered");
}

void RadioManager::handleTxDone() {
    m_tx_done = true;
    ESP_EARLY_LOGI(TAG, "TX interrupt: transmission complete");
}

// ============================================================================
// DATA ACCESS - Immediate read in continuous mode
// ============================================================================

bool RadioManager::readReceivedData() {
    if (!g_radio) {
        ESP_LOGE(TAG, "Radio not initialized");
        return false;
    }

    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(100)) == pdFALSE) {
        ESP_LOGE(TAG, "Mutex timeout in readReceivedData");
        return false;
    }

    // Read data - in continuous mode this must be called regularly
    int16_t state = g_radio->readData(m_rx_buffer, LORA_RX_PAYLOAD_SIZE);
    
    if (state > 0) {
        m_rx_length = state;
        ESP_LOGI(TAG, "Read %d bytes from radio", state);
        xSemaphoreGive(m_mutex);
        return true;
    } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
        // Normal timeout - no packet received
        m_rx_length = 0;
        xSemaphoreGive(m_mutex);
        return false;
    } else if (state == 0) {
        m_rx_length = 0;
        ESP_LOGW(TAG, "readData returned 0");
        xSemaphoreGive(m_mutex);
        return false;
    } else {
        m_rx_length = 0;
        if (state == RADIOLIB_ERR_CRC_MISMATCH) {
            ESP_LOGW(TAG, "RX CRC error");
        } else {
            ESP_LOGE(TAG, "RX read failed, code: %d", state);
        }
        xSemaphoreGive(m_mutex);
        return false;
    }
}

float RadioManager::getRSSI() {
    if (!g_radio) return 0.0f;
    
    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(50)) == pdFALSE) {
        return 0.0f;
    }
    
    float rssi = g_radio->getRSSI();
    xSemaphoreGive(m_mutex);
    return rssi;
}

float RadioManager::getSNR() {
    if (!g_radio) return 0.0f;
    
    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(50)) == pdFALSE) {
        return 0.0f;
    }
    
    float snr = g_radio->getSNR();
    xSemaphoreGive(m_mutex);
    return snr;
}

// ============================================================================
// TX OPERATION - Blocking transmit
// ============================================================================

void RadioManager::transmit(const uint8_t *data, size_t length) {
    if (!g_radio) {
        ESP_LOGE(TAG, "Radio not initialized");
        return;
    }

    if (length > LORA_TX_PAYLOAD_SIZE) {
        ESP_LOGE(TAG, "Payload too large: %zu > %d", length, LORA_TX_PAYLOAD_SIZE);
        return;
    }

    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(1000)) == pdFALSE) {
        ESP_LOGE(TAG, "Mutex timeout in transmit");
        return;
    }

    ESP_LOGI(TAG, "Transmitting %zu bytes via LoRa...", length);
    
    // Set state
    m_state = RADIO_STATE_TX;
    
    // Set RF switch to TX mode
    setTxMode();
    
    m_tx_done = false;

    // Use blocking transmit for reliability
    int16_t state = g_radio->transmit((uint8_t*)data, length);
    
    if (state == RADIOLIB_ERR_NONE) {
        ESP_LOGI(TAG, "TX complete");
    } else {
        ESP_LOGE(TAG, "TX failed, code: %d", state);
    }

    // Switch back to RX state
    m_state = RADIO_STATE_RX;

    xSemaphoreGive(m_mutex);
}

// ============================================================================
// RX OPERATION - Use continuous receive mode with timeout
// ============================================================================

void RadioManager::startReceive() {
    if (!g_radio) {
        ESP_LOGE(TAG, "Radio not initialized");
        return;
    }

    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(1000)) == pdFALSE) {
        ESP_LOGE(TAG, "Mutex timeout in startReceive");
        return;
    }

    // Clear any stale data
    m_rx_data_available = false;
    m_rx_length = 0;
    
    // Set state
    m_state = RADIO_STATE_RX;
    
    // Set RF switch to RX mode
    setRxMode();
    
    // Small delay for RF switch settling
    vTaskDelay(pdMS_TO_TICKS(5));
    
    // IMPORTANT: Use startReceive with NO timeout for continuous mode
    // This allows the radio to stay in RX and accept packets continuously
    int16_t state = g_radio->startReceive();
    
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "Failed to start RX, code: %d", state);
        m_state = RADIO_STATE_IDLE;
        setIdleMode();
    } else {
        ESP_LOGI(TAG, "Started continuous RX mode...");
    }

    xSemaphoreGive(m_mutex);
}

// ============================================================================
// HELPER: Check if TX is in progress
// ============================================================================

bool RadioManager::isTxInProgress() {
    return (m_state == RADIO_STATE_TX);
}