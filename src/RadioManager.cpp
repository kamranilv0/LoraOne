#include "RadioManager.h"
#include "HardwareInit.h"
#include "Utils.h"
#include "Config.h"

#include <RadioLib.h>
#include <EspHal.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/task.h"

static const char* TAG = "RadioManager";

// Static member initialization
RadioManager* RadioManager::s_instance = nullptr;

RadioManager::RadioManager()
    : m_module(nullptr)
    , m_radio(nullptr)
    , m_radioState(RADIO_STATE_IDLE)
    , m_rxDataAvailable(false)
    , m_txDone(false)
    , m_mutex(nullptr)
{
    m_mutex = xSemaphoreCreateMutex();
    if (m_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create radio mutex");
    }
}

RadioManager::~RadioManager()
{
    if (m_mutex != NULL) {
        vSemaphoreDelete(m_mutex);
    }
    
    if (m_radio != nullptr) {
        delete m_radio;
    }
    
    if (m_module != nullptr) {
        delete m_module;
    }
}

RadioManager& RadioManager::getInstance()
{
    if (s_instance == nullptr) {
        s_instance = new RadioManager();
    }
    return *s_instance;
}

void RadioManager::writeLNABoostRegister()
{
    // SX1262 command: WriteRegister (0x0D)
    // Register address: 0x08AC (LNA boost)
    // Value: 0x96 (enable boost)
    
    uint8_t cmd_buffer[5] = {
        0x0D,           // WriteRegister command
        0x08, 0xAC,     // Register address (0x08AC)
        0x96,           // Value to write
    };

    // Wait for radio to be ready (BUSY pin low)
    uint32_t timeout = 1000;
    while (gpio_get_level((gpio_num_t)L_BUSY) && timeout--) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (timeout == 0) {
        ESP_LOGE(TAG, "Timeout waiting for BUSY pin before LNA boost write");
        return;
    }

    spi_device_handle_t spi_handle = HardwareInit::getSPIHandle();
    if (spi_handle == NULL) {
        ESP_LOGE(TAG, "SPI handle not initialized");
        return;
    }

    // Perform SPI transaction
    gpio_set_level((gpio_num_t)L_NSS, 0);  // CS low
    Utils::delayMicroseconds(10);

    spi_transaction_t trans = {
        .length = 32,  // 4 bytes * 8 bits
        .tx_buffer = cmd_buffer,
        .rx_buffer = NULL,
    };

    esp_err_t ret = spi_device_transmit(spi_handle, &trans);
    
    Utils::delayMicroseconds(10);
    gpio_set_level((gpio_num_t)L_NSS, 1);  // CS high

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "LNA boost enabled (register 0x08AC = 0x96)");
    } else {
        ESP_LOGE(TAG, "Failed to write LNA boost register: %s", esp_err_to_name(ret));
    }

    // Wait for operation to complete
    timeout = 1000;
    while (gpio_get_level((gpio_num_t)L_BUSY) && timeout--) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void RadioManager::init()
{
    ESP_LOGI(TAG, "Initializing SX1262 radio module with RadioLib...");

    // Create HAL instance for ESP-IDF
    EspHal* hal = new EspHal(L_SCK, L_MISO, L_MOSI);
    
    // Create RadioLib module instance with HAL and pin configuration
    m_module = new Module(hal, L_NSS, L_DIO1, L_RST, L_BUSY);
    
    // Create SX1262 radio instance
    m_radio = new SX1262(m_module);

    // Perform hardware reset
    m_radio->reset();
    vTaskDelay(pdMS_TO_TICKS(50));

    // Begin initialization
    ESP_LOGI(TAG, "Configuring SX1262...");
    
    int16_t state = m_radio->begin(
        LORA_FREQUENCY,           // Frequency in MHz
        LORA_BANDWIDTH,           // Bandwidth in kHz
        LORA_SPREADING_FACTOR,    // Spreading factor
        LORA_CODING_RATE,         // Coding rate
        LORA_SYNC_WORD,           // Sync word (0x12 = private, 0x34 = public)
        LORA_TX_POWER,            // Output power in dBm
        LORA_PREAMBLE_LENGTH,     // Preamble length
        TCXO_VOLTAGE              // TCXO voltage (0 = no TCXO)
    );

    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "Radio initialization failed, code: %d", state);
        return;
    }

    ESP_LOGI(TAG, "SX1262 initialized successfully");

    // Enable DIO2 as RF switch for TX path (FEM control)
    state = m_radio->setDio2AsRfSwitch(true);
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "Failed to set DIO2 as RF switch, code: %d", state);
    } else {
        ESP_LOGI(TAG, "DIO2 configured as RF switch for TX");
    }

    // Configure RF switch pins: L_RXEN for RX path control
    m_radio->setRfSwitchPins(L_RXEN, RADIOLIB_NC);
    ESP_LOGI(TAG, "RF switch pins configured: RX_EN on GPIO %d", L_RXEN);

    // Enable LNA boost for improved RX sensitivity via direct SPI
    writeLNABoostRegister();

    // Set DIO1 interrupt action callbacks
    m_radio->setDio1Action(RadioManager::rxDoneCallback);

    // Configure CRC
    state = m_radio->setCRC(true);
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGW(TAG, "Failed to enable CRC, code: %d", state);
    }

    // Log final configuration
    ESP_LOGI(TAG, "Radio configuration:");
    ESP_LOGI(TAG, "  Frequency: %.3f MHz", LORA_FREQUENCY);
    ESP_LOGI(TAG, "  Bandwidth: %.1f kHz", LORA_BANDWIDTH);
    ESP_LOGI(TAG, "  SF: %d, CR: 4/%d", LORA_SPREADING_FACTOR, LORA_CODING_RATE);
    ESP_LOGI(TAG, "  TX Power: %d dBm", LORA_TX_POWER);
    ESP_LOGI(TAG, "  Sync Word: 0x%02X", LORA_SYNC_WORD);
    ESP_LOGI(TAG, "  Preamble: %d symbols", LORA_PREAMBLE_LENGTH);
    ESP_LOGI(TAG, "  DIO2 RF Switch: ENABLED (TX)");
    ESP_LOGI(TAG, "  RF Switch Control: ENABLED (RX via GPIO %d)", L_RXEN);
    ESP_LOGI(TAG, "  LNA Boost: ENABLED");
    ESP_LOGI(TAG, "  CRC: ENABLED");
    
    m_radioState = RADIO_STATE_IDLE;
}

void RadioManager::startReceive()
{
    if (m_radio == nullptr) {
        ESP_LOGE(TAG, "Radio not initialized");
        return;
    }

    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(1000)) == pdFALSE) {
        ESP_LOGE(TAG, "Radio mutex timeout in startReceive");
        return;
    }

    // Start continuous RX mode
    int16_t state = m_radio->startReceive();
    
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "Failed to start RX, code: %d", state);
        xSemaphoreGive(m_mutex);
        return;
    }

    m_radioState = RADIO_STATE_RX;
    xSemaphoreGive(m_mutex);
    ESP_LOGI(TAG, "Started listening for LoRa packets...");
}

void RadioManager::transmit(const uint8_t* data, size_t length)
{
    if (m_radio == nullptr) {
        ESP_LOGE(TAG, "Radio not initialized");
        return;
    }

    if (length > LORA_TX_PAYLOAD_SIZE) {
        ESP_LOGE(TAG, "Payload too large: %zu > %d", length, LORA_TX_PAYLOAD_SIZE);
        return;
    }

    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(1000)) == pdFALSE) {
        ESP_LOGE(TAG, "Radio mutex timeout in transmit");
        return;
    }

    ESP_LOGI(TAG, "Transmitting %zu bytes via LoRa...", length);
    Utils::logHexDump(data, length, "TX Data");

    // Set TX done callback
    m_txDone = false;
    m_radio->setDio1Action(RadioManager::txDoneCallback);

    // Transmit the data
    int16_t state = m_radio->startTransmit((uint8_t*)data, length);
    
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "Failed to start TX, code: %d", state);
        xSemaphoreGive(m_mutex);
        return;
    }

    m_radioState = RADIO_STATE_TX;

    // Wait for TX done with timeout
    uint32_t timeout_ms = 5000;
    uint32_t start_time = xTaskGetTickCount();
    
    while (!m_txDone) {
        if ((xTaskGetTickCount() - start_time) > pdMS_TO_TICKS(timeout_ms)) {
            ESP_LOGE(TAG, "TX timeout!");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (m_txDone) {
        ESP_LOGI(TAG, "TX complete");
    }

    // Restore RX callback and return to RX mode
    m_radio->setDio1Action(RadioManager::rxDoneCallback);
    
    int16_t rx_state = m_radio->startReceive();
    if (rx_state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "Failed to restart RX after TX, code: %d", rx_state);
    }

    m_radioState = RADIO_STATE_RX;
    xSemaphoreGive(m_mutex);
}

bool RadioManager::hasDataAvailable()
{
    return m_rxDataAvailable;
}

int16_t RadioManager::readData(uint8_t* buffer, size_t maxLength)
{
    if (m_radio == nullptr) {
        ESP_LOGE(TAG, "Radio not initialized");
        return -1;
    }

    if (!m_rxDataAvailable) {
        return 0;
    }

    m_rxDataAvailable = false;

    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Radio mutex timeout in readData");
        return -1;
    }

    // Read received data from radio
    int16_t state = m_radio->readData(buffer, maxLength);
    
    if (state < 0) {
        if (state == RADIOLIB_ERR_CRC_MISMATCH) {
            ESP_LOGW(TAG, "RX CRC error");
        } else {
            ESP_LOGE(TAG, "RX failed, code: %d", state);
        }
    }

    // Return to RX mode
    m_radio->startReceive();
    xSemaphoreGive(m_mutex);

    return state;
}

float RadioManager::getRSSI()
{
    if (m_radio == nullptr) {
        return 0.0f;
    }
    return m_radio->getRSSI();
}

float RadioManager::getSNR()
{
    if (m_radio == nullptr) {
        return 0.0f;
    }
    return m_radio->getSNR();
}

void IRAM_ATTR RadioManager::rxDoneCallback()
{
    if (s_instance != nullptr) {
        s_instance->m_rxDataAvailable = true;
    }
}

void IRAM_ATTR RadioManager::txDoneCallback()
{
    if (s_instance != nullptr) {
        s_instance->m_txDone = true;
    }
}
