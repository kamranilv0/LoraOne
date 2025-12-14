#include "SerialBridge.h"
#include "Config.h"
#include "Utils.h"

#include "driver/uart.h"
#include "esp_log.h"

static const char* TAG = "SerialBridge";

SerialBridge::SerialBridge(RadioManager& radio)
    : m_radio(radio)
    , m_serialTaskHandle(nullptr)
    , m_radioTaskHandle(nullptr)
    , m_running(false)
{
}

SerialBridge::~SerialBridge()
{
    stop();
}

void SerialBridge::start()
{
    if (m_running) {
        ESP_LOGW(TAG, "Serial bridge already running");
        return;
    }

    m_running = true;

    // Create serial RX task (UART -> LoRa)
    xTaskCreate(
        serialRxTaskWrapper,
        "serial_rx",
        4096,
        this,
        5,
        &m_serialTaskHandle
    );

    // Create radio RX task (LoRa -> UART)
    xTaskCreate(
        radioRxTaskWrapper,
        "radio_rx",
        4096,
        this,
        6,
        &m_radioTaskHandle
    );

    ESP_LOGI(TAG, "Serial bridge started");
}

void SerialBridge::stop()
{
    if (!m_running) {
        return;
    }

    m_running = false;

    // Delete tasks
    if (m_serialTaskHandle != nullptr) {
        vTaskDelete(m_serialTaskHandle);
        m_serialTaskHandle = nullptr;
    }

    if (m_radioTaskHandle != nullptr) {
        vTaskDelete(m_radioTaskHandle);
        m_radioTaskHandle = nullptr;
    }

    ESP_LOGI(TAG, "Serial bridge stopped");
}

void SerialBridge::serialRxTaskWrapper(void* pvParameters)
{
    SerialBridge* bridge = static_cast<SerialBridge*>(pvParameters);
    bridge->serialRxTask();
}

void SerialBridge::radioRxTaskWrapper(void* pvParameters)
{
    SerialBridge* bridge = static_cast<SerialBridge*>(pvParameters);
    bridge->radioRxTask();
}

void SerialBridge::serialRxTask()
{
    uint8_t rx_buffer[SERIAL_RX_BUFFER_SIZE];
    const uart_port_t uart_port = UART_NUM_0;
    
    ESP_LOGI(TAG, "Serial RX task started");
    
    while (m_running) {
        // Read from UART
        int length = uart_read_bytes(
            uart_port,
            rx_buffer,
            SERIAL_RX_BUFFER_SIZE,
            pdMS_TO_TICKS(SERIAL_RX_TIMEOUT_MS)
        );
        
        if (length > 0) {
            // Data received from Serial - transmit via LoRa
            ESP_LOGI(TAG, "Serial RX: %d bytes", length);
            m_radio.transmit(rx_buffer, length);
        }
    }
    
    ESP_LOGI(TAG, "Serial RX task stopped");
}

void SerialBridge::radioRxTask()
{
    uint8_t rx_buffer[LORA_RX_PAYLOAD_SIZE];
    const uart_port_t uart_port = UART_NUM_0;
    
    ESP_LOGI(TAG, "Radio RX task started");
    
    while (m_running) {
        // Wait for DIO1 interrupt indicating RX data available
        if (m_radio.hasDataAvailable()) {
            // Read received data from radio
            int16_t length = m_radio.readData(rx_buffer, LORA_RX_PAYLOAD_SIZE);
            
            if (length > 0) {
                // Get RSSI and SNR
                float rssi = m_radio.getRSSI();
                float snr = m_radio.getSNR();
                
                ESP_LOGI(TAG, "LoRa RX: %d bytes | RSSI: %.1f dBm | SNR: %.1f dB", 
                         length, rssi, snr);
                Utils::logHexDump(rx_buffer, length, "RX Data");
                
                // Echo to UART
                uart_write_bytes(uart_port, (const char*)rx_buffer, length);
                uart_write_bytes(uart_port, "\r\n", 2);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    ESP_LOGI(TAG, "Radio RX task stopped");
}
