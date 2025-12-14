#ifndef RADIO_MANAGER_H
#define RADIO_MANAGER_H

#include <stdint.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "Config.h"

// Forward declarations
class Module;
class SX1262;

class RadioManager {
public:
    /**
     * @brief Get singleton instance
     * @return Reference to RadioManager instance
     */
    static RadioManager& getInstance();
    
    /**
     * @brief Initialize LoRa radio module
     */
    void init();
    
    /**
     * @brief Start receiving mode
     */
    void startReceive();
    
    /**
     * @brief Transmit data via LoRa
     * @param data Pointer to data buffer
     * @param length Length of data to transmit
     */
    void transmit(const uint8_t* data, size_t length);
    
    /**
     * @brief Check if RX data is available
     * @return true if data available, false otherwise
     */
    bool hasDataAvailable();
    
    /**
     * @brief Read received data
     * @param buffer Buffer to store received data
     * @param maxLength Maximum buffer size
     * @return Number of bytes read, negative on error
     */
    int16_t readData(uint8_t* buffer, size_t maxLength);
    
    /**
     * @brief Get RSSI of last received packet
     * @return RSSI in dBm
     */
    float getRSSI();
    
    /**
     * @brief Get SNR of last received packet
     * @return SNR in dB
     */
    float getSNR();
    
    /**
     * @brief Get current radio state
     * @return Current radio state
     */
    radio_state_t getState() const { return m_radioState; }
    
    /**
     * @brief RX done interrupt callback (called from ISR)
     */
    static void IRAM_ATTR rxDoneCallback();
    
    /**
     * @brief TX done interrupt callback (called from ISR)
     */
    static void IRAM_ATTR txDoneCallback();
    
private:
    RadioManager();
    ~RadioManager();
    
    // Prevent copying
    RadioManager(const RadioManager&) = delete;
    RadioManager& operator=(const RadioManager&) = delete;
    
    /**
     * @brief Write LNA boost register for improved RX sensitivity
     */
    void writeLNABoostRegister();
    
    // RadioLib instances
    Module* m_module;
    SX1262* m_radio;
    
    // State management
    volatile radio_state_t m_radioState;
    volatile bool m_rxDataAvailable;
    volatile bool m_txDone;
    
    // Thread safety
    SemaphoreHandle_t m_mutex;
    
    // Singleton instance
    static RadioManager* s_instance;
};

#endif // RADIO_MANAGER_H
