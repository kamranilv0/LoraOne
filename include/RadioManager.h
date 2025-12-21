#ifndef RADIO_MANAGER_H
#define RADIO_MANAGER_H

#include <stdint.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "Config.h"

// ============================================================================
// RADIO MANAGER CLASS
// ============================================================================
class RadioManager {
public:
    RadioManager();
    ~RadioManager();
    
    // Initialization
    bool init();
    bool isTxInProgress();
    
    // RX/TX operations
    void startReceive();
    void transmit(const uint8_t *data, size_t length);
    radio_state_t getState() const { return m_state; }
    bool pollReceive();  // If you're using the poll method

    
    // State queries
    bool isRxDataAvailable() const { return m_rx_data_available; }
    bool isTxDone() const { return m_tx_done; }
    void clearRxFlag() { m_rx_data_available = false; }
    void clearTxFlag() { m_tx_done = false; }
    
    // Data access
    const uint8_t* getRxBuffer() const { return m_rx_buffer; }
    size_t getRxLength() const { return m_rx_length; }

    bool readReceivedData();
    
    // Get current radio state
    float getRSSI();
    float getSNR();
    
    // Callbacks (to be called from ISR)
    void handleRxDone();
    void handleTxDone();
    
private:
    // Helper methods
    void writeLnaBoostRegister();
    void setRxMode();
    void setTxMode();
    void setIdleMode();
    void reset();

    // State variables
    volatile bool m_rx_data_available;
    volatile bool m_tx_done;
    uint8_t m_rx_buffer[255];
    size_t m_rx_length;
    radio_state_t m_state;
    // Synchronization
    SemaphoreHandle_t m_mutex;
};

// Global RadioManager instance (declared extern, defined in RadioManager.cpp)
extern RadioManager g_radioManager;

#endif // RADIO_MANAGER_H