#ifndef SERIAL_BRIDGE_H
#define SERIAL_BRIDGE_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "RadioManager.h"

class SerialBridge {
public:
    /**
     * @brief Constructor
     * @param radio Reference to RadioManager instance
     */
    explicit SerialBridge(RadioManager& radio);
    
    /**
     * @brief Destructor
     */
    ~SerialBridge();
    
    /**
     * @brief Start the serial bridge tasks
     */
    void start();
    
    /**
     * @brief Stop the serial bridge tasks
     */
    void stop();
    
private:
    /**
     * @brief Serial RX task - forwards UART data to LoRa
     * @param pvParameters Task parameters (pointer to SerialBridge instance)
     */
    static void serialRxTaskWrapper(void* pvParameters);
    
    /**
     * @brief Radio RX task - forwards LoRa data to UART
     * @param pvParameters Task parameters (pointer to SerialBridge instance)
     */
    static void radioRxTaskWrapper(void* pvParameters);
    
    /**
     * @brief Serial RX task implementation
     */
    void serialRxTask();
    
    /**
     * @brief Radio RX task implementation
     */
    void radioRxTask();
    
    // Reference to radio manager
    RadioManager& m_radio;
    
    // Task handles
    TaskHandle_t m_serialTaskHandle;
    TaskHandle_t m_radioTaskHandle;
    
    // Running flag
    volatile bool m_running;
};

#endif // SERIAL_BRIDGE_H
