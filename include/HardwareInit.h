#ifndef HARDWARE_INIT_H
#define HARDWARE_INIT_H

#include "driver/spi_master.h"

namespace HardwareInit {
    /**
     * @brief Initialize all GPIO pins for LoRa module
     */
    void initGPIO();
    
    /**
     * @brief Initialize SPI bus for LoRa communication
     */
    void initSPI();
    
    /**
     * @brief Initialize UART for serial communication
     */
    void initUART();
    
    /**
     * @brief Initialize all hardware (GPIO, SPI, UART)
     */
    void initAll();
    
    /**
     * @brief Get SPI device handle for direct register access
     * @return SPI device handle
     */
    spi_device_handle_t getSPIHandle();
}

#endif // HARDWARE_INIT_H
