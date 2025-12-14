#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>
#include <stddef.h>

namespace Utils {
    /**
     * @brief Log data in hexadecimal format
     * @param data Pointer to data buffer
     * @param len Length of data
     * @param prefix Prefix string for log message
     */
    void logHexDump(const uint8_t* data, size_t len, const char* prefix);
    
    /**
     * @brief Delay execution for specified microseconds
     * @param us Microseconds to delay
     */
    void delayMicroseconds(uint32_t us);
    
    /**
     * @brief Print startup banner
     */
    void printBanner();
}

#endif // UTILS_H
