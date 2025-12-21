#ifndef USB_MANAGER_H
#define USB_MANAGER_H

#include <stddef.h>
#include <stdint.h>

// ============================================================================
// USB MANAGER CLASS
// ============================================================================
class USBManager {
public:
    static bool init();
    static int read(uint8_t *buffer, size_t size, uint32_t timeout_ms);
    static int write(const uint8_t *buffer, size_t size, uint32_t timeout_ms);
};

#endif // USB_MANAGER_H