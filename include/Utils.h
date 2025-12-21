/* =============================================================================
   BosMag LoRa Transceiver - Utility Functions
   ============================================================================= */

#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>
#include <stddef.h>

// Microsecond delay
void delay_us(uint32_t us);

// Hex dump for debugging
void log_hex_dump(const uint8_t *data, size_t len, const char *prefix);

#endif // UTILS_H