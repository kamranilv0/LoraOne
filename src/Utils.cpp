/* =============================================================================
   BosMag LoRa Transceiver - Utility Functions Implementation
   ============================================================================= */

#include "Utils.h"
#include "Config.h"
#include "esp_timer.h"
#include <stdio.h>

void delay_us(uint32_t us)
{
    uint32_t start = (uint32_t)esp_timer_get_time();
    while ((uint32_t)esp_timer_get_time() - start < us) {
        // Spin wait
    }
}

void log_hex_dump(const uint8_t *data, size_t len, const char *prefix)
{
#if ENABLE_DEBUG_LOGS
    printf("%s: ", prefix);
    for (size_t i = 0; i < len; i++) {
        printf("%02X ", data[i]);
    }
    printf("\n");
#else
    (void)data; (void)len; (void)prefix;
#endif
}