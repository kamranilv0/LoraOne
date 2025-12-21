/* =============================================================================
   BosMag LoRa Transceiver - Radio Manager
   ============================================================================= */

#ifndef RADIO_MANAGER_H
#define RADIO_MANAGER_H

#include <stdint.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "Config.h"

// Forward declaration
class SX1262;

// Global state flags
extern volatile bool g_rx_data_available;
extern volatile bool g_tx_done;

// RX buffer
extern uint8_t g_rx_buffer[LORA_RX_PAYLOAD_SIZE];
extern size_t g_rx_length;

// Radio mutex
extern SemaphoreHandle_t g_radio_mutex;

// Radio instance (for Tasks.cpp access)
extern SX1262* g_radio;

// Initialize SPI bus
void spi_setup();

// Initialize radio module
void radio_init();

// Start RX mode
void radio_start_rx();

// Transmit data
void radio_transmit(const uint8_t *data, size_t length);

// Interrupt callbacks
void IRAM_ATTR setFlag_RxDone();
void IRAM_ATTR setFlag_TxDone();

#endif // RADIO_MANAGER_H