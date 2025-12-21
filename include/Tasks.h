#ifndef TASKS_H
#define TASKS_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ============================================================================
// TASK DECLARATIONS
// ============================================================================

// Serial RX task - reads from USB and transmits via LoRa
void serial_rx_task(void *pvParameters);

// Radio RX task - handles incoming LoRa packets
void radio_rx_task(void *pvParameters);

// Create and start both tasks
void tasks_init();

#endif // TASKS_H