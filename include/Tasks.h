/* =============================================================================
   BosMag LoRa Transceiver - Task Definitions
   ============================================================================= */

#ifndef TASKS_H
#define TASKS_H

// Serial RX Task - reads from USB and transmits via LoRa
void serial_rx_task(void *pvParameters);

// Radio RX Task - handles incoming LoRa packets
void radio_rx_task(void *pvParameters);

#endif // TASKS_H