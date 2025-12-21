#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

/* =========================================================================
   BOARD IDENTIFICATION
   ========================================================================= */
#define BOARD_NAME "BosMag"
#define MCU_VARIANT "ESP32-S3-MINI-1-N4R2"
#define LORA_MODULE "Ebyte E22-900M-30S (SX1262)"

#define CONFIG_IDF_TARGET_ESP32 1

/* =========================================================================
   SPI INTERFACE PINS (L-Series LoRa Module)
   ========================================================================= */
#define L_MOSI 18  // SPI Master Out Slave In
#define L_MISO 17  // SPI Master In Slave Out
#define L_SCK  21  // SPI Serial Clock
#define L_NSS  47  // SPI Chip Select (Active Low)

/* =========================================================================
   LoRa CONTROL PINS
   ========================================================================= */
#define L_RST  16  // LoRa Module Reset (Active Low)
#define L_BUSY 15  // LoRa Busy Signal (must check before SPI ops)
#define L_DIO1 14  // Interrupt Pin: TxDone, RxDone, Timeout

#define TCXO_VOLTAGE 1.8f  // TCXO Voltage (if applicable)

/* =========================================================================
   RF SWITCH CONTROL PINS - CRITICAL FOR SAFETY
   =========================================================================
   
   HYBRID RF SWITCHING MECHANISM:
   1. TX Path: Controlled by SX1262 DIO2 pin directly (TX_EN of FEM)
      - RadioLib handles this via setDio2AsRfSwitch(true)
      - No MCU GPIO control needed
   
   2. RX Path: Controlled by MCU GPIO 13 (RX_EN of FEM/LNA)
      - GPIO 13 HIGH:  RX Mode (LNA enabled)
      - GPIO 13 LOW:   TX Mode (LNA disabled - CRITICAL!)
   
   ========================================================================= */
#define L_RXEN 13  // RX Enable GPIO - Controls LNA. Must be LOW during TX!

/* =========================================================================
   LoRa RADIO PARAMETERS
   ========================================================================= */

// Frequency Configuration
#define LORA_FREQUENCY 869.525f        // MHz - ISM Band 868-870 MHz (Europe)

// Modulation Parameters
#define LORA_BANDWIDTH 250.0f        // kHz
#define LORA_SPREADING_FACTOR 8      // SF7: Balance between range and data rate
#define LORA_CODING_RATE 5           // CR 4/5

// Power Configuration
// #define LORA_TX_POWER 21             // dBm (Module supports 30dBm, limited to 21 for lab)
#define LORA_TX_POWER 10             // dBm (Module supports 30dBm, limited to 21 for lab)


// Preamble and Payload
#define LORA_PREAMBLE_LENGTH 8      // Standard LoRa preamble length
#define LORA_PAYLOAD_LENGTH 255      // Maximum payload

#define LORA_USE_DCDC true          // Use DC-DC regulator for power efficiency
// CRC and Header
#define LORA_CRC_ENABLED true
#define LORA_EXPLICIT_HEADER true

#define LORA_SYNC_WORD 0x12          // 0x12 = Private Network, 0x34 = Public Network

/* =========================================================================
   SX1262 SPECIFIC CONFIGURATIONS
   ========================================================================= */

// LNA Boost Register Configuration (Critical for RX sensitivity)
#define SX1262_REG_RX_GAIN 0x08AC     // Register address for RX gain
#define SX1262_RX_BOOSTED_GAIN 0x96   // Value for boosted gain mode

// TX Ramp Time
#define LORA_TX_RAMP_TIME 2           // 0-15, affects TX envelope

// DIO2 as RF Switch (MANDATORY for this design)
#define DIO2_AS_RF_SWITCH true

/* =========================================================================
   UART / SERIAL CONFIGURATION
   ========================================================================= */
#define UART_BAUDRATE 115200
#define UART_TX_PIN 43               // ESP32-S3 UART0 TX
#define UART_RX_PIN 44               // ESP32-S3 UART0 RX
// Note: Use UART_NUM_0 enum in code, not 0 directly

/* =========================================================================
   TIMING CONSTANTS
   ========================================================================= */
#define LORA_RX_TIMEOUT_MS 5000      // RX timeout before returning to idle
#define SERIAL_RX_TIMEOUT_MS 100     // Serial data timeout
#define RF_SWITCH_DELAY_US 10        // GPIO settling time
#define SPI_FREQUENCY 5000000        // 5 MHz SPI clock

/* =========================================================================
   SAFETY & DEBUG CONSTANTS
   ========================================================================= */
#define ENABLE_DEBUG_LOGS 1
#define LOG_RF_SWITCH_STATES 1       // Log RF switch state transitions
#define WATCHDOG_TIMEOUT_MS 10000    // Watchdog timeout

/* =========================================================================
   BUFFER SIZES
   ========================================================================= */
#define SERIAL_RX_BUFFER_SIZE 256
#define LORA_RX_PAYLOAD_SIZE 255
#define LORA_TX_PAYLOAD_SIZE 255

/* =========================================================================
   ENUM DEFINITIONS
   ========================================================================= */

typedef enum {
    RF_STATE_RX = 0,      // RX enabled, LNA active (GPIO 13 = HIGH)
    RF_STATE_TX = 1,      // TX enabled, LNA disabled (GPIO 13 = LOW)
    RF_STATE_IDLE = 2,    // Idle, both disabled (GPIO 13 = LOW)
    RF_STATE_ERROR = 3    // Error state
} rf_state_t;

typedef enum {
    RADIO_STATE_IDLE = 0,
    RADIO_STATE_RX = 1,
    RADIO_STATE_TX = 2,
    RADIO_STATE_ERROR = 3
} radio_state_t;

#endif // CONFIG_H