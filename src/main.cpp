/* =============================================================================
   BosMag LoRa Transceiver Firmware
   ESP32-S3 + Ebyte E22-900M-30S (SX1262)
   
   Features:
   - Serial-to-LoRa transparent bridge
   - RadioLib RF switch control via setRfSwitchPins
   - LNA boost enabled for improved RX sensitivity
   - Interrupt-driven RX with DIO1
   - Safe RF state management to prevent hardware damage
   ============================================================================= */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "hal/gpio_hal.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_chip_info.h"
#include "esp_timer.h"

#include "Config.h"

// RadioLib includes
#include <RadioLib.h>
#include <EspHal.h> // Include the header file for EspHal

// ============================================================================
// CONSTANTS & GLOBALS
// ============================================================================

static const char *TAG = "BOSMAG";

// Global state variables
static volatile radio_state_t g_radio_state = RADIO_STATE_IDLE;
static volatile bool g_rx_data_available = false;
static volatile bool g_tx_done = false;
static uint8_t g_rx_buffer[LORA_RX_PAYLOAD_SIZE];
static size_t g_rx_length = 0;
static SemaphoreHandle_t g_radio_mutex = NULL;

// RadioLib HAL, module and radio
static Module* g_mod = nullptr;
static SX1262* g_radio = nullptr;

// SPI handle for direct register access
static spi_device_handle_t g_spi_handle = NULL;

// Helper: Microsecond delay using ESP timer
static inline void delay_us(uint32_t us)
{
    uint32_t start = (uint32_t)esp_timer_get_time();
    while ((uint32_t)esp_timer_get_time() - start < us) {
        // Spin wait
    }
}

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

static void radio_init();
static void radio_start_rx();
static void radio_transmit(const uint8_t *data, size_t length);
static void gpio_setup();
static void spi_setup();
static void uart_init();
static void IRAM_ATTR setFlag_RxDone();
static void IRAM_ATTR setFlag_TxDone();
static void serial_rx_task(void *pvParameters);
static void radio_rx_task(void *pvParameters);
static void write_lna_boost_register();

// ============================================================================
// LOGGING & DEBUG UTILITIES
// ============================================================================

static void log_hex_dump(const uint8_t *data, size_t len, const char *prefix)
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

// ============================================================================
// RADIOLIB INTERRUPT CALLBACKS
// ============================================================================

static void IRAM_ATTR setFlag_RxDone()
{
    g_rx_data_available = true;
}

static void IRAM_ATTR setFlag_TxDone()
{
    g_tx_done = true;
}

// ============================================================================
// GPIO SETUP
// ============================================================================

static void gpio_setup()
{
    ESP_LOGI(TAG, "Configuring GPIO pins...");

    // Configure L_NSS as output (SPI chip select)
    gpio_config_t nss_config = {
        .pin_bit_mask = (1ULL << L_NSS),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&nss_config);
    gpio_set_level((gpio_num_t)L_NSS, 1);  // SPI CS idle HIGH

    // Configure L_RST as output (reset pin)
    gpio_config_t rst_config = {
        .pin_bit_mask = (1ULL << L_RST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&rst_config);
    gpio_set_level((gpio_num_t)L_RST, 1);  // Reset inactive HIGH

    // Configure L_BUSY as input (read-only)
    gpio_config_t busy_config = {
        .pin_bit_mask = (1ULL << L_BUSY),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&busy_config);

    // Configure L_DIO1 as input with interrupt (RX/TX done)
    gpio_config_t dio1_config = {
        .pin_bit_mask = (1ULL << L_DIO1),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,  // Rising edge (positive edge)
    };
    gpio_config(&dio1_config);

    // Configure L_RXEN as output (RF switch - RX path control)
    // RadioLib will manage this via setRfSwitchPins
    gpio_config_t rxen_config = {
        .pin_bit_mask = (1ULL << L_RXEN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&rxen_config);
    gpio_set_level((gpio_num_t)L_RXEN, 0);  // Start with LNA disabled (safe state)

    ESP_LOGI(TAG, "GPIO configuration complete");
}

// ============================================================================
// SPI SETUP
// ============================================================================

static void spi_setup()
{
    ESP_LOGI(TAG, "SPI will be initialized by RadioLib...");
    
    // RadioLib will handle SPI initialization internally
    // We just need to keep our SPI handle for direct register access
    
    spi_bus_config_t bus_config = {
        .mosi_io_num = L_MOSI,
        .miso_io_num = L_MISO,
        .sclk_io_num = L_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
        .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_GPIO_PINS,
    };

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return;
    }

    spi_device_interface_config_t device_config = {
        .mode = 0,  // SPI mode 0 (CPOL=0, CPHA=0)
        .clock_speed_hz = SPI_FREQUENCY,
        .spics_io_num = -1,  // We'll handle CS manually
        .queue_size = 7,
        .pre_cb = NULL,
        .post_cb = NULL,
    };

    ret = spi_bus_add_device(SPI2_HOST, &device_config, &g_spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "SPI bus ready for direct register access");
}

// ============================================================================
// UART SETUP
// ============================================================================

static void uart_init()
{
    uart_config_t uart_config = {
        .baud_rate = UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    const uart_port_t uart_port = UART_NUM_0;
    
    uart_driver_install(uart_port, SERIAL_RX_BUFFER_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(uart_port, &uart_config);
    uart_set_pin(uart_port, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    ESP_LOGI(TAG, "UART initialized (115200 baud)");
}

// ============================================================================
// LNA BOOST REGISTER WRITE (Direct SPI Access)
// ============================================================================

static void write_lna_boost_register()
{
    // SX1262 command: WriteRegister (0x0D)
    // Register address: 0x08AC (LNA boost)
    // Value: 0x96 (enable boost)
    
    uint8_t cmd_buffer[5] = {
        0x0D,           // WriteRegister command
        0x08, 0xAC,     // Register address (0x08AC)
        0x96,           // Value to write
    };

    // Wait for radio to be ready (BUSY pin low)
    uint32_t timeout = 1000;
    while (gpio_get_level((gpio_num_t)L_BUSY) && timeout--) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (timeout == 0) {
        ESP_LOGE(TAG, "Timeout waiting for BUSY pin before LNA boost write");
        return;
    }

    // Perform SPI transaction
    gpio_set_level((gpio_num_t)L_NSS, 0);  // CS low
    delay_us(10);

    spi_transaction_t trans = {
        .length = 32,  // 4 bytes * 8 bits
        .tx_buffer = cmd_buffer,
        .rx_buffer = NULL,
    };

    esp_err_t ret = spi_device_transmit(g_spi_handle, &trans);
    
    delay_us(10);
    gpio_set_level((gpio_num_t)L_NSS, 1);  // CS high

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "LNA boost enabled (register 0x08AC = 0x96)");
    } else {
        ESP_LOGE(TAG, "Failed to write LNA boost register: %s", esp_err_to_name(ret));
    }

    // Wait for operation to complete
    timeout = 1000;
    while (gpio_get_level((gpio_num_t)L_BUSY) && timeout--) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ============================================================================
// RADIO INITIALIZATION WITH RADIOLIB
// ============================================================================

static void radio_init()
{
    ESP_LOGI(TAG, "Initializing SX1262 radio module with RadioLib...");

    // Create HAL instance for ESP-IDF
    EspHal* g_hal = new EspHal(L_SCK, L_MISO, L_MOSI);
    
    // Create RadioLib module instance with HAL and pin configuration
    g_mod = new Module(g_hal, L_NSS, L_DIO1, L_RST, L_BUSY);
    
    // Create SX1262 radio instance
    g_radio = new SX1262(g_mod);

    // Perform hardware reset
    g_radio->reset();
    vTaskDelay(pdMS_TO_TICKS(50));

    // Begin initialization
    ESP_LOGI(TAG, "Configuring SX1262...");
    
    int16_t state = g_radio->begin(
        LORA_FREQUENCY,           // Frequency in MHz
        LORA_BANDWIDTH,           // Bandwidth in kHz
        LORA_SPREADING_FACTOR,    // Spreading factor
        LORA_CODING_RATE,         // Coding rate
        LORA_SYNC_WORD,           // Sync word (0x12 = private, 0x34 = public)
        LORA_TX_POWER,            // Output power in dBm
        LORA_PREAMBLE_LENGTH,     // Preamble length
        TCXO_VOLTAGE              // TCXO voltage (0 = no TCXO)
    );

    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "Radio initialization failed, code: %d", state);
        return;
    }

    ESP_LOGI(TAG, "SX1262 initialized successfully");

    // Enable DIO2 as RF switch for TX path (FEM control)
    state = g_radio->setDio2AsRfSwitch(true);
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "Failed to set DIO2 as RF switch, code: %d", state);
    } else {
        ESP_LOGI(TAG, "DIO2 configured as RF switch for TX");
    }

    // Configure RF switch pins: L_RXEN for RX path control (GPIO 13)
    // setRfSwitchPins(rxPin, txPin) - txPin unused (DIO2 handles TX), rxPin = L_RXEN
    g_radio->setRfSwitchPins(L_RXEN, RADIOLIB_NC);
    ESP_LOGI(TAG, "RF switch pins configured: RX_EN on GPIO %d", L_RXEN);

    // Enable LNA boost for improved RX sensitivity via direct SPI
    write_lna_boost_register();

    // Set DIO1 interrupt action callbacks
    g_radio->setDio1Action(setFlag_RxDone);

    // Configure CRC
    state = g_radio->setCRC(true);
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGW(TAG, "Failed to enable CRC, code: %d", state);
    }

    // Log final configuration
    ESP_LOGI(TAG, "Radio configuration:");
    ESP_LOGI(TAG, "  Frequency: %.3f MHz", LORA_FREQUENCY);
    ESP_LOGI(TAG, "  Bandwidth: %.1f kHz", LORA_BANDWIDTH);
    ESP_LOGI(TAG, "  SF: %d, CR: 4/%d", LORA_SPREADING_FACTOR, LORA_CODING_RATE);
    ESP_LOGI(TAG, "  TX Power: %d dBm", LORA_TX_POWER);
    ESP_LOGI(TAG, "  Sync Word: 0x%02X", LORA_SYNC_WORD);
    ESP_LOGI(TAG, "  Preamble: %d symbols", LORA_PREAMBLE_LENGTH);
    ESP_LOGI(TAG, "  DIO2 RF Switch: ENABLED (TX)");
    ESP_LOGI(TAG, "  RF Switch Control: ENABLED (RX via GPIO %d)", L_RXEN);
    ESP_LOGI(TAG, "  LNA Boost: ENABLED");
    ESP_LOGI(TAG, "  CRC: ENABLED");
}

// ============================================================================
// RADIO RX START
// ============================================================================

static void radio_start_rx()
{
    if (g_radio == nullptr) {
        ESP_LOGE(TAG, "Radio not initialized");
        return;
    }

    if (xSemaphoreTake(g_radio_mutex, pdMS_TO_TICKS(1000)) == pdFALSE) {
        ESP_LOGE(TAG, "Radio mutex timeout in start_rx");
        return;
    }

    // Start continuous RX mode
    // RadioLib's setRfSwitchPins automatically controls GPIO 13 during RX
    int16_t state = g_radio->startReceive();
    
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "Failed to start RX, code: %d", state);
        xSemaphoreGive(g_radio_mutex);
        return;
    }

    xSemaphoreGive(g_radio_mutex);
    ESP_LOGI(TAG, "Started listening for LoRa packets...");
}

// ============================================================================
// RADIO TRANSMIT
// ============================================================================

static void radio_transmit(const uint8_t *data, size_t length)
{
    if (g_radio == nullptr) {
        ESP_LOGE(TAG, "Radio not initialized");
        return;
    }

    if (length > LORA_TX_PAYLOAD_SIZE) {
        ESP_LOGE(TAG, "Payload too large: %zu > %d", length, LORA_TX_PAYLOAD_SIZE);
        return;
    }

    if (xSemaphoreTake(g_radio_mutex, pdMS_TO_TICKS(1000)) == pdFALSE) {
        ESP_LOGE(TAG, "Radio mutex timeout in transmit");
        return;
    }

    ESP_LOGI(TAG, "Transmitting %zu bytes via LoRa...", length);
    log_hex_dump(data, length, "TX Data");

    // Set TX done callback
    g_tx_done = false;
    g_radio->setDio1Action(setFlag_TxDone);

    // Transmit the data (cast away const since RadioLib doesn't use const)
    // RadioLib's setRfSwitchPins automatically controls GPIO 13 and DIO2 during TX
    int16_t state = g_radio->startTransmit((uint8_t*)data, length);
    
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "Failed to start TX, code: %d", state);
        xSemaphoreGive(g_radio_mutex);
        return;
    }

    // Wait for TX done with timeout
    uint32_t timeout_ms = 5000;
    uint32_t start_time = xTaskGetTickCount();
    
    while (!g_tx_done) {
        if ((xTaskGetTickCount() - start_time) > pdMS_TO_TICKS(timeout_ms)) {
            ESP_LOGE(TAG, "TX timeout!");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (g_tx_done) {
        ESP_LOGI(TAG, "TX complete");
    }

    // Restore RX callback and return to RX mode
    g_radio->setDio1Action(setFlag_RxDone);
    
    int16_t rx_state = g_radio->startReceive();
    if (rx_state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "Failed to restart RX after TX, code: %d", rx_state);
    }

    xSemaphoreGive(g_radio_mutex);
}

// ============================================================================
// SERIAL RX TASK
// ============================================================================

static void serial_rx_task(void *pvParameters)
{
    uint8_t rx_buffer[SERIAL_RX_BUFFER_SIZE];
    const uart_port_t uart_port = UART_NUM_0;
    
    while (1) {
        // Read from UART
        int length = uart_read_bytes(uart_port, rx_buffer, SERIAL_RX_BUFFER_SIZE, 
                                     pdMS_TO_TICKS(SERIAL_RX_TIMEOUT_MS));
        
        if (length > 0) {
            // Data received from Serial - transmit via LoRa
            ESP_LOGI(TAG, "Serial RX: %d bytes", length);
            radio_transmit(rx_buffer, length);
        }
    }
}

// ============================================================================
// RADIO RX TASK (Event Handler)
// ============================================================================

static void radio_rx_task(void *pvParameters)
{
    const uart_port_t uart_port = UART_NUM_0;
    
    while (1) {
        // Wait for DIO1 interrupt indicating RX data available
        if (g_rx_data_available) {
            g_rx_data_available = false;

            if (xSemaphoreTake(g_radio_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
                // Read received data from radio
                int16_t state = g_radio->readData(g_rx_buffer, LORA_RX_PAYLOAD_SIZE);
                
                if (state >= 0) {
                    // state contains the number of bytes received
                    g_rx_length = state;
                    
                    // Get RSSI and SNR
                    float rssi = g_radio->getRSSI();
                    float snr = g_radio->getSNR();
                    
                    ESP_LOGI(TAG, "LoRa RX: %zu bytes | RSSI: %.1f dBm | SNR: %.1f dB", 
                             g_rx_length, rssi, snr);
                    log_hex_dump(g_rx_buffer, g_rx_length, "RX Data");
                    
                    // Echo to UART
                    uart_write_bytes(uart_port, (const char *)g_rx_buffer, g_rx_length);
                    uart_write_bytes(uart_port, "\r\n", 2);
                    
                } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
                    ESP_LOGW(TAG, "RX CRC error");
                } else {
                    ESP_LOGE(TAG, "RX failed, code: %d", state);
                }

                // Return to RX mode
                g_radio->startReceive();
                xSemaphoreGive(g_radio_mutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ============================================================================
// MAIN APPLICATION
// ============================================================================

extern "C" void app_main()
{
    // Print startup banner
    printf("\n");
    printf("╔════════════════════════════════════════════════════════════╗\n");
    printf("║                  BosMag LoRa Transceiver                   ║\n");
    printf("║            ESP32-S3 + Ebyte E22-900M-30S (SX1262)          ║\n");
    printf("║                    with RadioLib                           ║\n");
    printf("╚════════════════════════════════════════════════════════════╝\n");
    printf("\n");

    ESP_LOGI(TAG, "Firmware Version: 1.0.0");
    ESP_LOGI(TAG, "Board: %s", BOARD_NAME);
    ESP_LOGI(TAG, "MCU: %s", MCU_VARIANT);

    // Create synchronization primitives
    g_radio_mutex = xSemaphoreCreateMutex();
    
    if (g_radio_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create radio mutex");
        return;
    }

    // Initialize peripherals
    gpio_setup();
    spi_setup();
    uart_init();
    radio_init();

    // Start RX mode
    radio_start_rx();

    // Create tasks
    xTaskCreate(serial_rx_task, "serial_rx", 4096, NULL, 5, NULL);
    xTaskCreate(radio_rx_task, "radio_rx", 4096, NULL, 6, NULL);

    ESP_LOGI(TAG, "=== BosMag Ready ===");
    ESP_LOGI(TAG, "Serial-to-LoRa bridge active");
    ESP_LOGI(TAG, "Waiting for data...");

    // Main task enters idle state (FreeRTOS handles everything)
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));

    }
}