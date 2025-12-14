#include "HardwareInit.h"
#include "Config.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_log.h"

static const char* TAG = "HardwareInit";
static spi_device_handle_t g_spi_handle = NULL;

namespace HardwareInit {

void initGPIO()
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
        .intr_type = GPIO_INTR_POSEDGE,  // Rising edge
    };
    gpio_config(&dio1_config);

    // Configure L_RXEN as output (RF switch - RX path control)
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

void initSPI()
{
    ESP_LOGI(TAG, "Initializing SPI bus...");
    
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

    ESP_LOGI(TAG, "SPI bus initialized successfully");
}

void initUART()
{
    ESP_LOGI(TAG, "Initializing UART...");
    
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

void initAll()
{
    ESP_LOGI(TAG, "Initializing all hardware...");
    initGPIO();
    initSPI();
    initUART();
    ESP_LOGI(TAG, "All hardware initialized");
}

spi_device_handle_t getSPIHandle()
{
    return g_spi_handle;
}

} // namespace HardwareInit
