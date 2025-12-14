#ifndef ESP_HAL_H
#define ESP_HAL_H

#include <RadioLib.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_timer.h"
#include "esp_log.h"

#define RADIOLIB_NC   (0xFFFFFFFF)

class EspHal : public RadioLibHal {
public:
    EspHal(int8_t sck, int8_t miso, int8_t mosi)
        : RadioLibHal(GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, 0, 1, GPIO_INTR_POSEDGE, GPIO_INTR_NEGEDGE),
          _sck(sck), _miso(miso), _mosi(mosi) {
    }

    void init() override {
        spiBegin();
    }

    void term() override {
        spiEnd();
    }

    void pinMode(uint32_t pin, uint32_t mode) override {
        if (pin == RADIOLIB_NC) return;
        
        gpio_config_t conf = {};
        conf.pin_bit_mask = (1ULL << pin);
        conf.mode = (gpio_mode_t)mode;
        conf.pull_up_en = GPIO_PULLUP_DISABLE;
        conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        conf.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&conf);
    }

    void digitalWrite(uint32_t pin, uint32_t value) override {
        if (pin != RADIOLIB_NC) gpio_set_level((gpio_num_t)pin, value);
    }

    uint32_t digitalRead(uint32_t pin) override {
        if (pin != RADIOLIB_NC) return gpio_get_level((gpio_num_t)pin);
        return 0;
    }

    void attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void), uint32_t mode) override {
        if (interruptNum == RADIOLIB_NC) return;
        gpio_install_isr_service(0);
        gpio_set_intr_type((gpio_num_t)interruptNum, (gpio_int_type_t)(mode & 0x7));
        gpio_isr_handler_add((gpio_num_t)interruptNum, (void (*)(void*))interruptCb, NULL);
    }

    void detachInterrupt(uint32_t interruptNum) override {
        if (interruptNum != RADIOLIB_NC) gpio_isr_handler_remove((gpio_num_t)interruptNum);
    }

    void delay(unsigned long ms) override {
        vTaskDelay(pdMS_TO_TICKS(ms));
    }

    void delayMicroseconds(unsigned long us) override {
        int64_t start = esp_timer_get_time();
        while ((esp_timer_get_time() - start) < us) {
            asm volatile ("nop");
        }
    }

    unsigned long millis() override {
        return (unsigned long)(esp_timer_get_time() / 1000ULL);
    }

    unsigned long micros() override {
        return (unsigned long)esp_timer_get_time();
    }

    long pulseIn(uint32_t pin, uint32_t state, unsigned long timeout) override {
        if (pin == RADIOLIB_NC) return 0;
        uint32_t start = micros();
        while (digitalRead(pin) != state) {
            if (micros() - start > timeout) return 0;
        }
        uint32_t pulseStart = micros();
        while (digitalRead(pin) == state) {
            if (micros() - start > timeout) return 0;
        }
        return micros() - pulseStart;
    }

    void spiBegin() {
        // Initialize SPI Bus
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = _mosi;
        buscfg.miso_io_num = _miso;
        buscfg.sclk_io_num = _sck;
        buscfg.quadwp_io_num = -1;
        buscfg.quadhd_io_num = -1;
        buscfg.max_transfer_sz = 4096;

        // Use SPI2_HOST (FSPI) for S3
        spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);

        // Add Device
        spi_device_interface_config_t devcfg = {};
        devcfg.clock_speed_hz = 2 * 1000 * 1000; // 2 MHz
        devcfg.mode = 0;
        devcfg.spics_io_num = -1; // CS handled manually by RadioLib
        devcfg.queue_size = 7;
        
        spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    }

    void spiEnd() {
        if (spi) spi_bus_remove_device(spi);
        spi_bus_free(SPI2_HOST);
    }

    void spiBeginTransaction() {} // Not needed for ESP-IDF
    void spiEndTransaction() {}   // Not needed for ESP-IDF

    void spiTransfer(uint8_t* out, size_t len, uint8_t* in) override {
        spi_transaction_t t = {};
        t.length = 8 * len;
        t.tx_buffer = out;
        t.rx_buffer = in;
        // Use polling for short transactions (lower latency)
        spi_device_polling_transmit(spi, &t);
    }

private:
    int8_t _sck, _miso, _mosi;
    spi_device_handle_t spi = NULL;
};

#endif