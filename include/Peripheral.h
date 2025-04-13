#ifndef PERIPHERAL_H
#define PERIPHERAL_H

#include "esp_err.h"

#include "driver/i2c_master.h"

#include "freertos/FreeRTOS.h"

#define I2C_MASTER_TIMEOUT_MS       1000

class I2CBus;

// Peripheral interface for I2C and SPI.
class Peripheral {
public:
    virtual ~Peripheral() {}
    virtual esp_err_t ReadRegister(uint8_t reg_addr, uint8_t *data, size_t len) = 0;
    virtual esp_err_t WriteByteToRegister(uint8_t reg_addr, uint8_t data) = 0;
};

class I2CPeripheral : public Peripheral {
public:
    I2CPeripheral(I2CBus& bus, i2c_addr_bit_len_t addrLen, uint16_t address, uint32_t speed);

    virtual ~I2CPeripheral() {
        ESP_ERROR_CHECK(i2c_master_bus_rm_device(_handle));
    }

    virtual esp_err_t ReadRegister(uint8_t reg_addr, uint8_t *data, size_t len) override {
        return i2c_master_transmit_receive(_handle, &reg_addr, 1, data, len,
                                           I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    }

    virtual esp_err_t WriteByteToRegister(uint8_t reg_addr, uint8_t data) override {
        uint8_t write_buf[2] = {reg_addr, data};
        return i2c_master_transmit(_handle, write_buf, sizeof(write_buf),
                                   I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    }

private:
    i2c_master_dev_handle_t _handle;
};

#endif // PERIPHERAL_H
