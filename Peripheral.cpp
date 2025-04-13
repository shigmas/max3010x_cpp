#include "Peripheral.h"

#include "I2CBus.h"

I2CPeripheral::I2CPeripheral(I2CBus& bus, i2c_addr_bit_len_t addrLen, uint16_t address, uint32_t speed) {
    i2c_device_config_t dev_config = {
        .dev_addr_length = addrLen,
        .device_address = address,
        .scl_speed_hz = speed,
    };
    i2c_master_bus_handle_t busHandle = bus.GetHandle();
    i2c_master_dev_handle_t handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(busHandle, &dev_config, &handle));
    _handle = handle;
}
