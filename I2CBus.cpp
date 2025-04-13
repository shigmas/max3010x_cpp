#include "I2CBus.h"

#include <algorithm>

#include "esp_err.h"

#include "driver/i2c_master.h"
#include "Peripheral.h"

I2CBus::I2CBus(i2c_port_num_t port, gpio_num_t sda, gpio_num_t scl, i2c_clock_source_t clock) {

    // we get a warning when we don't set all of them, but there are defaults. And, if we
    // don't use them in this case, it will crash.
    i2c_master_bus_config_t bus_config = {
        .i2c_port = port,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .clk_source = clock,
        .glitch_ignore_cnt = 7,
        // .intr_priority = 0, // driver selects default
        // .trans_queue_depth = 4, // for async trans, transfers pending in background
        .flags = {
            .enable_internal_pullup = true,
            // .allow_pd = false,
        },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &_busHandle));
}

I2CBus::~I2CBus() {
    std::for_each(_peripherals.begin(), _peripherals.end(), [](Peripheral *p) {
        delete p;
    });
    ESP_ERROR_CHECK(i2c_del_master_bus(_busHandle));
}

Peripheral * I2CBus::AddPeripheral(i2c_addr_bit_len_t addrLen, uint16_t address,
                                   uint32_t speed) {
    Peripheral *p = new I2CPeripheral(*this, addrLen, address, speed);
    _peripherals.push_back(p);

    return p;
}    
