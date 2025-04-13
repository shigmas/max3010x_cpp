#ifndef I2CBUS_H
#define I2CBUS_H

#include "driver/i2c_master.h"

#include <vector>

class Peripheral;

class I2CBus {
public:
    I2CBus(i2c_port_num_t port, gpio_num_t sda, gpio_num_t scl, i2c_clock_source_t clock);
    ~I2CBus();

    Peripheral * AddPeripheral(i2c_addr_bit_len_t addrLen, uint16_t address, uint32_t speed);
    i2c_master_bus_handle_t GetHandle() const {
        return _busHandle;
    }
    
private:
    i2c_master_bus_handle_t _busHandle;
    std::vector<Peripheral *> _peripherals;
};

#endif // I2CBUS_H
