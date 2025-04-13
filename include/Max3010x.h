#ifndef MAX3010x_H
#define MAX3010x_H

#include "sdkconfig.h"

#include "driver/i2c_master.h"

// While this is an I2C device, we abtract that, with some leakage (the I2C bus).
class I2CBus;
class Peripheral;

#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL       /*!< GPIO number
 used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA       /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
// 100000 from the arduino library
#define I2C_MASTER_FREQ_HZ          CONFIG_I2C_MASTER_FREQUENCY /*!< I2C master clock frequency */

#define MAX3010X_SENSOR_ADDR         0x57        /*!< Address of the MAX3010X sensor */
#define MAX3010X_WHO_AM_I_REG_ADDR   0x75        /*!< Register addresses of the "who am I" register */
#define MAX3010X_PWR_MGMT_1_REG_ADDR 0x6B        /*!< Register addresses of the power management register */
#define MAX3010X_RESET_BIT           7

class Max3010x {
public:
    Max3010x();
    virtual ~Max3010x();

    bool VerifyDevice();

    // called by SetUpDevice, so usually no need to call explicitly
    bool SoftReset();

    // After constructor, call this.
    void SetUpDevice(uint8_t powerLevel, uint8_t sampleAverage, uint8_t ledMode,
                     int sampleRate, int pulseWidth, int adcRange);

    float ReadTemperature();

private:
    // functions that we may expose, but I think they're to too low level
    void _SetFIFOAverage(uint8_t numberOfSamples);
    void _EnableFIFORollover();
    void _SetLEDMode(uint8_t mode);
    void _SetADCRange(uint8_t adcRange);
    void _SetSampleRate(uint8_t adcRange);
    void _SetPulseWidth(uint8_t pulseWidth);
    void _SetPulseAmplitudeRed(uint8_t amplitude);
    void _SetPulseAmplitudeIR(uint8_t amplitude);
    void _SetPulseAmplitudeGreen(uint8_t amplitude);
    void _SetPulseAmplitudeProximity(uint8_t amplitude);
    //Given a slot number assign a thing to it
    //Devices are SLOT_RED_LED or SLOT_RED_PILOT (proximity)
    //Assigning a SLOT_RED_LED will pulse LED
    //Assigning a SLOT_RED_PILOT will ??
    void _EnableSlot(uint8_t slotNumber, uint8_t device);
    //Resets all points to start in a known state
    //Page 15 recommends clearing FIFO before beginning a read
    void _ClearFIFO();

    void _Bitmask(uint8_t reg, uint8_t mask, uint8_t bit);

private:
    I2CBus *_i2c;
    Peripheral* _p;
    uint8_t _activeLEDs;
};

#endif // MAX3010x_H
