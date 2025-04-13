#include "Max3010x.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <esp_log.h>
#include <esp_timer.h>

#include "I2CBus.h"
#include "Peripheral.h"
#include "include/Max3010x.h"

#define TAG "MAX3010X"

// stolen verbatim from
// https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library/blob/master/src/MAX30105.cpp
// Status Registers
static const uint8_t MAX30105_INTSTAT1 =		0x00;
static const uint8_t MAX30105_INTSTAT2 =		0x01;
static const uint8_t MAX30105_INTENABLE1 =		0x02;
static const uint8_t MAX30105_INTENABLE2 =		0x03;

// FIFO Registers
static const uint8_t MAX30105_FIFOWRITEPTR = 	0x04;
static const uint8_t MAX30105_FIFOOVERFLOW = 	0x05;
static const uint8_t MAX30105_FIFOREADPTR = 	0x06;
static const uint8_t MAX30105_FIFODATA =		0x07;

// Configuration Registers
static const uint8_t MAX30105_FIFOCONFIG = 		0x08;
static const uint8_t MAX30105_MODECONFIG = 		0x09;
static const uint8_t MAX30105_PARTICLECONFIG = 	0x0A;    // Note, sometimes listed as "SPO2" config in datasheet (pg. 11)
static const uint8_t MAX30105_LED1_PULSEAMP = 	0x0C;
static const uint8_t MAX30105_LED2_PULSEAMP = 	0x0D;
static const uint8_t MAX30105_LED3_PULSEAMP = 	0x0E;
static const uint8_t MAX30105_LED_PROX_AMP = 	0x10;
static const uint8_t MAX30105_MULTILEDCONFIG1 = 0x11;
static const uint8_t MAX30105_MULTILEDCONFIG2 = 0x12;

// Die Temperature Registers
static const uint8_t MAX30105_DIETEMPINT = 		0x1F;
static const uint8_t MAX30105_DIETEMPFRAC = 	0x20;
static const uint8_t MAX30105_DIETEMPCONFIG = 	0x21;

// Proximity Function Registers
static const uint8_t MAX30105_PROXINTTHRESH = 	0x30;

// Part ID Registers
static const uint8_t MAX30105_REVISIONID = 		0xFE;
static const uint8_t MAX30105_PARTID = 			0xFF;    // Should always be 0x15. Identical to MAX30102.

// MAX30105 Commands
// Interrupt configuration (pg 13, 14)
static const uint8_t MAX30105_INT_A_FULL_MASK =		(uint8_t)~0b10000000;
static const uint8_t MAX30105_INT_A_FULL_ENABLE = 	0x80;
static const uint8_t MAX30105_INT_A_FULL_DISABLE = 	0x00;

static const uint8_t MAX30105_INT_DATA_RDY_MASK = (uint8_t)~0b01000000;
static const uint8_t MAX30105_INT_DATA_RDY_ENABLE =	0x40;
static const uint8_t MAX30105_INT_DATA_RDY_DISABLE = 0x00;

static const uint8_t MAX30105_INT_ALC_OVF_MASK = (uint8_t)~0b00100000;
static const uint8_t MAX30105_INT_ALC_OVF_ENABLE = 	0x20;
static const uint8_t MAX30105_INT_ALC_OVF_DISABLE = 0x00;

static const uint8_t MAX30105_INT_PROX_INT_MASK = (uint8_t)~0b00010000;
static const uint8_t MAX30105_INT_PROX_INT_ENABLE = 0x10;
static const uint8_t MAX30105_INT_PROX_INT_DISABLE = 0x00;

static const uint8_t MAX30105_INT_DIE_TEMP_RDY_MASK = (uint8_t)~0b00000010;
static const uint8_t MAX30105_INT_DIE_TEMP_RDY_ENABLE = 0x02;
static const uint8_t MAX30105_INT_DIE_TEMP_RDY_DISABLE = 0x00;

static const uint8_t MAX30105_SAMPLEAVG_MASK =	(uint8_t)~0b11100000;
static const uint8_t MAX30105_SAMPLEAVG_1 = 	0x00;
static const uint8_t MAX30105_SAMPLEAVG_2 = 	0x20;
static const uint8_t MAX30105_SAMPLEAVG_4 = 	0x40;
static const uint8_t MAX30105_SAMPLEAVG_8 = 	0x60;
static const uint8_t MAX30105_SAMPLEAVG_16 = 	0x80;
static const uint8_t MAX30105_SAMPLEAVG_32 = 	0xA0;

static const uint8_t MAX30105_ROLLOVER_MASK = 	0xEF;
static const uint8_t MAX30105_ROLLOVER_ENABLE = 0x10;
static const uint8_t MAX30105_ROLLOVER_DISABLE = 0x00;

static const uint8_t MAX30105_A_FULL_MASK = 	0xF0;

// Mode configuration commands (page 19)
static const uint8_t MAX30105_SHUTDOWN_MASK = 	0x7F;
static const uint8_t MAX30105_SHUTDOWN = 		0x80;
static const uint8_t MAX30105_WAKEUP = 			0x00;

static const uint8_t MAX30105_RESET_MASK = 		0xBF;
static const uint8_t MAX30105_RESET = 			0x40;

static const uint8_t MAX30105_MODE_MASK = 		0xF8;
static const uint8_t MAX30105_MODE_REDONLY = 	0x02;
static const uint8_t MAX30105_MODE_REDIRONLY = 	0x03;
static const uint8_t MAX30105_MODE_MULTILED = 	0x07;

// Particle sensing configuration commands (pgs 19-20)
static const uint8_t MAX30105_ADCRANGE_MASK = 	0x9F;
static const uint8_t MAX30105_ADCRANGE_2048 = 	0x00;
static const uint8_t MAX30105_ADCRANGE_4096 = 	0x20;
static const uint8_t MAX30105_ADCRANGE_8192 = 	0x40;
static const uint8_t MAX30105_ADCRANGE_16384 = 	0x60;

static const uint8_t MAX30105_SAMPLERATE_MASK = 0xE3;
static const uint8_t MAX30105_SAMPLERATE_50 = 	0x00;
static const uint8_t MAX30105_SAMPLERATE_100 = 	0x04;
static const uint8_t MAX30105_SAMPLERATE_200 = 	0x08;
static const uint8_t MAX30105_SAMPLERATE_400 = 	0x0C;
static const uint8_t MAX30105_SAMPLERATE_800 = 	0x10;
static const uint8_t MAX30105_SAMPLERATE_1000 = 0x14;
static const uint8_t MAX30105_SAMPLERATE_1600 = 0x18;
static const uint8_t MAX30105_SAMPLERATE_3200 = 0x1C;

static const uint8_t MAX30105_PULSEWIDTH_MASK = 0xFC;
static const uint8_t MAX30105_PULSEWIDTH_69 = 	0x00;
static const uint8_t MAX30105_PULSEWIDTH_118 = 	0x01;
static const uint8_t MAX30105_PULSEWIDTH_215 = 	0x02;
static const uint8_t MAX30105_PULSEWIDTH_411 = 	0x03;

//Multi-LED Mode configuration (pg 22)
static const uint8_t MAX30105_SLOT1_MASK = 		0xF8;
static const uint8_t MAX30105_SLOT2_MASK = 		0x8F;
static const uint8_t MAX30105_SLOT3_MASK = 		0xF8;
static const uint8_t MAX30105_SLOT4_MASK = 		0x8F;

static const uint8_t SLOT_NONE = 				0x00;
static const uint8_t SLOT_RED_LED = 			0x01;
static const uint8_t SLOT_IR_LED = 				0x02;
static const uint8_t SLOT_GREEN_LED = 			0x03;
static const uint8_t SLOT_NONE_PILOT = 			0x04;
static const uint8_t SLOT_RED_PILOT =			0x05;
static const uint8_t SLOT_IR_PILOT = 			0x06;
static const uint8_t SLOT_GREEN_PILOT = 		0x07;

static const uint8_t MAX_30105_EXPECTEDPARTID = 0x15;

Max3010x::Max3010x() {
    // initialize the bus.
    _i2c = new I2CBus((i2c_port_num_t)I2C_MASTER_NUM, (gpio_num_t)I2C_MASTER_SDA_IO, (gpio_num_t)I2C_MASTER_SCL_IO, I2C_CLK_SRC_DEFAULT);

    _p = _i2c->AddPeripheral(I2C_ADDR_BIT_LEN_7, MAX3010X_SENSOR_ADDR,
                             I2C_MASTER_FREQ_HZ);
}
Max3010x::~Max3010x() { delete _i2c; }

bool Max3010x::VerifyDevice() {
    uint8_t partID;
    _p->ReadRegister(MAX30105_PARTID, &partID, 1);
    if (partID != MAX_30105_EXPECTEDPARTID)  {
        ESP_LOGI(TAG, "I2C MAX30105 did not match expected part ID");
        return false;
    }

    uint8_t revID;
    _p->ReadRegister(MAX30105_REVISIONID, &revID, 1);
    ESP_LOGI(TAG, "I2C MAX30105 rev ID: ", revID);

    return true;
}

bool Max3010x::SoftReset() {
    _Bitmask(MAX30105_MODECONFIG, MAX30105_RESET_MASK,
              MAX30105_RESET);

    return true;
}

void Max3010x::_SetFIFOAverage(uint8_t numberOfSamples) {
    _Bitmask(MAX30105_FIFOCONFIG, MAX30105_SAMPLEAVG_MASK, numberOfSamples);
}

void Max3010x::_EnableFIFORollover() {
    _Bitmask(MAX30105_FIFOCONFIG, MAX30105_ROLLOVER_MASK, MAX30105_ROLLOVER_ENABLE);
}

void Max3010x::_SetLEDMode(uint8_t mode) {
    // Set which LEDs are used for sampling -- Red only, RED+IR only, or custom.
    // See datasheet, page 19
    _Bitmask(MAX30105_MODECONFIG, MAX30105_MODE_MASK, mode);
}

void Max3010x::_SetADCRange(uint8_t adcRange) {
    // adcRange: one of MAX30105_ADCRANGE_2048, _4096, _8192, _16384
    _Bitmask(MAX30105_PARTICLECONFIG, MAX30105_ADCRANGE_MASK, adcRange);
}

void Max3010x::_SetSampleRate(uint8_t sampleRate) {
    // sampleRate: one of MAX30105_SAMPLERATE_50, _100, _200, _400, _800, _1000, _1600, _3200
    _Bitmask(MAX30105_PARTICLECONFIG, MAX30105_SAMPLERATE_MASK, sampleRate);
}

void Max3010x::_SetPulseWidth(uint8_t pulseWidth) {
    // pulseWidth: one of MAX30105_PULSEWIDTH_69, _188, _215, _411
    _Bitmask(MAX30105_PARTICLECONFIG, MAX30105_PULSEWIDTH_MASK, pulseWidth);
}

void Max3010x::_SetPulseAmplitudeRed(uint8_t amplitude) {
    _p->WriteByteToRegister(MAX30105_LED1_PULSEAMP, amplitude);
}

void Max3010x::_SetPulseAmplitudeIR(uint8_t amplitude) {
    _p->WriteByteToRegister(MAX30105_LED2_PULSEAMP, amplitude);

}
void Max3010x::_SetPulseAmplitudeGreen(uint8_t amplitude) {
    _p->WriteByteToRegister(MAX30105_LED3_PULSEAMP, amplitude);
}

void Max3010x::_SetPulseAmplitudeProximity(uint8_t amplitude) {
    _p->WriteByteToRegister(MAX30105_LED_PROX_AMP, amplitude);
}

void Max3010x::_EnableSlot(uint8_t slotNumber, uint8_t device) {

    switch (slotNumber) {
    case (1):
        _Bitmask(MAX30105_MULTILEDCONFIG1, MAX30105_SLOT1_MASK, device);
        break;
    case (2):
        _Bitmask(MAX30105_MULTILEDCONFIG1, MAX30105_SLOT2_MASK, device << 4);
        break;
    case (3):
        _Bitmask(MAX30105_MULTILEDCONFIG2, MAX30105_SLOT3_MASK, device);
        break;
    case (4):
        _Bitmask(MAX30105_MULTILEDCONFIG2, MAX30105_SLOT4_MASK, device << 4);
        break;
    default:
        //Shouldn't be here!
        break;
    }
}

void Max3010x::_ClearFIFO() {
    _p->WriteByteToRegister(MAX30105_FIFOWRITEPTR, 0);
    _p->WriteByteToRegister(MAX30105_FIFOOVERFLOW, 0);
    _p->WriteByteToRegister(MAX30105_FIFOREADPTR, 0);
}


void Max3010x::_Bitmask(uint8_t reg, uint8_t mask, uint8_t bit) {
    uint8_t origBit;
    _p->ReadRegister(reg, &origBit, 1);
    origBit = origBit & mask;
    _p->WriteByteToRegister(reg, origBit | bit);
}

void Max3010x::SetUpDevice(uint8_t powerLevel, uint8_t sampleAverage, uint8_t ledMode,
                           int sampleRate, int pulseWidth, int adcRange) {
    SoftReset();

    //FIFO Configuration
    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    //The chip will average multiple samples of same type together if you wish
    if (sampleAverage == 1) _SetFIFOAverage(MAX30105_SAMPLEAVG_1); //No averaging per FIFO record
    else if (sampleAverage == 2) _SetFIFOAverage(MAX30105_SAMPLEAVG_2);
    else if (sampleAverage == 4) _SetFIFOAverage(MAX30105_SAMPLEAVG_4);
    else if (sampleAverage == 8) _SetFIFOAverage(MAX30105_SAMPLEAVG_8);
    else if (sampleAverage == 16) _SetFIFOAverage(MAX30105_SAMPLEAVG_16
);
    else if (sampleAverage == 32) _SetFIFOAverage(MAX30105_SAMPLEAVG_32);
    else _SetFIFOAverage(MAX30105_SAMPLEAVG_4);

    //setFIFOAlmostFull(2); //Set to 30 samples to trigger an 'Almost Full' interrupt
    _EnableFIFORollover(); //Allow FIFO to wrap/roll over
    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

    //Mode Configuration
    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    if (ledMode == 3) _SetLEDMode(MAX30105_MODE_MULTILED); //Watch all three LED channels
    else if (ledMode == 2) _SetLEDMode(MAX30105_MODE_REDIRONLY); //Red and IR
    else _SetLEDMode(MAX30105_MODE_REDONLY); //Red only
    _activeLEDs = ledMode; //Used to control how many bytes to read from FIFO buffer

    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    //Particle Sensing Configuration
    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    if(adcRange < 4096) _SetADCRange(MAX30105_ADCRANGE_2048); //7.81pA per LSB
    else if(adcRange < 8192) _SetADCRange(MAX30105_ADCRANGE_4096); //15.63pA per LSB
    else if(adcRange < 16384) _SetADCRange(MAX30105_ADCRANGE_8192); //31.25pA per LSB
    else if(adcRange == 16384) _SetADCRange(MAX30105_ADCRANGE_16384); //62.5pA per LSB
    else _SetADCRange(MAX30105_ADCRANGE_2048);

    if (sampleRate < 100) _SetSampleRate(MAX30105_SAMPLERATE_50); //Take 50 samples per second
    else if (sampleRate < 200) _SetSampleRate(MAX30105_SAMPLERATE_100);
    else if (sampleRate < 400) _SetSampleRate(MAX30105_SAMPLERATE_200);
    else if (sampleRate < 800) _SetSampleRate(MAX30105_SAMPLERATE_400);
    else if (sampleRate < 1000) _SetSampleRate(MAX30105_SAMPLERATE_800);
    else if (sampleRate < 1600) _SetSampleRate(MAX30105_SAMPLERATE_1000);
    else if (sampleRate < 3200) _SetSampleRate(MAX30105_SAMPLERATE_1600);
    else if (sampleRate == 3200) _SetSampleRate(MAX30105_SAMPLERATE_3200);
    else _SetSampleRate(MAX30105_SAMPLERATE_50);

    //The longer the pulse width the longer range of detection you'll have
    //At 69us and 0.4mA it's about 2 inches
    //At 411us and 0.4mA it's about 6 inches
    if (pulseWidth < 118) _SetPulseWidth(MAX30105_PULSEWIDTH_69); //Page 26, Gets us 15 bit resolution
    else if (pulseWidth < 215) _SetPulseWidth(MAX30105_PULSEWIDTH_118); //16 bit resolution
    else if (pulseWidth < 411) _SetPulseWidth(MAX30105_PULSEWIDTH_215); //17 bit resolution
    else if (pulseWidth == 411) _SetPulseWidth(MAX30105_PULSEWIDTH_411); //18 bit resolution
    else _SetPulseWidth(MAX30105_PULSEWIDTH_69);
    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

    //LED Pulse Amplitude Configuration
    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    //Default is 0x1F which gets us 6.4mA
    //powerLevel = 0x02, 0.4mA - Presence detection of ~4 inch
    //powerLevel = 0x1F, 6.4mA - Presence detection of ~8 inch
    //powerLevel = 0x7F, 25.4mA - Presence detection of ~8 inch
    //powerLevel = 0xFF, 50.0mA - Presence detection of ~12 inch

    _SetPulseAmplitudeRed(powerLevel);
    _SetPulseAmplitudeIR(powerLevel);
    _SetPulseAmplitudeGreen(powerLevel);
    _SetPulseAmplitudeProximity(powerLevel);
    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

    //Multi-LED Mode Configuration, Enable the reading of the three LEDs
    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    _EnableSlot(1, SLOT_RED_LED);
    if (ledMode > 1) _EnableSlot(2, SLOT_IR_LED);
    if (ledMode > 2) _EnableSlot(3, SLOT_GREEN_LED);
    //enableSlot(1, SLOT_RED_PILOT);
    //enableSlot(2, SLOT_IR_PILOT);
    //enableSlot(3, SLOT_GREEN_PILOT);
    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

    _ClearFIFO(); //Reset the FIFO before we begin checking the sensors
}

float Max3010x::ReadTemperature() {
    //DIE_TEMP_RDY interrupt must be enabled
    //See issue 19: https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library/issues/19
  
    // Step 1: Config die temperature register to take 1 temperature sample
    _p->WriteByteToRegister(MAX30105_DIETEMPCONFIG, 0x01);

    // Poll for bit to clear, reading is then complete
    // Timeout after 100ms
    unsigned long startTime = esp_timer_get_time();
    while (esp_timer_get_time() - startTime < 100)  {
        //uint8_t response = readRegister8(_i2caddr, MAX30105_DIETEMPCONFIG); //Original way
        //if ((response & 0x01) == 0) break; //We're done!

        //Check to see if DIE_TEMP_RDY interrupt is set
        uint8_t response;
        _p->ReadRegister(MAX30105_INTSTAT2, &response, 1);
        if ((response & MAX30105_INT_DIE_TEMP_RDY_ENABLE) > 0) break; //We're done!
        vTaskDelay(1000); //Let's not over burden the I2C bus
    }
    //TODO How do we want to fail? With what type of error?
    //? if(millis() - startTime >= 100) return(-999.0);

    // Step 2: Read die temperature register (integer)
    uint8_t tempInt;
    _p->ReadRegister(MAX30105_DIETEMPINT, &tempInt, 1);
    uint8_t tempFrac;
    _p->ReadRegister(MAX30105_DIETEMPFRAC, &tempFrac, 1); //Causes the clearing of the DIE_TEMP_RDY interrupt

    // Step 3: Calculate temperature (datasheet pg. 23)
    return (float)tempInt + ((float)tempFrac * 0.0625);
}
