#ifndef AHT10_H
#define AHT10_H

#include <Wire.h>
#include <SoftWire.h>

// Enum to select the I2C bus type
enum I2CBusType { BUS_WIRE, BUS_SOFTWIRE };

class AHT10 {
public:
    /**
     * @brief Construct a new AHT10 object
     * 
     * @param address   I2C address of the sensor (default 0x38)
     * @param busType   BUS_WIRE for hardware I2C, BUS_SOFTWIRE for software I2C
     * @param wire      Pointer to Wire object (default: &Wire)
     * @param softwire  Pointer to SoftWire object (default: nullptr)
     */
    AHT10(uint8_t address, I2CBusType busType, TwoWire* wire = &Wire, SoftWire* softwire = nullptr)
        : _address(address), _busType(busType), _wire(wire), _softwire(softwire) {}

    /**
     * @brief Initialize the sensor; call this in setup()
     */
    void begin() {
        if (_busType == BUS_WIRE && _wire) {
            _wire->begin();
            _wire->beginTransmission(_address);
            _wire->write(0xE1); // Initialization command
            _wire->endTransmission();
        } else if (_busType == BUS_SOFTWIRE && _softwire) {
            _softwire->begin();
            _softwire->beginTransmission(_address);
            _softwire->write(0xE1); // Initialization command
            _softwire->endTransmission();
        }
        delay(20); // Wait for sensor initialization
    }

    /**
     * @brief Read humidity and temperature from the sensor
     * 
     * @param humidity     Reference to store humidity (%)
     * @param temperature  Reference to store temperature (C)
     * @return true        If reading is successful and data valid
     * @return false       If failed to read valid data
     */
    bool read(float &humidity, float &temperature) {
        if (_busType == BUS_WIRE && _wire) {
            _wire->beginTransmission(_address);
            _wire->write(0xAC); // Measure command
            _wire->write(0x33);
            _wire->write(0x00);
            _wire->endTransmission();
            delay(100);

            int numBytes = _wire->requestFrom(_address, (uint8_t)6);
            if (numBytes != 6) return false;
            uint8_t data[6];
            for (int i = 0; i < 6; i++) data[i] = _wire->read();
            parse(data, humidity, temperature);
            return true;
        } else if (_busType == BUS_SOFTWIRE && _softwire) {
            _softwire->beginTransmission(_address);
            _softwire->write(0xAC);
            _softwire->write(0x33);
            _softwire->write(0x00);
            _softwire->endTransmission();
            delay(100);

            int numBytes = _softwire->requestFrom(_address, (uint8_t)6);
            if (numBytes != 6) return false;
            uint8_t data[6];
            for (int i = 0; i < 6; i++) data[i] = _softwire->read();
            parse(data, humidity, temperature);
            return true;
        }
        return false;
    }

private:
    uint8_t _address;
    I2CBusType _busType;
    TwoWire* _wire;
    SoftWire* _softwire;

    /**
     * @brief Parse the 6-byte sensor data buffer into humidity and temperature
     */
    void parse(uint8_t* data, float &humidity, float &temperature) {
        unsigned long humidity_raw = ((unsigned long)data[1] << 12) | ((unsigned long)data[2] << 4) | (data[3] >> 4);
        unsigned long temp_raw = (((unsigned long)data[3] & 0x0F) << 16) | ((unsigned long)data[4] << 8) | data[5];
        humidity = humidity_raw * (100.0 / 1048576.0);
        temperature = (temp_raw * (200.0 / 1048576.0)) - 50;
    }
};

#endif // AHT10_H
