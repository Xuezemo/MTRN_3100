#pragma once

#include <Wire.h>
#include <VL6180X.h>

namespace mtrn3100 {

class Lidar {
public:
    Lidar(uint8_t xshut_pin, uint8_t i2c_address)
        : xshut_pin(xshut_pin), i2c_address(i2c_address) {}

    void begin() {
        pinMode(xshut_pin, OUTPUT);
        digitalWrite(xshut_pin, LOW);    // Shutdown sensor
        delay(5);

        digitalWrite(xshut_pin, HIGH);   // Power up sensor
        delay(50);                       // Let it stabilize

        sensor.init();
        sensor.configureDefault();
        sensor.setTimeout(250);           // Set small timeout
        sensor.setAddress(i2c_address);  // Assign new address
    }

    /**
     * Reads distance in mm from VL6180X
     * This function is blocking for ~5-8ms (sensor spec), but no long delay().
     */
    uint16_t readDistance() {
        uint16_t distance = sensor.readRangeSingleMillimeters();
        return distance;
    }

    bool timeoutOccurred() const {
        return sensor.timeoutOccurred();
    }

private:
    uint8_t xshut_pin;
    uint8_t i2c_address;
    VL6180X sensor;
};

}  // namespace mtrn3100