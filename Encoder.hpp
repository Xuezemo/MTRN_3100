#pragma once

#include <Arduino.h>

namespace mtrn3100 {

class Encoder {
public:
    Encoder(uint8_t enc1, uint8_t enc2) 
        : encoder1_pin(enc1), encoder2_pin(enc2) {

        pinMode(encoder1_pin, INPUT_PULLUP);
        pinMode(encoder2_pin, INPUT_PULLUP);

        if (instance_count < 2) {
            slot[instance_count] = this;

            if (instance_count == 0) {
                attachInterrupt(digitalPinToInterrupt(enc1), isr0, RISING);
            } else if (instance_count == 1) {
                attachInterrupt(digitalPinToInterrupt(enc1), isr1, RISING);
            }

            instance_count++;
        } else {

        }
    }

    void readEncoder() {
        noInterrupts();
        if (digitalRead(encoder2_pin) == HIGH) {
            count++;
        } else {
            count--;
        }
        interrupts();
    }

    int32_t getCount() const {
        return count;
    }

    void reset() {
        count = 0;
    }

    float getRotation() {
        return (2.0f * PI * count) / counts_per_revolution;
    }

    float getDistance() {
    return getRotation() * wheel_radius * 1000.0f;  
}

public:
    const uint8_t encoder1_pin;
    const uint8_t encoder2_pin;
    uint16_t counts_per_revolution = 680;
    float wheel_radius =  0.017f;
    volatile long count = 0;

private:
    static void isr0() {
        if (slot[0]) slot[0]->readEncoder();
    }

    static void isr1() {
        if (slot[1]) slot[1]->readEncoder();
    }

    static Encoder* slot[2];
    static uint8_t instance_count;
};

Encoder* Encoder::slot[2] = {nullptr, nullptr};
uint8_t Encoder::instance_count = 0;

}