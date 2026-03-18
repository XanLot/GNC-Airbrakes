#pragma once
#include <Arduino.h>
#include "state_machine.hpp"

class AirbrakeMotor {
public:
    AirbrakeMotor(int stepPin, int dirPin, int slpPin);

    void begin();
    void update(AirbrakeStatus status);

private:
    int stepPin_;
    int dirPin_;
    int slpPin_;

    unsigned long lastStepTime_;
    const unsigned long stepIntervalUs_ = 1000;

    void stepOnce();
};