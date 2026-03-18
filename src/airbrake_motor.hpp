#pragma once
#include <Arduino.h>

enum class AirbrakeStatus {
    LOCKED,
    PERMITTED,
    ACTIVE_CONT
};

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