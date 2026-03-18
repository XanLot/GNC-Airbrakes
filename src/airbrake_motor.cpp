#include "airbrake_motor.hpp"

AirbrakeMotor::AirbrakeMotor(int stepPin, int dirPin, int slpPin)
    : stepPin_(stepPin), dirPin_(dirPin), slpPin_(slpPin), lastStepTime_(0) {}

void AirbrakeMotor::begin() {
    pinMode(stepPin_, OUTPUT);
    pinMode(dirPin_, OUTPUT);
    pinMode(slpPin_, OUTPUT);

    digitalWrite(slpPin_, HIGH); // keep driver awake by default
}

void AirbrakeMotor::update(AirbrakeStatus status) {
    static bool justWoke = false;

    switch (status) {

        case AirbrakeStatus::LOCKED:
        case AirbrakeStatus::PERMITTED:
            digitalWrite(slpPin_, LOW);
            justWoke = false;
            break;

        case AirbrakeStatus::ACTIVE_CONT:
            if (!justWoke) {
                digitalWrite(slpPin_, HIGH);
                delayMicroseconds(1500); // wake time
                justWoke = true;
            }

            digitalWrite(dirPin_, HIGH);
            stepOnce();
            break;
    }
}

void AirbrakeMotor::stepOnce() {
    if (micros() - lastStepTime_ >= stepIntervalUs_) {
        digitalWrite(stepPin_, HIGH);
        delayMicroseconds(2); // minimum pulse width
        digitalWrite(stepPin_, LOW);
        lastStepTime_ = micros();
    }
}