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
    switch (status) {

        case AirbrakeStatus::LOCKED:
            // Hold position (do NOT sleep unless mechanically locked)
            digitalWrite(slpPin_, HIGH);
            break;

        case AirbrakeStatus::PERMITTED:
            // Ready but not moving
            digitalWrite(slpPin_, HIGH);
            break;

        case AirbrakeStatus::ACTIVE_CONT:
            digitalWrite(slpPin_, HIGH);

            // Placeholder behavior (until control team finishes)
            digitalWrite(dirPin_, HIGH);  // always extend for now
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