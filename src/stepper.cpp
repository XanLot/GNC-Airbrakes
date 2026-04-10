#include "stepper.hpp"
#include "pins.hpp"
#include <Arduino.h>

Stepper::Stepper()
    : stepper_(AccelStepper::DRIVER, STEPPER_STEP, STEPPER_DIR)
{}

void Stepper::init() {
    stepper_.setMaxSpeed(MAX_SPEED);
    stepper_.setAcceleration(ACCELERATION);
    stepper_.setCurrentPosition(0);
    Serial.println("[STEPPER] Initialized at home.");
}

void Stepper::run() {
    stepper_.run();
}

void Stepper::deployTo(int steps) {
    Serial.print("[STEPPER] Deploying to step ");
    Serial.println(steps);
    stepper_.moveTo(steps);
}

void Stepper::retractToHome() {
    Serial.println("[STEPPER] Retracting to home.");
    stepper_.moveTo(0);
}

int Stepper::currentPosition() {
    return stepper_.currentPosition();
}

int Stepper::targetPosition() {
    return stepper_.targetPosition();
}

bool Stepper::isRunning() {
    return stepper_.isRunning();
}
