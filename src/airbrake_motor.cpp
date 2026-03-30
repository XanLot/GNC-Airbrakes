#include "airbrake_motor.hpp"

// Static instance pointer for ISR
AirbrakeMotor* AirbrakeMotor::activeInstance_ = nullptr;

AirbrakeMotor::AirbrakeMotor(int stepPin, int dirPin, int slpPin)
    : stepPin_(stepPin), dirPin_(dirPin), slpPin_(slpPin),
      currentPosition_(0), targetPosition_(0), motorState_(IDLE),
      pulseHighPhase_(false) {}

AirbrakeMotor::~AirbrakeMotor() {
    // Stop the timer before destruction
    stepperTimer_.end();
    activeInstance_ = nullptr;
}

void AirbrakeMotor::begin() {
    pinMode(stepPin_, OUTPUT);
    pinMode(dirPin_, OUTPUT);
    pinMode(slpPin_, OUTPUT);

    digitalWrite(slpPin_, LOW);  // Start with driver in sleep mode
    
    // Register this instance for ISR access
    activeInstance_ = this;
    
    // Start the stepping timer (runs steppingISR every steppingIntervalUs_)
    stepperTimer_.begin(steppingISR, steppingIntervalUs_);
}

void AirbrakeMotor::update(AirbrakeStatus status) {
    // Manage driver wake/sleep based on flight status
    // Actual stepping happens asynchronously in the ISR
    
    switch (status) {
        case AirbrakeStatus::LOCKED:
        case AirbrakeStatus::PERMITTED:
            // Flight not in active control phase — stop stepping and sleep driver
            motorState_ = IDLE;
            digitalWrite(slpPin_, LOW);
            break;

        case AirbrakeStatus::ACTIVE_CONT:
            // Flight in active control phase — wake driver if stepping
            if (motorState_ != IDLE) {
                digitalWrite(slpPin_, HIGH);
            }
            break;
    }
}

void AirbrakeMotor::moveToPosition(long targetSteps) {
    // Validate input
    if (targetSteps < 0) {
        targetSteps = 0;  // Clamp to minimum
    }
    targetPosition_ = targetSteps;
    motorState_ = AT_TARGET;
}

void AirbrakeMotor::setCurrentPosition(long position) {
    // Set the current position to establish known state
    if (position < 0) {
        position = 0;  // Clamp to minimum
    }
    currentPosition_ = position;
}

void AirbrakeMotor::continuousExtend() {
    motorState_ = EXTENDING;
}

void AirbrakeMotor::continuousRetract() {
    motorState_ = RETRACTING;
}

void AirbrakeMotor::stop() {
    motorState_ = IDLE;
}

void AirbrakeMotor::setSteppingIntervalUs(unsigned long intervalUs) {
    // Validate input
    if (intervalUs < 10) {
        intervalUs = 10;  // Minimum to allow ISR overhead
    }
    stepperTimer_.update(intervalUs);
    steppingIntervalUs_ = intervalUs;
}

// Static ISR wrapper
void AirbrakeMotor::steppingISR() {
    if (activeInstance_) {
        activeInstance_->doSteppingISR();
    }
}

// Non-blocking pulse generation (no delays in ISR)
void AirbrakeMotor::sendStepPulse() {
    if (pulseHighPhase_) {
        // Pulse is HIGH this half-interval, set LOW next ISR call
        digitalWrite(stepPin_, LOW);
        pulseHighPhase_ = false;
    } else {
        // Pulse is LOW, set HIGH for next ISR interval
        digitalWrite(stepPin_, HIGH);
        pulseHighPhase_ = true;
    }
}

// Instance method with all stepping logic
void AirbrakeMotor::doSteppingISR() {
    switch (motorState_) {
        case EXTENDING:
            digitalWrite(dirPin_, HIGH);
            sendStepPulse();
            break;

        case RETRACTING:
            digitalWrite(dirPin_, LOW);
            sendStepPulse();
            break;

        case AT_TARGET:
            // Move towards target position
            if (currentPosition_ < targetPosition_) {
                digitalWrite(dirPin_, HIGH);
                sendStepPulse();
                // Increment position when pulse completes (LOW phase)
                if (!pulseHighPhase_) {
                    currentPosition_++;
                }
            } 
            else if (currentPosition_ > targetPosition_) {
                digitalWrite(dirPin_, LOW);
                sendStepPulse();
                // Decrement position when pulse completes (LOW phase)
                if (!pulseHighPhase_) {
                    currentPosition_--;
                }
            }
            // else: at target, do nothing
            break;

        case IDLE:
            // Not stepping
            break;
    }
}
