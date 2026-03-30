#pragma once
#include <Arduino.h>
#include "IntervalTimer.h"
#include "state_machine.hpp"

class AirbrakeMotor {
public:
    // Motor control states
    enum MotorState {
        IDLE,           // Not stepping
        EXTENDING,      // Continuously extending airbrakes
        RETRACTING,     // Continuously retracting airbrakes
        AT_TARGET       // Stepping towards a target position
    };

    AirbrakeMotor(int stepPin, int dirPin, int slpPin);
    ~AirbrakeMotor();  // Cleanup timer

    void begin();
    void update(AirbrakeStatus status);
    void moveToPosition(long targetSteps);
    void setCurrentPosition(long position);
    void continuousExtend();
    void continuousRetract();
    void stop();
    void setSteppingIntervalUs(unsigned long intervalUs);
    MotorState getState() const { return motorState_; }
    bool isAtTarget() const { return currentPosition_ == targetPosition_ && motorState_ == AT_TARGET; }

    // Static ISR function for IntervalTimer
    static void steppingISR();

private:
    int stepPin_;
    int dirPin_;
    int slpPin_;

    IntervalTimer stepperTimer_;
    unsigned long steppingIntervalUs_ = 50;  // 50µs = 20 kHz stepping frequency
    static constexpr unsigned int PULSE_WIDTH_US = 2;

    volatile long currentPosition_;
    volatile long targetPosition_;
    volatile MotorState motorState_ = IDLE;
    volatile bool pulseHighPhase_ = false;  // Track pulse HIGH/LOW phase

    // Static instance pointer for ISR access
    static AirbrakeMotor* activeInstance_;

    void doSteppingISR();
    void sendStepPulse();  // Non-blocking pulse generation
    void setState(MotorState newState) { motorState_ = newState; }  // Private state setter
};