#include "state_machine.hpp"
#include <Arduino.h>
#include <cmath>

// ─── Constructor ─────────────────────────────────────────────────────────────
StateMachine::StateMachine(sd_log& sdLog)
    : currentState_(FlightState::ON_PAD),
      sdLog_(sdLog),
      bufferHead_(0),
      bufferCount_(0),
      coastOnsetEntryMs_(0),
      previousAltitude_(0.0f)
{
    // Zero out the pre-launch circular buffer
    for (int i = 0; i < PRE_LAUNCH_BUFFER_SIZE; i++) {
        preLaunchBuffer_[i] = PreLaunchSample{};
    }
    onEnter_OnPad();
}

// ─── Main update loop ────────────────────────────────────────────────────────
void StateMachine::update(const IMUData& imu, const BarometerData& baro) {
    FlightState nextState = currentState_;

    switch (currentState_) {
        case FlightState::ON_PAD:
            storePreLaunchSample(imu, baro);
            nextState = checkTransition_OnPad(imu);
            break;

        case FlightState::BOOST:
            nextState = checkTransition_Boost(imu);
            break;

        case FlightState::COAST_ONSET:
            nextState = checkTransition_CoastOnset();
            break;

        case FlightState::COAST:
            nextState = checkTransition_Coast(baro);
            previousAltitude_ = baro.altitude;
            break;

        case FlightState::RECOVERY:
            // Terminal state — no transitions.
            break;
    }

    // Fire entry action if state changed
    if (nextState != currentState_) {
        Serial.print("[STATE] Transition: ");
        Serial.print(static_cast<int>(currentState_));
        Serial.print(" -> ");
        Serial.println(static_cast<int>(nextState));

        currentState_ = nextState;

        switch (currentState_) {
            case FlightState::ON_PAD:      onEnter_OnPad();          break;
            case FlightState::BOOST:       onEnter_Boost(imu, baro); break;
            case FlightState::COAST_ONSET: onEnter_CoastOnset();     break;
            case FlightState::COAST:       onEnter_Coast();          break;
            case FlightState::RECOVERY:    onEnter_Recovery();       break;
        }
    }
}

// ─── Getters ─────────────────────────────────────────────────────────────────
FlightState StateMachine::getState() const {
    return currentState_;
}

bool StateMachine::isLogging() const {
    return currentState_ != FlightState::ON_PAD;
}

// ─── Transition checks ──────────────────────────────────────────────────────
FlightState StateMachine::checkTransition_OnPad(const IMUData& imu) {
    if (accelMagnitude(imu) >= BOOST_ACCEL_THRESHOLD_MS2) {
        return FlightState::BOOST;
    }
    return FlightState::ON_PAD;
}

FlightState StateMachine::checkTransition_Boost(const IMUData& imu) {
    if (accelMagnitude(imu) <= BURNOUT_ACCEL_THRESHOLD_MS2) {
        return FlightState::COAST_ONSET;
    }
    return FlightState::BOOST;
}

FlightState StateMachine::checkTransition_CoastOnset() {
    if (millis() - coastOnsetEntryMs_ >= static_cast<unsigned long>(COAST_TIMER_SECONDS * 1000)) {
        return FlightState::COAST;
    }
    return FlightState::COAST_ONSET;
}

FlightState StateMachine::checkTransition_Coast(const BarometerData& baro) {
    if (baro.altitude < previousAltitude_) {
        return FlightState::RECOVERY;
    }
    return FlightState::COAST;
}

// ─── Entry actions ──────────────────────────────────────────────────────────
void StateMachine::onEnter_OnPad() {
    Serial.println("[STATE] ON_PAD: Waiting for launch. Airbrakes locked.");
    setAirbrakeStatus(AirbrakeStatus::LOCKED);
}

void StateMachine::onEnter_Boost(const IMUData& imu, const BarometerData& baro) {
    Serial.println("[STATE] BOOST: Motor burning. Airbrakes locked.");
    setAirbrakeStatus(AirbrakeStatus::LOCKED);
    flushPreLaunchBuffer();
}

void StateMachine::onEnter_CoastOnset() {
    Serial.println("[STATE] COAST_ONSET: Burnout detected. Airbrakes permitted.");
    setAirbrakeStatus(AirbrakeStatus::PERMITTED);
    coastOnsetEntryMs_ = millis();
}

void StateMachine::onEnter_Coast() {
    Serial.println("[STATE] COAST: GNC active. Airbrakes under closed-loop control.");
    setAirbrakeStatus(AirbrakeStatus::ACTIVE_CONT);
}

void StateMachine::onEnter_Recovery() {
    Serial.println("[STATE] RECOVERY: Apogee passed. Airbrakes locked.");
    setAirbrakeStatus(AirbrakeStatus::LOCKED);
}

// ─── Airbrake status stub ───────────────────────────────────────────────────
void StateMachine::setAirbrakeStatus(AirbrakeStatus status) {
    Serial.print("[AIRBRAKE] Status set to: ");
    switch (status) {
        case AirbrakeStatus::LOCKED:      Serial.println("LOCKED");      break;
        case AirbrakeStatus::PERMITTED:   Serial.println("PERMITTED");   break;
        case AirbrakeStatus::ACTIVE_CONT: Serial.println("ACTIVE_CONT"); break;
    }
}

// ─── Acceleration magnitude ─────────────────────────────────────────────────
float StateMachine::accelMagnitude(const IMUData& imu) {
    return sqrtf(imu.accel.x * imu.accel.x +
                 imu.accel.y * imu.accel.y +
                 imu.accel.z * imu.accel.z);
}

// ─── Circular buffer: store one pre-launch sample ───────────────────────────
void StateMachine::storePreLaunchSample(const IMUData& imu, const BarometerData& baro) {
    preLaunchBuffer_[bufferHead_].imu  = imu;
    preLaunchBuffer_[bufferHead_].baro = baro;
    bufferHead_ = (bufferHead_ + 1) % PRE_LAUNCH_BUFFER_SIZE;
    if (bufferCount_ < PRE_LAUNCH_BUFFER_SIZE) {
        bufferCount_++;
    }
}

// ─── Flush pre-launch buffer to SD ──────────────────────────────────────────
void StateMachine::flushPreLaunchBuffer() {
    // Determine the starting index: oldest sample in the circular buffer
    int start;
    if (bufferCount_ < PRE_LAUNCH_BUFFER_SIZE) {
        start = 0;  // Buffer hasn't wrapped yet — oldest is at index 0
    } else {
        start = bufferHead_;  // Buffer is full — oldest is at bufferHead_
    }

    for (int i = 0; i < bufferCount_; i++) {
        int idx = (start + i) % PRE_LAUNCH_BUFFER_SIZE;
        sdLog_.log(preLaunchBuffer_[idx].imu, preLaunchBuffer_[idx].baro);
    }

    Serial.print("[STATE] Flushed ");
    Serial.print(bufferCount_);
    Serial.println(" pre-launch samples to SD.");
}
