#include "state_machine.hpp"
#include <Arduino.h>
#include <cmath>

StateMachine::StateMachine(sd_log& sdLog)
    : currentState_(FlightState::ON_PAD),
      sdLog_(sdLog),
      bufferHead_(0),
      bufferCount_(0),
      coastOnsetEntryMs_(0),
      previousAltitude_(0.0f),
      altitudeDecreasingCount_(0),
      burnoutConfirmCount_(0)
{
    for (int i = 0; i < PRE_LAUNCH_BUFFER_SIZE; i++) {
        preLaunchBuffer_[i] = PreLaunchSample{};
    }
    onEnter_OnPad();
}

// ── Stepper init — call once in setup() ──────────────────────────────────
void StateMachine::initStepper() {
    stepper_.setMaxSpeed(MAX_SPEED);
    stepper_.setAcceleration(ACCELERATION);
    stepper_.setCurrentPosition(0);
    Serial.println("[STEPPER] Initialized at home position.");
}

// ── Stepper run — call every loop() ──────────────────────────────────────
void StateMachine::runStepper() {
    stepper_.run();
}

void StateMachine::update(const SensorData& data) {
    FlightState nextState = currentState_;

    switch (currentState_) {
        case FlightState::ON_PAD:
            storePreLaunchSample(data);
            nextState = checkTransition_OnPad(data);
            break;
        case FlightState::BOOST:
            nextState = checkTransition_Boost(data);
            break;
        case FlightState::COAST_ONSET:
            nextState = checkTransition_CoastOnset();
            break;
        case FlightState::COAST:
            nextState = checkTransition_Coast(data);
            previousAltitude_ = data.baro[0].altitude;
            break;
        case FlightState::RECOVERY:
            break;
    }

    if (nextState != currentState_) {
        Serial.print("[STATE] Transition: ");
        Serial.print(static_cast<int>(currentState_));
        Serial.print(" -> ");
        Serial.println(static_cast<int>(nextState));

        currentState_ = nextState;

        switch (currentState_) {
            case FlightState::ON_PAD:
                onEnter_OnPad();
                break;
            case FlightState::BOOST:
                onEnter_Boost(data);
                break;
            case FlightState::COAST_ONSET:
                onEnter_CoastOnset();
                break;
            case FlightState::COAST:
                onEnter_Coast();
                break;
            case FlightState::RECOVERY:
                onEnter_Recovery();
                break;
        }
    }
}

FlightState StateMachine::getState() const { return currentState_; }
bool StateMachine::isLogging() const { return currentState_ != FlightState::ON_PAD; }

FlightState StateMachine::checkTransition_OnPad(const SensorData& data) {
    if (accelMagnitude(data.imu[0]) >= BOOST_ACCEL_THRESHOLD_MS2)
        return FlightState::BOOST;
    return FlightState::ON_PAD;
}

FlightState StateMachine::checkTransition_Boost(const SensorData& data) {
    if (accelMagnitude(data.imu[0]) <= BURNOUT_ACCEL_THRESHOLD_MS2) {
        burnoutConfirmCount_++;
    } else {
        burnoutConfirmCount_ = 0;
    }
    if (burnoutConfirmCount_ >= BURNOUT_CONFIRM_SAMPLES)
        return FlightState::COAST_ONSET;
    return FlightState::BOOST;
}

FlightState StateMachine::checkTransition_CoastOnset() {
    if (millis() - coastOnsetEntryMs_ >= static_cast<unsigned long>(COAST_TIMER_SECONDS * 1000))
        return FlightState::COAST;
    return FlightState::COAST_ONSET;
}

FlightState StateMachine::checkTransition_Coast(const SensorData& data) {
    if (data.baro[0].altitude < previousAltitude_) {
        altitudeDecreasingCount_++;
    } else {
        altitudeDecreasingCount_ = 0;
    }
    if (altitudeDecreasingCount_ >= APOGEE_CONFIRM_SAMPLES)
        return FlightState::RECOVERY;
    return FlightState::COAST;
}

void StateMachine::onEnter_OnPad() {
    Serial.println("[STATE] ON_PAD: Waiting for launch. Airbrakes locked.");
    setAirbrakeStatus(AirbrakeStatus::LOCKED);
    retractAirbrakes();
}

void StateMachine::onEnter_Boost(const SensorData& data) {
    (void)data;
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
    deployAirbrakes();
}

void StateMachine::onEnter_Recovery() {
    Serial.println("[STATE] RECOVERY: Apogee passed. Airbrakes locked.");
    setAirbrakeStatus(AirbrakeStatus::LOCKED);
    retractAirbrakes();
}

void StateMachine::deployAirbrakes() {
    Serial.print("[STEPPER] Deploying to step ");
    delay(1000); // Manual delay to ensure in coast phase, will have to change when adding MCP.
    Serial.println(DEPLOY_STEPS);
    stepper_.moveTo(DEPLOY_STEPS);
}

void StateMachine::retractAirbrakes() {
    Serial.println("[STEPPER] Retracting to home.");
    stepper_.moveTo(0);
}

void StateMachine::setAirbrakeStatus(AirbrakeStatus status) {
    Serial.print("[AIRBRAKE] Status set to: ");
    switch (status) {
        case AirbrakeStatus::LOCKED:
            Serial.println("LOCKED");
            break;
        case AirbrakeStatus::PERMITTED:
            Serial.println("PERMITTED");
            break;
        case AirbrakeStatus::ACTIVE_CONT:
            Serial.println("ACTIVE_CONT");
            break;
    }
}

float StateMachine::accelMagnitude(const IMUData& imu) {
    return sqrtf(imu.accel.x * imu.accel.x +
                 imu.accel.y * imu.accel.y +
                 imu.accel.z * imu.accel.z);
}

void StateMachine::storePreLaunchSample(const SensorData& data) {
    preLaunchBuffer_[bufferHead_].data = data;
    bufferHead_ = (bufferHead_ + 1) % PRE_LAUNCH_BUFFER_SIZE;
    if (bufferCount_ < PRE_LAUNCH_BUFFER_SIZE) bufferCount_++;
}

void StateMachine::flushPreLaunchBuffer() {
    int start = (bufferCount_ < PRE_LAUNCH_BUFFER_SIZE) ? 0 : bufferHead_;
    for (int i = 0; i < bufferCount_; i++) {
        int idx = (start + i) % PRE_LAUNCH_BUFFER_SIZE;
        sdLog_.log(preLaunchBuffer_[idx].data);
    }
    Serial.print("[STATE] Flushed ");
    Serial.print(bufferCount_);
    Serial.println(" pre-launch samples to SD.");
}
