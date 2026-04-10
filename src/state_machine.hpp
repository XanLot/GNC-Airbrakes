#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include "sensor_data.hpp"
#include "sd_log_file.hpp"
#include "stepper.hpp"

enum class FlightState {
    ON_PAD,
    BOOST,
    COAST_ONSET,
    COAST,
    RECOVERY
};

enum class AirbrakeStatus {
    LOCKED,
    PERMITTED,
    ACTIVE_CONT
};

constexpr float BOOST_ACCEL_THRESHOLD_MS2   = 5.0f * 9.81f;
constexpr float BURNOUT_ACCEL_THRESHOLD_MS2 = 0.5f * 9.81f;
constexpr float COAST_TIMER_SECONDS         = 3.0f;
constexpr int   PRE_LAUNCH_BUFFER_SIZE      = 100;
constexpr int   BURNOUT_CONFIRM_SAMPLES     = 3;
constexpr int   APOGEE_CONFIRM_SAMPLES      = 5;

struct PreLaunchSample {
    SensorData data;
};

class StateMachine {
public:
    StateMachine(sd_log& sdLog, Stepper& stepper);

    void update(const SensorData& data);
    FlightState getState() const;
    bool isLogging() const;

private:
    FlightState currentState_;
    sd_log&     sdLog_;
    Stepper&    stepper_;

    static constexpr int DEPLOY_STEPS = 400;

    PreLaunchSample preLaunchBuffer_[PRE_LAUNCH_BUFFER_SIZE];
    int             bufferHead_;
    int             bufferCount_;

    unsigned long coastOnsetEntryMs_;
    float previousAltitude_;
    int   altitudeDecreasingCount_;
    int   burnoutConfirmCount_;

    FlightState checkTransition_OnPad(const SensorData& data);
    FlightState checkTransition_Boost(const SensorData& data);
    FlightState checkTransition_CoastOnset();
    FlightState checkTransition_Coast(const SensorData& data);

    void onEnter_OnPad();
    void onEnter_Boost(const SensorData& data);
    void onEnter_CoastOnset();
    void onEnter_Coast();
    void onEnter_Recovery();

    void setAirbrakeStatus(AirbrakeStatus status);
    void deployAirbrakes();
    void retractAirbrakes();

    static float accelMagnitude(const IMUData& imu);
    void         storePreLaunchSample(const SensorData& data);
    void         flushPreLaunchBuffer();
};

#endif // STATE_MACHINE_HPP
