#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include "imu.hpp"
#include "barometer.hpp"
#include "sd_log_file.hpp"

// ─── Flight states ────────────────────────────────────────────────────────────
// The rocket passes through these states in order during a normal flight.
enum class FlightState {
    ON_PAD,       // Sitting on the launch pad. Airbrakes locked. No SD logging.
    BOOST,        // Motor burning. High acceleration. Airbrakes locked.
    COAST_ONSET,  // Motor burned out. Airbrakes permitted but not yet controlled.
    COAST,        // Closed-loop GNC active. Airbrakes commanded by algorithm.
    RECOVERY      // Apogee passed, rocket descending. Airbrakes locked.
};

// ─── Airbrake status ──────────────────────────────────────────────────────────
// Describes what the airbrake actuator is allowed/commanded to do.
enum class AirbrakeStatus {
    LOCKED,       // Airbrake held closed. Motor not powered.
    PERMITTED,    // Airbrake may extend, but no active control yet.
    ACTIVE_CONT   // Closed-loop GNC is actively commanding the airbrake.
};

// ─── Configurable transition thresholds ──────────────────────────────────────
// Change these to tune when state transitions fire.
constexpr float BOOST_ACCEL_THRESHOLD_MS2   = 5.0f * 9.81f;  // 5g in m/s²
constexpr float BURNOUT_ACCEL_THRESHOLD_MS2 = 0.0f;          // net accel <= 0
constexpr float COAST_TIMER_SECONDS         = 3.0f;          // seconds in COAST_ONSET before COAST
constexpr int   PRE_LAUNCH_BUFFER_SIZE      = 100;           // ~1 second at 100 Hz IMU

// ─── Pre-launch sample ────────────────────────────────────────────────────────
// One snapshot stored in the circular buffer during ON_PAD.
struct PreLaunchSample {
    IMUData imu;
    BarometerData baro;
};

// ─── StateMachine ─────────────────────────────────────────────────────────────
class StateMachine {
public:
    explicit StateMachine(sd_log& sdLog);

    void update(const IMUData& imu, const BarometerData& baro);
    FlightState getState() const;
    bool isLogging() const;

private:
    FlightState currentState_;
    sd_log&     sdLog_;

    PreLaunchSample preLaunchBuffer_[PRE_LAUNCH_BUFFER_SIZE];
    int             bufferHead_;
    int             bufferCount_;

    unsigned long coastOnsetEntryMs_;
    float previousAltitude_;

    FlightState checkTransition_OnPad(const IMUData& imu);
    FlightState checkTransition_Boost(const IMUData& imu);
    FlightState checkTransition_CoastOnset();
    FlightState checkTransition_Coast(const BarometerData& baro);

    void onEnter_OnPad();
    void onEnter_Boost(const IMUData& imu, const BarometerData& baro);
    void onEnter_CoastOnset();
    void onEnter_Coast();
    void onEnter_Recovery();

    void setAirbrakeStatus(AirbrakeStatus status);

    static float accelMagnitude(const IMUData& imu);
    void         storePreLaunchSample(const IMUData& imu, const BarometerData& baro);
    void         flushPreLaunchBuffer();
};

#endif // STATE_MACHINE_HPP
