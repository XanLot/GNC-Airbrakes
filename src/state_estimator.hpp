#ifndef STATE_ESTIMATOR_HPP
#define STATE_ESTIMATOR_HPP

#include "ekf.hpp"
#include "sensor_data.hpp"

// ─────────────────────────────────────────────────────────────────────────────
//  EKF flight phase
//
//  BOOST   — motor burning, 6-state EKF with thrust (Tx, Ty) active
//  COAST   — motor burned out, Tx/Ty forced to zero each step
//  DESCENT — apogee passed (ydot < 0 for DESCENT_SAMPLES consecutive steps),
//             EKF frozen; last valid state returned until power-off
// ─────────────────────────────────────────────────────────────────────────────
enum class EKFPhase { BOOST, COAST, DESCENT };

// ─────────────────────────────────────────────────────────────────────────────
//  StateEstimator
//
//  Orchestrates the EKF: reads SensorData each loop tick, selects boost or
//  coast dynamics, manages phase transitions, and returns the current state.
//
//  Usage:
//    StateEstimator estimator;
//    estimator.init(baro.altitude);   // call once at launch
//    const EKFState& state = estimator.update(sensorData);  // call each tick
// ─────────────────────────────────────────────────────────────────────────────
class StateEstimator {
public:
    StateEstimator();

    // Call once before the first update, with the initial barometer altitude.
    void init(float initial_altitude_m);

    // Call every IMU tick. Returns a reference to the current EKF state.
    // In DESCENT phase the reference points to the frozen state.
    const EKFState& update(const SensorData& data);

    EKFPhase getPhase() const { return phase_; }

private:
    EKFState ekf_;
    EKFState frozen_ekf_;     // snapshot taken at descent detection
    EKFPhase phase_;

    int  burnout_count_;      // consecutive low-thrust samples
    int  descent_count_;      // consecutive negative-ydot samples
    int  step_count_;         // total steps since init (used for baro timing)
    bool frozen_;             // true once DESCENT is detected

    // ── Vehicle parameters ───────────────────────────────────────────────────
    static constexpr float MASS = 1.0f;    // kg — coast/boost mass
    static constexpr float CD_R = 0.45f;   // rocket body drag coefficient
    static constexpr float A_R  = 0.008f;  // rocket body reference area (m^2)

    // ── Phase transition thresholds ──────────────────────────────────────────
    // Burnout: |Ty| below this for BURNOUT_SAMPLES consecutive ticks → COAST
    static constexpr float THRUST_ACCEL_TOL  = 5.0f;  // m/s^2
    static constexpr int   BURNOUT_SAMPLES   = 3;

    // Descent: ydot < 0 for DESCENT_SAMPLES consecutive ticks → freeze
    static constexpr int   DESCENT_SAMPLES   = 10;

    // ── Timing ───────────────────────────────────────────────────────────────
    // IMU ODR = 480 Hz  →  dt = 1/480 s
    // Baro ODR = 100 Hz  →  apply baro update every ~5 accel ticks
    static constexpr float DT         = 1.0f / 480.0f;
    static constexpr int   BARO_RATIO = 5;

    // ── Measurement noise — derived from sensor specs in sim_config.m ────────
    // R_accel = (accel_noise_density * sqrt(accel_odr/2) * g)^2
    //         = (60e-6 * sqrt(240) * 9.80665)^2  ≈ 8.31e-5 (m/s^2)^2
    // R_baro  = (pressure_noise_pa / sqrt(osr_n) / 8.5)^2
    //         = (0.09 / sqrt(4) / 8.5)^2          ≈ 2.80e-5 m^2
    static constexpr float R_ACCEL = 8.31e-5f;
    static constexpr float R_BARO  = 2.80e-5f;

    // ── Private helpers ──────────────────────────────────────────────────────
    // Average altitude from baro[0]; falls back to current EKF estimate on NaN.
    float getAltitude(const SensorData& data) const;

    // Average accel from imu[0] and imu[1], skipping NaN readings.
    // z_out[0] = lateral magnitude sqrt(ax^2 + ay^2)
    // z_out[1] = axial body-z
    void  getAccelMeasurement(const SensorData& data, float z_out[2]) const;

    // Lateral channel noise scale: 2 normally, 1e6 when near-vertical.
    float getLateralScale() const;
};

#endif // STATE_ESTIMATOR_HPP
