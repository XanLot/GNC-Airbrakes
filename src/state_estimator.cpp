#include "state_estimator.hpp"
#include <cmath>

// ─────────────────────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────────────────────
StateEstimator::StateEstimator()
    : phase_(EKFPhase::BOOST),
      burnout_count_(0),
      descent_count_(0),
      step_count_(0),
      frozen_(false)
{
    for (int i = 0; i < 6; i++) {
        ekf_.x[i]        = 0.0f;
        frozen_ekf_.x[i] = 0.0f;
        for (int j = 0; j < 6; j++) {
            ekf_.P[i][j]        = 0.0f;
            frozen_ekf_.P[i][j] = 0.0f;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  init
//  Sets initial state and covariance. Call once at launch with the first
//  valid barometer altitude reading.
// ─────────────────────────────────────────────────────────────────────────────
void StateEstimator::init(float initial_altitude_m) {
    // Zero everything
    for (int i = 0; i < 6; i++) {
        ekf_.x[i] = 0.0f;
        for (int j = 0; j < 6; j++) ekf_.P[i][j] = 0.0f;
    }

    // Initial state — mirrors x0_boost from EKF_combined_main.m
    ekf_.x[2] = initial_altitude_m;  // y: seed from first baro reading
    ekf_.x[3] = 1.0f;                // ydot: small launch-rail velocity guess
    ekf_.x[5] = 10.0f;               // Ty:   small vertical thrust guess

    // P0 diagonal — from EKF_combined_main.m P0_boost
    for (int i = 0; i < 6; i++) ekf_.P[i][i] = EKF_P0_BOOST[i];

    phase_         = EKFPhase::BOOST;
    burnout_count_ = 0;
    descent_count_ = 0;
    step_count_    = 0;
    frozen_        = false;
}

// ─────────────────────────────────────────────────────────────────────────────
//  getAltitude
//  Returns baro[0] altitude. Falls back to current EKF altitude on NaN.
// ─────────────────────────────────────────────────────────────────────────────
float StateEstimator::getAltitude(const SensorData& data) const {
    if (!std::isnan(data.baro[0].altitude)) {
        return data.baro[0].altitude;
    }
    return ekf_.x[2];  // fallback: hold last estimate
}

// ─────────────────────────────────────────────────────────────────────────────
//  getAccelMeasurement
//  Averages imu[0] and imu[1], skipping any NaN readings.
//  z_out[0] = lateral magnitude  sqrt(ax^2 + ay^2)  (m/s^2)
//  z_out[1] = axial body-z                           (m/s^2)
// ─────────────────────────────────────────────────────────────────────────────
void StateEstimator::getAccelMeasurement(const SensorData& data,
                                          float z_out[2]) const {
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    int   count = 0;

    for (int i = 0; i < 2; i++) {
        const Vec3& a = data.imu[i].accel;
        if (!std::isnan(a.x) && !std::isnan(a.y) && !std::isnan(a.z)) {
            sum_x += a.x;
            sum_y += a.y;
            sum_z += a.z;
            count++;
        }
    }

    if (count == 0) {
        // No valid IMU — pass zeros (EKF will rely on predict only)
        z_out[0] = 0.0f;
        z_out[1] = 0.0f;
        return;
    }

    float ax = sum_x / static_cast<float>(count);
    float ay = sum_y / static_cast<float>(count);
    float az = sum_z / static_cast<float>(count);

    z_out[0] = sqrtf(ax*ax + ay*ay);  // lateral magnitude
    z_out[1] = az;                     // axial body-z
}

// ─────────────────────────────────────────────────────────────────────────────
//  getLateralScale
//  Returns 1e6 (suppress lateral channel) when the estimated flight angle
//  from vertical is below EKF_LATERAL_THRESHOLD (10 deg), otherwise 2.
//  Mirrors the near-vertical suppression in EKF_combined_main.m.
// ─────────────────────────────────────────────────────────────────────────────
float StateEstimator::getLateralScale() const {
    float beta_est = fabsf(atan2f(ekf_.x[1], ekf_.x[3] + EKF_EP));
    return (beta_est < EKF_LATERAL_THRESHOLD) ? 1.0e6f : 2.0f;
}

// ─────────────────────────────────────────────────────────────────────────────
//  update
//  Main entry point — call every IMU tick (~480 Hz).
//  Returns a const reference to the current EKF state (or the frozen state
//  once DESCENT is detected).
// ─────────────────────────────────────────────────────────────────────────────
const EKFState& StateEstimator::update(const SensorData& data) {

    // ── DESCENT: state is frozen, return immediately ──────────────────────────
    if (frozen_) return frozen_ekf_;

    // ── Sensor measurements ───────────────────────────────────────────────────
    float altitude  = getAltitude(data);
    float z_accel[2];
    getAccelMeasurement(data, z_accel);
    float lat_scale = getLateralScale();

    // Drag parameter: k = rho * Cd * A / 2  (updated each step via stdatmo)
    float rho = stdatmo(altitude);
    float k   = rho * CD_R * A_R / 2.0f;

    // True when a baro update should be applied this tick
    bool do_baro = (step_count_ % BARO_RATIO == 0);

    // ── Phase state machine ───────────────────────────────────────────────────
    switch (phase_) {

        // ── BOOST ─────────────────────────────────────────────────────────────
        case EKFPhase::BOOST:

            boost_predict(ekf_, DT, k, MASS, EKF_G, EKF_Q_BOOST);

            if (do_baro)
                boost_update_baro(ekf_, altitude, R_BARO);

            boost_update_accel(ekf_, z_accel, R_ACCEL, k, MASS, EKF_G, lat_scale);

            // Burnout detection: |Ty| below threshold for BURNOUT_SAMPLES in a row
            if (fabsf(ekf_.x[5]) < THRUST_ACCEL_TOL) {
                burnout_count_++;
                if (burnout_count_ >= BURNOUT_SAMPLES) {
                    phase_         = EKFPhase::COAST;
                    ekf_.x[4]      = 0.0f;  // zero Tx at handoff
                    ekf_.x[5]      = 0.0f;  // zero Ty at handoff
                    burnout_count_ = 0;
                }
            } else {
                burnout_count_ = 0;
            }
            break;

        // ── COAST ─────────────────────────────────────────────────────────────
        case EKFPhase::COAST:

            coast_predict(ekf_, DT, k, MASS, EKF_G, EKF_Q_COAST);

            if (do_baro)
                coast_update_baro(ekf_, altitude, R_BARO);

            coast_update_accel(ekf_, z_accel, R_ACCEL, k, MASS, EKF_G, lat_scale);

            // Thrust is always zero during coast — clamp each step to prevent
            // numerical drift from pushing these states away from zero
            ekf_.x[4] = 0.0f;
            ekf_.x[5] = 0.0f;

            // Descent detection: ydot < 0 for DESCENT_SAMPLES in a row
            if (ekf_.x[3] < 0.0f) {
                descent_count_++;
                if (descent_count_ >= DESCENT_SAMPLES) {
                    phase_      = EKFPhase::DESCENT;
                    frozen_ekf_ = ekf_;   // snapshot state at apogee
                    frozen_     = true;
                }
            } else {
                descent_count_ = 0;
            }
            break;

        // ── DESCENT: unreachable here (handled by early return above) ─────────
        case EKFPhase::DESCENT:
            break;
    }

    step_count_++;
    return frozen_ ? frozen_ekf_ : ekf_;
}
