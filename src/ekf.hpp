#ifndef EKF_HPP
#define EKF_HPP

// ─────────────────────────────────────────────────────────────────────────────
//  Extended Kalman Filter — Boost + Coast
//
//  State vector (6-state, both phases):
//    x[0] - horizontal position       (m)
//    x[1] - horizontal velocity       (m/s)
//    x[2] - vertical position         (m)
//    x[3] - vertical velocity         (m/s)
//    x[4] - Tx: horizontal thrust accel  (m/s^2)  [zero-driven during coast]
//    x[5] - Ty: vertical thrust accel    (m/s^2)  [zero-driven during coast]
//
//  Both phases share the same 6-state vector so the boost→coast handoff
//  requires no state resizing; Tx/Ty simply stop being driven at burnout.
// ─────────────────────────────────────────────────────────────────────────────

struct EKFState {
    float x[6];       // state vector
    float P[6][6];    // error covariance matrix
};

// ── Physical constants ─────────────────────────────────────────────────────────
constexpr float EKF_G  = 9.80665f;
constexpr float EKF_EP = 1.0e-4f;  // singularity guard added to y_dot in atan2

// ── Process noise Q diagonal values (indices match state vector) ───────────────
// Tuned values from MATLAB CoordDesc optimization (EKF_combined_main.m)
constexpr float EKF_Q_BOOST[6] = {
    0.0100000000f,   // Q_x
    0.0026445998f,   // Q_xdot
    0.0100000000f,   // Q_y
    0.4538447820f,   // Q_ydot
    4.5708501960f,   // Q_Tx
    4.1808228529f    // Q_Ty
};

constexpr float EKF_Q_COAST[6] = {
    0.01f,           // Q_x
    0.0026445998f,   // Q_xdot
    0.01f,           // Q_y
    0.4538447820f,   // Q_ydot
    0.0f,            // Q_Tx  (no thrust uncertainty during coast)
    0.0f             // Q_Ty
};

// ── Initial covariance P0 diagonal — from EKF_combined_main.m ─────────────────
// P0_boost = diag([1, 0.01, 1, 0.01, 1000, 1000])
constexpr float EKF_P0_BOOST[6] = { 1.0f, 0.01f, 1.0f, 0.01f, 1000.0f, 1000.0f };

// ── Lateral accel suppression threshold ───────────────────────────────────────
// When flight angle from vertical is below this value the lateral accelerometer
// channel is suppressed (lat_scale = 1e6) to avoid Rayleigh noise bias.
// 10 degrees in radians — from EKF_combined_main.m lateral_threshold
constexpr float EKF_LATERAL_THRESHOLD = 0.17453f;

// ─────────────────────────────────────────────────────────────────────────────
//  Function declarations
// ─────────────────────────────────────────────────────────────────────────────

// 1976 Standard Atmosphere: geopotential altitude (m) → air density (kg/m^3)
float stdatmo(float h_m);

// ── Boost phase (6-state) ─────────────────────────────────────────────────────
// Port of: ekf_predict_boost.m, ekf_update_baro.m, ekf_update_accel.m

void boost_predict(EKFState& s, float dt, float k, float m, float g,
                   const float Q[6]);

void boost_update_baro(EKFState& s, float z_baro, float R_baro);

// z_accel[0] = lateral magnitude (m/s^2), z_accel[1] = body-z axial (m/s^2)
// lat_scale:  R multiplier for lateral channel (pass 1e6 to suppress near-vertical)
void boost_update_accel(EKFState& s, const float z_accel[2], float R_accel,
                        float k, float m, float g, float lat_scale);

// ── Coast phase (6-state, Tx/Ty zero-driven) ──────────────────────────────────
// Port of: coast_predict, coast_update_baro, coast_update_accel from
// EKF_combined_main.m local functions, extended from 4-state to 6-state.

void coast_predict(EKFState& s, float dt, float k, float m, float g,
                   const float Q[6]);

void coast_update_baro(EKFState& s, float z_baro, float R_baro);

// z_accel[0] = lateral magnitude (m/s^2), z_accel[1] = body-z axial (m/s^2)
void coast_update_accel(EKFState& s, const float z_accel[2], float R_accel,
                        float k, float m, float g, float lat_scale);

// Helper: predicted accelerometer measurement from 6-state (coast, no thrust).
// h_out[0] = abs(body_x lateral),  h_out[1] = body_z axial
void coast_accel_h(const float x[6], float k, float m, float ep, float h_out[2]);

#endif // EKF_HPP
