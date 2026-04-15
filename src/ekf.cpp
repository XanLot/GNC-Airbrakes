#include "ekf.hpp"
#include <cmath>

// ─────────────────────────────────────────────────────────────────────────────
//  Static matrix helpers
//  All operations use fixed sizes matching the 6-state EKF.
// ─────────────────────────────────────────────────────────────────────────────

static void mat6_identity(float out[6][6]) {
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            out[i][j] = (i == j) ? 1.0f : 0.0f;
}

// C = A * B  (6x6 * 6x6)
static void mat6_mul(const float A[6][6], const float B[6][6], float C[6][6]) {
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++) {
            C[i][j] = 0.0f;
            for (int k = 0; k < 6; k++)
                C[i][j] += A[i][k] * B[k][j];
        }
}

// out = A^T  (6x6)
static void mat6_transpose(const float A[6][6], float out[6][6]) {
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            out[i][j] = A[j][i];
}

// Invert 2x2 matrix analytically. Returns false if near-singular.
static bool mat2_inv(const float A[2][2], float out[2][2]) {
    float det = A[0][0]*A[1][1] - A[0][1]*A[1][0];
    if (fabsf(det) < 1.0e-12f) return false;
    float inv_det = 1.0f / det;
    out[0][0] =  A[1][1] * inv_det;
    out[0][1] = -A[0][1] * inv_det;
    out[1][0] = -A[1][0] * inv_det;
    out[1][1] =  A[0][0] * inv_det;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  kalman_update_2meas
//  Shared 2-measurement Kalman update (Joseph form) used by both
//  boost_update_accel and coast_update_accel.
//
//  H[2x6], R[2x2] (diagonal measurement noise), inn[2] (innovation = z - h)
// ─────────────────────────────────────────────────────────────────────────────
static void kalman_update_2meas(EKFState& s, const float H[2][6],
                                const float R[2][2], const float inn[2]) {
    // S = H * P * H^T + R  (2x2)
    // Step 1: HP = H[2x6] * P[6x6] → [2x6]
    float HP[2][6] = {};
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 6; j++)
            for (int k = 0; k < 6; k++)
                HP[i][j] += H[i][k] * s.P[k][j];

    // Step 2: S = HP * H^T + R  (H^T[k][j] = H[j][k])
    float S[2][2] = {};
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 2; j++) {
            for (int k = 0; k < 6; k++)
                S[i][j] += HP[i][k] * H[j][k];
            S[i][j] += R[i][j];
        }

    float S_inv[2][2];
    if (!mat2_inv(S, S_inv)) return;

    // PHT = P[6x6] * H^T[6x2] → [6x2]   (PHT[i][j] = sum_k P[i][k] * H[j][k])
    float PHT[6][2] = {};
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 2; j++)
            for (int k = 0; k < 6; k++)
                PHT[i][j] += s.P[i][k] * H[j][k];

    // K = PHT * S_inv  → [6x2]
    float K[6][2] = {};
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 2; j++)
            for (int k = 0; k < 2; k++)
                K[i][j] += PHT[i][k] * S_inv[k][j];

    // State update: x += K * inn
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 2; j++)
            s.x[i] += K[i][j] * inn[j];

    // Covariance — Joseph form: P = (I - K*H) * P * (I - K*H)^T + K*R*K^T
    float IKH[6][6];
    mat6_identity(IKH);
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            for (int k = 0; k < 2; k++)
                IKH[i][j] -= K[i][k] * H[k][j];

    float IKHT[6][6], IKH_P[6][6], IKHPIKHT[6][6];
    mat6_transpose(IKH, IKHT);
    mat6_mul(IKH, s.P, IKH_P);
    mat6_mul(IKH_P, IKHT, IKHPIKHT);

    // KR = K[6x2] * R[2x2] → [6x2]
    float KR[6][2] = {};
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 2; j++)
            for (int k = 0; k < 2; k++)
                KR[i][j] += K[i][k] * R[k][j];

    // KRKT = KR[6x2] * K^T[2x6] → [6x6]   (K^T[k][j] = K[j][k])
    float KRKT[6][6] = {};
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            for (int k = 0; k < 2; k++)
                KRKT[i][j] += KR[i][k] * K[j][k];

    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            s.P[i][j] = IKHPIKHT[i][j] + KRKT[i][j];
}

// ─────────────────────────────────────────────────────────────────────────────
//  update_baro_6state
//  Shared 1-measurement baro update used by both boost and coast.
//  H = [0, 0, 1, 0, 0, 0]  →  measures vertical position x[2]
//  Port of ekf_update_baro.m (extended to 6-state)
// ─────────────────────────────────────────────────────────────────────────────
static void update_baro_6state(EKFState& s, float z_baro, float R_baro) {
    const float innov = z_baro - s.x[2];

    // S = H*P*H^T + R = P[2][2] + R_baro  (H selects row/col index 2)
    const float S = s.P[2][2] + R_baro;
    if (fabsf(S) < 1.0e-12f) return;

    // K[i] = P[i][2] / S
    float K[6];
    for (int i = 0; i < 6; i++) K[i] = s.P[i][2] / S;

    // State update
    for (int i = 0; i < 6; i++) s.x[i] += K[i] * innov;

    // Covariance — Joseph form
    // IKH = I - K*H  where H = e_2  →  IKH[i][2] -= K[i], rest unchanged
    float IKH[6][6];
    mat6_identity(IKH);
    for (int i = 0; i < 6; i++) IKH[i][2] -= K[i];

    float IKHT[6][6], IKH_P[6][6], tmp[6][6];
    mat6_transpose(IKH, IKHT);
    mat6_mul(IKH, s.P, IKH_P);
    mat6_mul(IKH_P, IKHT, tmp);

    // + K * R_baro * K^T  (rank-1 outer product)
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            s.P[i][j] = tmp[i][j] + K[i] * R_baro * K[j];
}

// ─────────────────────────────────────────────────────────────────────────────
//  stdatmo
//  1976 Standard Atmosphere — air density (kg/m^3) at geopotential altitude.
//  Port of stdatmo.m (troposphere through mesopause, 0–86000 m).
// ─────────────────────────────────────────────────────────────────────────────
float stdatmo(float h_m) {
    // Layer table: base altitude (m), lapse rate (K/m), base temp (K), base pressure (Pa)
    static const float H_base[8] = { 0.0f, 11000.0f, 20000.0f, 32000.0f,
                                      47000.0f, 51000.0f, 71000.0f, 84852.0f };
    static const float T_base[8] = { 288.15f, 216.65f, 216.65f, 228.65f,
                                      270.65f, 270.65f, 214.65f, 186.94f };
    static const float P_base[8] = { 101325.0f, 22632.04f,  5474.877f, 868.016f,
                                        110.906f,    66.939f,     3.956f,   0.373f };
    static const float L_rate[8] = { -0.0065f, 0.0f,  0.001f,  0.0028f,
                                       0.0f,  -0.0028f, -0.002f, 0.0f };

    const float R  = 287.05287f;   // J/(kg*K)
    const float g0 = 9.80665f;     // m/s^2

    if (h_m < 0.0f)     h_m = 0.0f;
    if (h_m > 86000.0f) h_m = 86000.0f;

    // Find atmosphere layer
    int layer = 0;
    for (int i = 1; i < 8; i++) {
        if (h_m >= H_base[i]) layer = i;
        else break;
    }

    const float dH = h_m - H_base[layer];
    float T, P;

    if (fabsf(L_rate[layer]) < 1.0e-10f) {
        // Isothermal layer
        T = T_base[layer];
        P = P_base[layer] * expf(-g0 * dH / (R * T));
    } else {
        // Gradient layer
        T = T_base[layer] + L_rate[layer] * dH;
        P = P_base[layer] * powf(T / T_base[layer], -g0 / (L_rate[layer] * R));
    }

    return P / (R * T);
}

// ─────────────────────────────────────────────────────────────────────────────
//  boost_predict
//  Port of ekf_predict_boost.m
//  State: x = [x, xdot, y, ydot, Tx, Ty]
// ─────────────────────────────────────────────────────────────────────────────
void boost_predict(EKFState& s, float dt, float k, float m, float g,
                   const float Q[6]) {
    const float x_dot = s.x[1];
    const float y_dot = s.x[3];
    const float Tx    = s.x[4];
    const float Ty    = s.x[5];
    const float km    = k / m;
    const float V     = sqrtf(x_dot*x_dot + y_dot*y_dot);
    const float beta  = atan2f(x_dot, y_dot + EKF_EP);
    const float sb    = sinf(beta);
    const float cb    = cosf(beta);

    // Nonlinear state propagation: x_pred = x + f(x)*dt
    const float f[6] = {
        x_dot,
        -km * V * x_dot + Tx,
        y_dot,
        -km * V * y_dot - g + Ty,
        0.0f,   // Tx modeled as constant between steps
        0.0f    // Ty modeled as constant between steps
    };
    for (int i = 0; i < 6; i++) s.x[i] += f[i] * dt;

    // Continuous-time Jacobian A (6x6) — derived in GNC work log
    float A[6][6] = {};
    A[0][1] = 1.0f;
    A[1][1] = -km * V * (1.0f + sb*sb);
    A[1][3] = -km * 0.5f * V * sinf(2.0f * beta);
    A[1][4] = 1.0f;
    A[2][3] = 1.0f;
    A[3][1] = -km * 0.5f * V * sinf(2.0f * beta);
    A[3][3] = -km * V * (1.0f + cb*cb);
    A[3][5] = 1.0f;

    // Fd = I + A*dt  (first-order discretization)
    float Fd[6][6];
    mat6_identity(Fd);
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            Fd[i][j] += A[i][j] * dt;

    // P = Fd * P * Fd^T + Q  (Q is diagonal)
    float FdT[6][6], FdP[6][6], tmp[6][6];
    mat6_transpose(Fd, FdT);
    mat6_mul(Fd, s.P, FdP);
    mat6_mul(FdP, FdT, tmp);
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            s.P[i][j] = tmp[i][j] + ((i == j) ? Q[i] : 0.0f);
}

// ─────────────────────────────────────────────────────────────────────────────
//  boost_update_baro
//  Port of ekf_update_baro.m (6-state)
// ─────────────────────────────────────────────────────────────────────────────
void boost_update_baro(EKFState& s, float z_baro, float R_baro) {
    update_baro_6state(s, z_baro, R_baro);
}

// ─────────────────────────────────────────────────────────────────────────────
//  boost_update_accel
//  Port of ekf_update_accel.m
//  z_accel[0] = lateral magnitude (m/s^2),  z_accel[1] = body-z axial (m/s^2)
// ─────────────────────────────────────────────────────────────────────────────
void boost_update_accel(EKFState& s, const float z_accel[2], float R_accel,
                        float k, float m, float g, float lat_scale) {
    (void)g;  // accelerometers measure specific force; gravity drops out of h(x)

    const float x_dot = s.x[1];
    const float y_dot = s.x[3];
    const float Tx    = s.x[4];
    const float Ty    = s.x[5];
    const float km    = k / m;
    const float V     = sqrtf(x_dot*x_dot + y_dot*y_dot);
    const float V2    = x_dot*x_dot + y_dot*y_dot;
    const float beta  = atan2f(x_dot, y_dot + EKF_EP);
    const float sb    = sinf(beta);
    const float cb    = cosf(beta);

    // Predicted specific force in inertial frame
    const float sf_x = -km * V * x_dot + Tx;
    const float sf_y = -km * V * y_dot + Ty;

    // Rotate to body frame
    const float h1 = sf_x * cb - sf_y * sb;  // lateral
    const float h2 = sf_x * sb + sf_y * cb;  // axial

    const float h[2] = { fabsf(h1), h2 };

    // Analytical Jacobian H (2x6) — from ekf_update_accel.m Section 3
    float H[2][6] = {};

    if (V > 1.0e-6f) {
        const float TdotN = Tx * sb + Ty * cb;  // used in lateral partials
        const float TdotA = Tx * cb - Ty * sb;  // used in axial partials

        H[0][1] =  km * (-2.0f*cb*x_dot*x_dot + 2.0f*sb*x_dot*y_dot) / V
                   - (y_dot / V2) * TdotN;
        H[0][3] =  km * (2.0f*y_dot*y_dot*sb - 2.0f*x_dot*y_dot*cb) / V
                   + (x_dot / V2) * TdotN;
        H[1][1] = -km * (2.0f*x_dot*x_dot*sb + 2.0f*x_dot*y_dot*cb) / V
                   + (y_dot / V2) * TdotA;
        H[1][3] = -km * (2.0f*x_dot*y_dot*sb + 2.0f*y_dot*y_dot*cb) / V
                   - (x_dot / V2) * TdotA;
    }
    H[0][4] =  cb;   H[0][5] = -sb;   // ∂h1/∂Tx, ∂h1/∂Ty
    H[1][4] =  sb;   H[1][5] =  cb;   // ∂h2/∂Tx, ∂h2/∂Ty

    // Sign correction — keeps Jacobian consistent with abs() in measurement model
    if (h1 < 0.0f)
        for (int j = 0; j < 6; j++) H[0][j] = -H[0][j];

    const float inn[2] = { z_accel[0] - h[0], z_accel[1] - h[1] };
    const float R[2][2] = {{ lat_scale * R_accel, 0.0f },
                            { 0.0f,               R_accel }};

    kalman_update_2meas(s, H, R, inn);
}

// ─────────────────────────────────────────────────────────────────────────────
//  coast_predict
//  Port of coast_predict() from EKF_combined_main.m, extended to 6-state.
//  Dynamics: drag + gravity only; Tx/Ty rows/cols are zero in A.
// ─────────────────────────────────────────────────────────────────────────────
void coast_predict(EKFState& s, float dt, float k, float m, float g,
                   const float Q[6]) {
    const float x_dot = s.x[1];
    const float y_dot = s.x[3];
    const float km    = k / m;
    const float V     = sqrtf(x_dot*x_dot + y_dot*y_dot);
    const float beta  = atan2f(x_dot, y_dot + EKF_EP);
    const float sb    = sinf(beta);
    const float cb    = cosf(beta);

    // Nonlinear propagation (drag + gravity, no thrust)
    const float f[6] = {
        x_dot,
        -km * V * x_dot,
        y_dot,
        -km * V * y_dot - g,
        0.0f,
        0.0f
    };
    for (int i = 0; i < 6; i++) s.x[i] += f[i] * dt;

    // Continuous-time Jacobian A (6x6, rows/cols 4-5 are zero — no thrust)
    float A[6][6] = {};
    A[0][1] = 1.0f;
    A[1][1] = -km * V * (1.0f + sb*sb);
    A[1][3] = -km * 0.5f * V * sinf(2.0f * beta);
    A[2][3] = 1.0f;
    A[3][1] = -km * 0.5f * V * sinf(2.0f * beta);
    A[3][3] = -km * V * (1.0f + cb*cb);

    // Fd = I + A*dt
    float Fd[6][6];
    mat6_identity(Fd);
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            Fd[i][j] += A[i][j] * dt;

    // P = Fd * P * Fd^T + Q
    float FdT[6][6], FdP[6][6], tmp[6][6];
    mat6_transpose(Fd, FdT);
    mat6_mul(Fd, s.P, FdP);
    mat6_mul(FdP, FdT, tmp);
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            s.P[i][j] = tmp[i][j] + ((i == j) ? Q[i] : 0.0f);
}

// ─────────────────────────────────────────────────────────────────────────────
//  coast_update_baro
//  Port of coast_update_baro() from EKF_combined_main.m (6-state)
// ─────────────────────────────────────────────────────────────────────────────
void coast_update_baro(EKFState& s, float z_baro, float R_baro) {
    update_baro_6state(s, z_baro, R_baro);
}

// ─────────────────────────────────────────────────────────────────────────────
//  coast_accel_h
//  Port of coast_accel_h() from EKF_combined_main.m
//  Predicted accelerometer reading from coast state (drag only, no thrust).
//  h_out[0] = abs(body_x lateral),  h_out[1] = body_z axial
// ─────────────────────────────────────────────────────────────────────────────
void coast_accel_h(const float x[6], float k, float m, float ep, float h_out[2]) {
    const float x_dot = x[1];
    const float y_dot = x[3];
    const float km    = k / m;
    const float V     = sqrtf(x_dot*x_dot + y_dot*y_dot);
    const float beta  = atan2f(x_dot, y_dot + ep);
    const float sb    = sinf(beta);
    const float cb    = cosf(beta);

    // Specific force in inertial frame (drag only, no thrust)
    const float sf_x = -km * V * x_dot;
    const float sf_y = -km * V * y_dot;

    // Rotate to body frame
    const float body_x = sf_x * cb - sf_y * sb;
    const float body_z = sf_x * sb + sf_y * cb;

    h_out[0] = fabsf(body_x);
    h_out[1] = body_z;
}

// ─────────────────────────────────────────────────────────────────────────────
//  coast_update_accel
//  Port of coast_update_accel() from EKF_combined_main.m, extended to 6-state.
//  Uses numerical Jacobian via coast_accel_h (columns 4-5 will be ~zero
//  since coast_accel_h does not depend on Tx/Ty).
// ─────────────────────────────────────────────────────────────────────────────
void coast_update_accel(EKFState& s, const float z_accel[2], float R_accel,
                        float k, float m, float g, float lat_scale) {
    (void)g;  // accelerometers measure specific force; gravity drops out of h(x)

    const float ep    = EKF_EP;
    const float delta = 1.0e-6f;  // finite-difference step

    float h[2];
    coast_accel_h(s.x, k, m, ep, h);

    // Numerical Jacobian H (2x6)
    float H[2][6] = {};
    for (int j = 0; j < 6; j++) {
        float x_p[6];
        for (int i = 0; i < 6; i++) x_p[i] = s.x[i];
        x_p[j] += delta;

        float h_p[2];
        coast_accel_h(x_p, k, m, ep, h_p);

        H[0][j] = (h_p[0] - h[0]) / delta;
        H[1][j] = (h_p[1] - h[1]) / delta;
    }

    // Sign correction for abs(body_x) — mirrors coast_update_accel in MATLAB
    const float x_dot      = s.x[1];
    const float y_dot      = s.x[3];
    const float V          = sqrtf(x_dot*x_dot + y_dot*y_dot);
    const float beta       = atan2f(x_dot, y_dot + ep);
    const float km         = k / m;
    const float sf_x       = -km * V * x_dot;
    const float sf_y       = -km * V * y_dot;
    const float body_x_raw = sf_x * cosf(beta) - sf_y * sinf(beta);

    if (body_x_raw < 0.0f)
        for (int j = 0; j < 6; j++) H[0][j] = -H[0][j];

    const float inn[2] = { z_accel[0] - h[0], z_accel[1] - h[1] };
    const float R[2][2] = {{ lat_scale * R_accel, 0.0f },
                            { 0.0f,               R_accel }};

    kalman_update_2meas(s, H, R, inn);
}
