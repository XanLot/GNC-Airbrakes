#include "apogee_predictor.hpp"
#include <cmath>

float apogee_predictor::sim_traj(double Cd_sim){

    // need current velocity, altitude, and time to run the sim

    float apogee_altitude = 0.0f;
    float t_apogee = 0.0f;

    for (int i = 0; i < max_steps; i++) {
        double rho    = RHO0 * std::exp(-alt / H_SCALE);
        double F_drag = -0.5 * rho * Cd_sim * Sref * v * std::abs(v);
        double a = (F_drag / mass) - G;
        v   += a * DT;
        alt += v * DT;
        t   += DT;
        if (v <= 0.0) {
            t_apogee = t;
            apogee_altitude = alt;
            break;
        }
    }
    return static_cast<float>(apogee_altitude);
}

float apogee_predictor::MPCloop() {
    // Placeholder for MPC loop implementation
    // In a real implementation, this would read sensor data, call predictApogee(),
    // solve the optimization problem, and return the desired control output.

    float Cd_low = Cd_clean;
    float Cd_high = Cd_max;
    float Cd_mid = Cd_clean;
    float apogee_mid = 0.0f;

    for (int i = 0; i < 20; i++) {
        Cd_mid = (Cd_low + Cd_high) / 2.0;
        apogee_mid = apogee_predictor::sim_traj(Cd_mid);

        if (apogee_mid > APOGEE_TGT) {
            Cd_low = Cd_mid;
        } else {
            Cd_high = Cd_mid;
        }

        if (Cd_high - Cd_low < 0.001) {
            break;
        }
    }

    float apogee_alt = apogee_mid;
    float Cd_cmd = std::max(std::min(Cd_mid, Cd_max), Cd_clean);

    return Cd_cmd;
}

float apogee_predictor::slewRateLimit(float currentCd, float desiredCd) {
    float delta = desiredCd - currentCd;
    if (std::abs(delta) >  DCd_MAX) {
        delta = (delta > 0) ? DCd_MAX : - DCd_MAX;
    }
    return currentCd + delta;
}