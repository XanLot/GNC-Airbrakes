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
}