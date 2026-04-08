#include "controller.h"
#include "apogee_predictor.h"
#include <algorithm>
#include <cmath>

MPCController::MPCController() : Cd_prev_(CD_CLEAN) {}

MPCOutput MPCController::update(const MPCState& state) {

    // Call apogee predictor
    ApogeePrediction pred = predictApogee(
        state.acceleration,
        state.velocity,
        state.altitude,
        APOGEE_TGT,
        SREF,
        CD_CLEAN,
        CD_MAX,
        MASS,
        state.timestamp
    );

    // Slew rate limiter
    double dCd  = pred.Cd_cmd - Cd_prev_;
    dCd = std::max(-DCd_MAX, std::min(DCd_MAX, dCd));
    double Cd_limited = Cd_prev_ + dCd;
    Cd_limited  = std::max(CD_CLEAN, std::min(CD_MAX, Cd_limited));

    // Actuator model - sliding deployment
    double deploy_mm = (Cd_limited - CD_CLEAN) / (CD_MAX - CD_CLEAN) * MAX_DEPLOY_MM;
    deploy_mm = std::max(0.0, std::min(MAX_DEPLOY_MM, deploy_mm));

    Cd_prev_ = Cd_limited;

    return {pred.Cd_cmd, Cd_limited, deploy_mm, pred.apogee_alt};
}

static const double RHO0    = 1.225;
static const double H_SCALE = 8500.0;
static const double G       = 9.81;
static const double DT      = 0.01;
static const int    MAX_STEPS = 5000;

static double simTrajectory(
    double v0, double alt0, double t0,
    double Cd_sim, double Sref, double mass,
    double& t_apogee_out)
{
    double v   = v0;
    double alt = alt0;
    double t   = t0;

    for (int i = 0; i < MAX_STEPS; i++) {
        double rho    = RHO0 * std::exp(-alt / H_SCALE);
        double F_drag = -0.5 * rho * Cd_sim * Sref * v * std::abs(v);
        double a      = (F_drag / mass) - G;
        v   += a * DT;
        alt += v * DT;
        t   += DT;
        if (v <= 0.0) {
            t_apogee_out = t;
            return alt;
        }
    }
    return std::numeric_limits<double>::quiet_NaN();
}

ApogeePrediction predictApogee(
    double ay, double vy, double alt,
    double tgt_alt, double Sref,
    double Cd_clean, double Cd_max,
    double mass, double currentTime)
{
    ApogeePrediction result = {Cd_clean, std::numeric_limits<double>::quiet_NaN(), 
                                std::numeric_limits<double>::quiet_NaN()};

    // Early exit: boost phase
    if (ay > -2.0) return result;

    double t_ap = 0.0;

    // Baseline check
    double apogee_clean = simTrajectory(vy, alt, currentTime, Cd_clean, Sref, mass, t_ap);
    if (!std::isnan(apogee_clean) && apogee_clean <= tgt_alt) {
        result = {Cd_clean, apogee_clean, t_ap};
        return result;
    }

    // Bisection search
    double Cd_low  = Cd_clean;
    double Cd_high = Cd_max;
    double Cd_mid  = Cd_clean;
    double apogee_mid = std::numeric_limits<double>::quiet_NaN();

    for (int iter = 0; iter < 20; iter++) {
        Cd_mid     = (Cd_low + Cd_high) / 2.0;
        apogee_mid = simTrajectory(vy, alt, currentTime, Cd_mid, Sref, mass, t_ap);

        if (std::isnan(apogee_mid)) break;
        if (std::abs(apogee_mid - tgt_alt) < 5.0) break;

        if (apogee_mid > tgt_alt)
            Cd_low  = Cd_mid;
        else
            Cd_high = Cd_mid;

        if ((Cd_high - Cd_low) < 0.001) break;
    }

    double Cd_cmd = std::max(Cd_clean, std::min(Cd_max, Cd_mid));
    result = {Cd_cmd, apogee_mid, t_ap};
    return result;
}