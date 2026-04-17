#pragma once

#include <cmath>




struct MPCOutput {
    double Cd_cmd;
    double Cd_limited;
    double deploy_mm;
    double apogee_pred;
};

class MPCController {
public:
    MPCController();
    MPCOutput update(const MPCState& state);

private:
    double Cd_prev_;

    // Parameters
    static constexpr double MASS         = 29.418;
    static constexpr double CD_CLEAN     = 0.55;
    static constexpr double CD_MAX       = 0.83;
    static constexpr double SREF         = 0.019348;   // pi*(0.157/2)^2
    static constexpr double APOGEE_TGT   = 3048.0;
    static constexpr double DCd_MAX      = 0.05;
    static constexpr double MAX_DEPLOY_MM = 50.0;
};