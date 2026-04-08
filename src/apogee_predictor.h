#pragma once

struct ApogeePrediction {
    double Cd_cmd;
    double apogee_alt;
    double t_apogee;
};

ApogeePrediction predictApogee(
    double ay,
    double vy,
    double alt,
    double tgt_alt,
    double Sref,
    double Cd_clean,
    double Cd_max,
    double mass,
    double currentTime
);