


class apogee_predictor {
    public:
        float sim_traj(double Cd_sim);
        float  MPCloop();
        float slewRateLimit(float desiredCd, float currentCd);
    private:
        static constexpr float Cd_clean = 0.56;
        static constexpr float Cd_max   = 0.9;
        static constexpr int max_steps = 5000;
        static constexpr float RHO0 = 1.225; // kg/m^3
        static constexpr float H_SCALE = 8500.0; // m
        static constexpr float G = 9.81; // m/s^2
        static constexpr float mass = 16.54; // kg
        static constexpr float Sref = 0.019348; // m^2
        static constexpr float DT = 0.01; // s
        static constexpr float DCd_MAX      = 0.05; // Max change in Cd per control step

};