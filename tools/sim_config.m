function [imu_config, baro_config, constants] = sim_config()
%% sim_config  Shared sensor configuration for simulation scripts.
%%
%% Returns structs matching the firmware IMUConfig / BarometerConfig layouts
%% (imu.hpp, barometer.hpp) plus a derived constants struct.
%%
%% Hardware:
%%   IMU:        LSM6DSV16X (x4) — ST 6-axis accel/gyro
%%   Barometer:  BMP581     (x2) — Bosch absolute pressure sensor

%% ══════════════════════════════════════════════════════════════════════════
%%  IMU CONFIGURATION  (matches IMUConfig struct in imu.hpp)
%%
%%  LSM6DSV16X datasheet (DS13510 Rev 4) noise specs:
%%    Accel noise density (high-perf mode): 60 µg/√Hz  (Table 3, symbol An)
%%    Accel zero-g offset: ±12 mg             (Table 3, symbol LA_TyOff)
%%    Gyro noise density (high-perf mode): 2.8 mdps/√Hz  (Table 3, symbol Rn)
%%    Gyro zero-rate level: ±1 dps            (Table 3, symbol G_TyOff)
%%    Accel sensitivity at ±8g: 0.244 mg/LSB  (Table 3, symbol LA_So)
%%    Gyro sensitivity at ±500 dps: 17.50 mdps/LSB  (Table 3, symbol G_So)
%%
%%  LP filter: LP2 (accel) and LP1 (gyro) cutoff ≈ ODR/4 in high-performance mode
%% ══════════════════════════════════════════════════════════════════════════
imu_config = struct( ...
    ... % ── Hardware config ──
    'accel_range_g',        8, ...      % ±8g   (LSM6DSV16X_8g)
    'gyro_range_dps',       500, ...    % ±500 dps (LSM6DSV16X_500dps)
    'accel_odr_hz',         240, ...    % LSM6DSV16X_ODR_AT_240Hz
    'gyro_odr_hz',          240, ...
    'accel_lp2_enable',     true, ...   % LP2 filter on accel output
    'gyro_lp1_enable',      true, ...   % LP1 filter on gyro output
    ...
    ... % ── Noise model ──
    'accel_noise_density',  60e-6, ...  % g/√Hz, high-performance mode
    'accel_bias_mg',        12, ...     % mg, zero-g offset (after calibration)
    'gyro_noise_density',   2.8e-3, ... % dps/√Hz, high-performance mode
    'gyro_bias_dps',        1, ...      % dps, zero-rate level
    ...
    ... % ── Quantization ──
    'accel_lsb_mg',         0.244, ...  % mg/LSB at ±8g
    'gyro_lsb_mdps',        17.50, ... % mdps/LSB at ±500 dps
    ...
    ... % ── Feature toggles ──
    'enable_noise',         true, ...
    'enable_bias',          true, ...
    'enable_lp_filter',     true, ...   % cutoff = ODR/4 (LP2/LP1 typical)
    'enable_quantization',  true, ...
    'deterministic_noise',  true ...    % fixed RNG seed per channel
);

%% ══════════════════════════════════════════════════════════════════════════
%%  BAROMETER CONFIGURATION  (matches BarometerConfig struct in barometer.hpp)
%%
%%  BMP581 noise: ~1.0 Pa RMS (see BMP581 datasheet)
%% ══════════════════════════════════════════════════════════════════════════
baro_config = struct( ...
    ... % ── Hardware config ──
    'pressure_osr',         8, ...      % 8x oversampling (BMP5_OVERSAMPLING_8X)
    'odr_hz',               50, ...     % 50 Hz (BMP5_ODR_50_HZ)
    'iir_coeff',            3, ...      % IIR coefficient (0=off, 1,3,7,15,31,63,127)
    'sea_level_hpa',        1013.25, ...
    ...
    ... % ── Noise model ──
    'pressure_noise_pa',    1.0, ...    % Pa RMS (native, before OSR)
    'pressure_bias_pa',     8, ...      % Pa, systematic offset per power cycle
    ...
    ... % ── Feature toggles ──
    'enable_noise',         true, ...
    'enable_bias',          true, ...
    'enable_iir',           true, ...
    'enable_quantization',  true, ...
    'deterministic_noise',  true ...
);

%% ══════════════════════════════════════════════════════════════════════════
%%  DERIVED CONSTANTS
%% ══════════════════════════════════════════════════════════════════════════
g = 9.80665;  % m/s^2

accel_odr        = imu_config.accel_odr_hz;
gyro_odr         = imu_config.gyro_odr_hz;
baro_odr         = baro_config.odr_hz;

accel_nd         = imu_config.accel_noise_density;    % g/√Hz
gyro_nd          = imu_config.gyro_noise_density;      % dps/√Hz

accel_lsb_g      = imu_config.accel_lsb_mg / 1000;    % g/LSB
gyro_lsb_dps     = imu_config.gyro_lsb_mdps / 1000;   % dps/LSB
accel_sensitivity = 1 / accel_lsb_g;                  % LSB/g
gyro_sensitivity  = 1 / gyro_lsb_dps;                 % LSB/dps
accel_fsr_g      = imu_config.accel_range_g;
gyro_fsr_dps     = imu_config.gyro_range_dps;

baro_iir_coeff   = baro_config.iir_coeff;
baro_osr_n       = baro_config.pressure_osr;

constants = struct( ...
    'g',                g, ...
    'accel_odr',        accel_odr, ...
    'gyro_odr',         gyro_odr, ...
    'baro_odr',         baro_odr, ...
    'accel_nd',         accel_nd, ...
    'gyro_nd',          gyro_nd, ...
    'accel_lsb_g',      accel_lsb_g, ...
    'gyro_lsb_dps',     gyro_lsb_dps, ...
    'accel_sensitivity', accel_sensitivity, ...
    'accel_fsr_g',      accel_fsr_g, ...
    'gyro_sensitivity', gyro_sensitivity, ...
    'gyro_fsr_dps',     gyro_fsr_dps, ...
    'baro_iir_coeff',   baro_iir_coeff, ...
    'baro_osr_n',       baro_osr_n ...
);

end
