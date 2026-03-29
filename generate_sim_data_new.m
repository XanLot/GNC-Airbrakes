%% generate_sim_data.m
%% Generates SIM.BIN for SD card from an OpenRocket CSV export.
%%
%% Pipeline:
%%   1. Load OpenRocket CSV
%%   2. Compute true body-frame accelerometer readings (specific force)
%%   3. Compute true barometer pressure readings
%%   4. Add bias (if enabled)
%%   5. Add white noise (if enabled)
%%   6. Apply DLPF / IIR filters (if enabled)
%%   7. Quantize to ADC resolution (if enabled)
%%   8. Downsample to sensor ODR
%%   9. Write binary SIM.BIN for SD card
%%
%% Usage:
%%   1. Export OpenRocket CSV with required columns (see below)
%%   2. Configure sensor settings to match firmware IMUConfig / BarometerConfig
%%   3. Run this script
%%
%% Required OpenRocket CSV columns (in this order):
%%   Time [s], Altitude [m], Vertical velocity [m/s],
%%   Vertical acceleration [m/s^2], Lateral distance [m],
%%   Lateral velocity [m/s], Lateral acceleration [m/s^2],
%%   Vertical orientation (zenith) [rad], Mass [kg], Thrust [N],
%%   Drag force [N], Drag coefficient, Air pressure [Pa],
%%   Air density [kg/m^3], Reference area [m^2]
%%
%% Hardware:
%%   IMU:         LSM6DSVTR (x4)  — ST 6-axis accel/gyro
%%   Barometer:   BMP581 (x2)     — Bosch absolute pressure sensor
%%   Magnetometer: MMC5983MA      — (placeholder, not yet modeled)
%%   Temperature:  TMP117 (x2)    — (placeholder, not yet modeled)

clear;
clc; 
close all;

%% ══════════════════════════════════════════════════════════════════════════
%%  FILE PATHS
%% ══════════════════════════════════════════════════════════════════════════
openrocket_csv = 'CompSim.csv';   % Path to OpenRocket export
output_file    = 'SIM.BIN';  % Binary file — copy to SD card root before running sim

%% ══════════════════════════════════════════════════════════════════════════
%%  LOAD SHARED CONFIGURATION
%% ══════════════════════════════════════════════════════════════════════════
[imu_config, baro_config, constants] = sim_config();

% Unpack derived parameters into local variables
g                = constants.g;
accel_odr        = constants.accel_odr;
gyro_odr         = constants.gyro_odr;
baro_odr         = constants.baro_odr;
accel_nd         = constants.accel_nd;
gyro_nd          = constants.gyro_nd;
accel_sensitivity = constants.accel_sensitivity;
accel_fsr_g      = constants.accel_fsr_g;
gyro_sensitivity = constants.gyro_sensitivity;
gyro_fsr_dps     = constants.gyro_fsr_dps;
baro_iir_coeff   = constants.baro_iir_coeff;
baro_osr_n       = constants.baro_osr_n;

% Print config summary
fprintf('═══ Sensor Configuration Summary ═══\n');
fprintf('IMU:   LSM6DSVTR (x4)\n');
fprintf('Accel: ±%dg, ODR=%d Hz, noise density=%.0f µg/√Hz\n', ...
        accel_fsr_g, accel_odr, accel_nd * 1e6);
fprintf('Gyro:  ±%d dps, ODR=%d Hz, noise density=%.1f mdps/√Hz\n', ...
        gyro_fsr_dps, gyro_odr, gyro_nd * 1000);
fprintf('Baro:  BMP581 (x2), ODR=%d Hz, OSR=%dx, IIR coeff=%d\n', ...
        baro_odr, baro_osr_n, baro_iir_coeff);
fprintf('       Pressure noise: %.3f Pa RMS (native)\n', baro_config.pressure_noise_pa);
fprintf('══════════════════════════════════════\n\n');

%% ══════════════════════════════════════════════════════════════════════════
%%  STEP 1: LOAD OPENROCKET CSV
%% ══════════════════════════════════════════════════════════════════════════
fprintf('Loading OpenRocket data from: %s\n', openrocket_csv);

% Read the CSV — skip comment lines starting with '#'
raw_data = readmatrix(openrocket_csv, 'NumHeaderLines', 0, 'CommentStyle', '#');

% Remove any rows with NaN in the time column
valid = ~isnan(raw_data(:,1));
raw_data = raw_data(valid, :);

fprintf('  Found %d columns, %d valid rows\n', size(raw_data, 2), size(raw_data, 1));

% Map columns by index
% Order: Time, Altitude, Vert vel, Vert accel, Lat dist, Lat vel, Lat accel,
%        Zenith, Mass, Thrust, Drag force, Drag coeff, Air pressure,
%        Air density, Ref area
or_time       = raw_data(:,1);   % s
or_altitude   = raw_data(:,2);   % m
or_vy         = raw_data(:,3);   % m/s   vertical velocity
or_ay         = raw_data(:,4);   % m/s^2 vertical acceleration
or_lat_dist   = raw_data(:,5);   % m     lateral distance
or_vx         = raw_data(:,6);   % m/s   lateral velocity
or_ax         = raw_data(:,7);   % m/s^2 lateral acceleration
or_zenith     = raw_data(:,8);   % rad   vertical orientation (zenith)
or_zenith = pi/2 - or_zenith;    % Convert from OR convention to β measured from vertical
or_mass       = raw_data(:,9);   % kg
or_thrust     = raw_data(:,10);  % N
or_drag       = raw_data(:,11);  % N
or_cd         = raw_data(:,12);  % drag coefficient
or_pressure   = raw_data(:,13);  % Pa    air pressure
or_density    = raw_data(:,14);  % kg/m^3 air density
or_ref_area   = raw_data(:,15);  % m^2   reference area

N_or = length(or_time);
dt_or = median(diff(or_time));  % OpenRocket time step
fs_or = 1 / dt_or;             % OpenRocket effective sample rate

fprintf('  Duration: %.2f s, OR sample rate: ~%.1f Hz\n', ...
        or_time(end), fs_or);

%% ══════════════════════════════════════════════════════════════════════════
%%  STEP 2: COMPUTE TRUE SENSOR READINGS AT HIGH RATE
%% ══════════════════════════════════════════════════════════════════════════
% We need to compute what the sensors would truly read (before noise).
%
% The accelerometer measures SPECIFIC FORCE in the body frame.
% Specific force = (total_force - gravity) / mass, measured in body frame.
%
% In the inertial frame:
%   a_inertial = [ax_lateral; ay_vertical]   (from OpenRocket)
%
% Specific force in inertial frame (what a perfect accel would read if
% it were in the inertial frame):
%   sf_inertial = a_inertial - [0; -g]  =  [ax; ay + g]
%
% Wait — OpenRocket's "acceleration" includes gravity (it's coordinate
% acceleration d²r/dt²). So:
%   sf = a_coord - g_vector
%   sf_x = ax_lateral
%   sf_y = ay_vertical + g       (because g_vector = [0; -g])
%
% Then rotate into body frame using zenith angle β:
%   The body z-axis points along the nose (up when vertical).
%   β = zenith angle = angle between body z-axis and vertical.
%
%   Body frame:
%     a_body_z (along nose) =  sf_x * sin(β) + sf_y * cos(β)
%     a_body_x (lateral)    =  sf_x * cos(β) - sf_y * sin(β)
%     a_body_y              =  0  (2D assumption)

fprintf('Computing true body-frame sensor readings...\n');

% Specific force in inertial frame
sf_x_inertial = or_ax;           % lateral specific force = lateral coord accel
sf_y_inertial = or_ay + g;       % vertical: remove gravity to get specific force

% Rotate to body frame
%   β = zenith angle (0 = pointing straight up)
%   Body z = along nose, Body x = perpendicular in the flight plane
beta = or_zenith;

% Body-frame specific force (m/s^2)
true_accel_bz = sf_x_inertial .* sin(beta) + sf_y_inertial .* cos(beta);
true_accel_bx = sf_x_inertial .* cos(beta) - sf_y_inertial .* sin(beta);
true_accel_by = zeros(N_or, 1);  % 2D assumption

% Convert to g for the accel noise model (LSM6DSV outputs in g internally)
true_accel_bz_g = true_accel_bz / g;
true_accel_bx_g = true_accel_bx / g;
true_accel_by_g = true_accel_by / g;

% True barometer pressure (directly from OpenRocket)
true_pressure = or_pressure;  % Pa

% True gyro reading — angular rate about body y-axis (pitch rate in 2D)
% Numerically differentiate zenith angle
true_gyro_y = gradient(beta, or_time);  % rad/s
true_gyro_x = zeros(N_or, 1);
true_gyro_z = zeros(N_or, 1);

% Convert gyro to dps for noise model
true_gyro_x_dps = rad2deg(true_gyro_x);
true_gyro_y_dps = rad2deg(true_gyro_y);
true_gyro_z_dps = rad2deg(true_gyro_z);

%% ══════════════════════════════════════════════════════════════════════════
%%  STEP 3: INTERPOLATE TO HIGH INTERNAL RATE
%% ══════════════════════════════════════════════════════════════════════════
% We generate data at a high internal rate (max of all sensor ODRs * 4)
% to properly model filtering before downsampling.

fs_internal = max([accel_odr, gyro_odr, baro_odr]) * 4;
fs_internal = max(fs_internal, fs_or);  % At least as fast as OpenRocket
dt_internal = 1 / fs_internal;

t_internal = (or_time(1) : dt_internal : or_time(end))';
N_internal = length(t_internal);

fprintf('Internal simulation rate: %.1f Hz (%d samples)\n', fs_internal, N_internal);

% Interpolate all truth signals to internal rate
accel_bx_g_hi = interp1(or_time, true_accel_bx_g, t_internal, 'pchip');
accel_by_g_hi = interp1(or_time, true_accel_by_g, t_internal, 'pchip');
accel_bz_g_hi = interp1(or_time, true_accel_bz_g, t_internal, 'pchip');

gyro_x_dps_hi = interp1(or_time, true_gyro_x_dps, t_internal, 'pchip');
gyro_y_dps_hi = interp1(or_time, true_gyro_y_dps, t_internal, 'pchip');
gyro_z_dps_hi = interp1(or_time, true_gyro_z_dps, t_internal, 'pchip');

pressure_hi   = interp1(or_time, true_pressure, t_internal, 'pchip');

%% ══════════════════════════════════════════════════════════════════════════
%%  STEP 4: ADD BIAS
%% ══════════════════════════════════════════════════════════════════════════
if imu_config.deterministic_noise
    rng(42, 'twister');  % Fixed seed for repeatability
end

% Accel bias: random offset drawn from ±bias_mg, constant for this run
%   LSM6DSV: ±12 mg zero-g offset (vs ±50 mg on ICM-20948)
if imu_config.enable_bias
    accel_bias = (imu_config.accel_bias_mg / 1000) * (2*rand(1,3) - 1);  % g
    gyro_bias  = imu_config.gyro_bias_dps * (2*rand(1,3) - 1);           % dps
    fprintf('Accel bias (mg): [%.2f, %.2f, %.2f]\n', accel_bias*1000);
    fprintf('Gyro bias (dps): [%.2f, %.2f, %.2f]\n', gyro_bias);
else
    accel_bias = [0, 0, 0];
    gyro_bias  = [0, 0, 0];
end

accel_bx_g_hi = accel_bx_g_hi + accel_bias(1);
accel_by_g_hi = accel_by_g_hi + accel_bias(2);
accel_bz_g_hi = accel_bz_g_hi + accel_bias(3);

gyro_x_dps_hi = gyro_x_dps_hi + gyro_bias(1);
gyro_y_dps_hi = gyro_y_dps_hi + gyro_bias(2);
gyro_z_dps_hi = gyro_z_dps_hi + gyro_bias(3);

% Baro bias
%   BMP581: absolute accuracy ±50 Pa (±0.5 hPa)
if baro_config.deterministic_noise
    rng(99, 'twister');
end

if baro_config.enable_bias
    pressure_bias = baro_config.pressure_bias_pa * (2*rand() - 1);
    fprintf('Baro pressure bias (Pa): %.2f\n', pressure_bias);
else
    pressure_bias = 0;
end

pressure_hi = pressure_hi + pressure_bias;

%% ══════════════════════════════════════════════════════════════════════════
%%  STEP 5: ADD WHITE NOISE
%% ══════════════════════════════════════════════════════════════════════════
% Noise RMS = noise_density * sqrt(noise_bandwidth)
% At the internal sample rate (before DLPF), the noise bandwidth is fs/2.
% The DLPF will then reduce this.

if imu_config.deterministic_noise
    rng(123, 'twister');
end

if imu_config.enable_noise
    % Accel: noise density is in g/√Hz
    % RMS noise at internal rate = noise_density * sqrt(fs_internal / 2)
    accel_noise_rms = accel_nd * sqrt(fs_internal / 2);  % g
    accel_bx_g_hi = accel_bx_g_hi + accel_noise_rms * randn(N_internal, 1);
    accel_by_g_hi = accel_by_g_hi + accel_noise_rms * randn(N_internal, 1);
    accel_bz_g_hi = accel_bz_g_hi + accel_noise_rms * randn(N_internal, 1);

    % Gyro: noise density is in dps/√Hz
    gyro_noise_rms = gyro_nd * sqrt(fs_internal / 2);  % dps
    gyro_x_dps_hi = gyro_x_dps_hi + gyro_noise_rms * randn(N_internal, 1);
    gyro_y_dps_hi = gyro_y_dps_hi + gyro_noise_rms * randn(N_internal, 1);
    gyro_z_dps_hi = gyro_z_dps_hi + gyro_noise_rms * randn(N_internal, 1);

    fprintf('Accel noise RMS (pre-filter): %.4f mg\n', accel_noise_rms * 1000);
    fprintf('Gyro noise RMS (pre-filter):  %.4f dps\n', gyro_noise_rms);
end

if baro_config.enable_noise
    if baro_config.deterministic_noise
        rng(456, 'twister');
    end
    % BMP581 noise, reduced by oversampling: noise / sqrt(osr_samples)
    effective_pressure_noise = baro_config.pressure_noise_pa / sqrt(baro_osr_n);
    pressure_hi = pressure_hi + effective_pressure_noise * randn(N_internal, 1);
    fprintf('Baro noise RMS (after OSR): %.6f Pa\n', effective_pressure_noise);
end

%% ══════════════════════════════════════════════════════════════════════════
%%  STEP 6: APPLY DIGITAL LOW-PASS FILTERS
%% ══════════════════════════════════════════════════════════════════════════
% The LSM6DSV does not have a separately-configurable DLPF register like
% the ICM-20948.  Filtering is implicit in the power mode and ODR selection.
% In high-performance mode, the anti-aliasing bandwidth is approximately
% ODR/2.  When enable_dlpf is set, we model this as a 2nd-order
% Butterworth at ODR/2 — a reasonable approximation since the actual
% on-chip filter topology is not published.

if imu_config.enable_dlpf
    accel_lpf_bw = accel_odr / 2;
    fprintf('Applying accel LPF (BW=%.1f Hz, Butterworth approx)...\n', accel_lpf_bw);
    [b_acc, a_acc] = butter(2, accel_lpf_bw / (fs_internal/2));
    accel_bx_g_hi = filtfilt(b_acc, a_acc, accel_bx_g_hi);
    accel_by_g_hi = filtfilt(b_acc, a_acc, accel_by_g_hi);
    accel_bz_g_hi = filtfilt(b_acc, a_acc, accel_bz_g_hi);

    gyro_lpf_bw = gyro_odr / 2;
    fprintf('Applying gyro LPF (BW=%.1f Hz, Butterworth approx)...\n', gyro_lpf_bw);
    [b_gyr, a_gyr] = butter(2, gyro_lpf_bw / (fs_internal/2));
    gyro_x_dps_hi = filtfilt(b_gyr, a_gyr, gyro_x_dps_hi);
    gyro_y_dps_hi = filtfilt(b_gyr, a_gyr, gyro_y_dps_hi);
    gyro_z_dps_hi = filtfilt(b_gyr, a_gyr, gyro_z_dps_hi);
end

% BMP581 IIR filter: y[n] = (y[n-1] * coeff + x[n]) / (coeff + 1)
% Same topology as BMP388
if baro_config.enable_iir && baro_iir_coeff > 0
    fprintf('Applying baro IIR filter (coeff=%d)...\n', baro_iir_coeff);
    c = baro_iir_coeff;
    pressure_filtered = zeros(N_internal, 1);
    pressure_filtered(1) = pressure_hi(1);
    for i = 2:N_internal
        pressure_filtered(i) = (pressure_filtered(i-1) * c + pressure_hi(i)) / (c + 1);
    end
    pressure_hi = pressure_filtered;
end

%% ══════════════════════════════════════════════════════════════════════════
%%  STEP 7: QUANTIZE TO ADC RESOLUTION
%% ══════════════════════════════════════════════════════════════════════════
% LSM6DSV: 16-bit signed ADC
%   accel LSB = sensitivity from Table 2 (e.g. 0.244 mg/LSB at ±8g)
%   gyro LSB  = sensitivity from Table 2 (e.g. 8.75 mdps/LSB at ±250 dps)

if imu_config.enable_quantization
    fprintf('Quantizing IMU to 16-bit ADC (LSM6DSV)...\n');
    accel_lsb_g   = 1.0 / accel_sensitivity;  % g per LSB
    gyro_lsb_dps  = 1.0 / gyro_sensitivity;   % dps per LSB

    % Quantize: round to nearest LSB, clamp to range
    accel_bx_g_hi = clamp_and_quantize(accel_bx_g_hi, accel_lsb_g, accel_fsr_g);
    accel_by_g_hi = clamp_and_quantize(accel_by_g_hi, accel_lsb_g, accel_fsr_g);
    accel_bz_g_hi = clamp_and_quantize(accel_bz_g_hi, accel_lsb_g, accel_fsr_g);

    gyro_x_dps_hi = clamp_and_quantize(gyro_x_dps_hi, gyro_lsb_dps, gyro_fsr_dps);
    gyro_y_dps_hi = clamp_and_quantize(gyro_y_dps_hi, gyro_lsb_dps, gyro_fsr_dps);
    gyro_z_dps_hi = clamp_and_quantize(gyro_z_dps_hi, gyro_lsb_dps, gyro_fsr_dps);
end

if baro_config.enable_quantization
    % BMP581: 24-bit ADC, resolution down to 1/64 Pa at highest OSR
    % Pressure resolution depends on oversampling:
    %   1x  → ~1 Pa/LSB (practical)
    %   16x → ~1/16 Pa/LSB
    %   128x → ~1/64 Pa/LSB  (datasheet: resolution down to 1/64 Pa)
    baro_lsb_pa = 1.0 / baro_osr_n;  % Approximate Pa per LSB, scales with OSR
    baro_lsb_pa = max(baro_lsb_pa, 1/64);  % Floor at datasheet limit
    pressure_hi = round(pressure_hi / baro_lsb_pa) * baro_lsb_pa;
    fprintf('Baro quantization step: %.4f Pa\n', baro_lsb_pa);
end

%% ══════════════════════════════════════════════════════════════════════════
%%  STEP 8: DOWNSAMPLE TO SENSOR ODRs
%% ══════════════════════════════════════════════════════════════════════════
fprintf('Downsampling to sensor ODRs...\n');

% ── Accel: downsample to accel_odr ──
t_accel = (t_internal(1) : 1/accel_odr : t_internal(end))';
accel_bx_out = interp1(t_internal, accel_bx_g_hi, t_accel, 'nearest');
accel_by_out = interp1(t_internal, accel_by_g_hi, t_accel, 'nearest');
accel_bz_out = interp1(t_internal, accel_bz_g_hi, t_accel, 'nearest');
N_accel = length(t_accel);

% ── Gyro: downsample to gyro_odr ──
t_gyro = (t_internal(1) : 1/gyro_odr : t_internal(end))';
gyro_x_out = interp1(t_internal, gyro_x_dps_hi, t_gyro, 'nearest');
gyro_y_out = interp1(t_internal, gyro_y_dps_hi, t_gyro, 'nearest');
gyro_z_out = interp1(t_internal, gyro_z_dps_hi, t_gyro, 'nearest');
N_gyro = length(t_gyro);

% ── Baro: downsample to baro_odr ──
t_baro = (t_internal(1) : 1/baro_odr : t_internal(end))';
pressure_out = interp1(t_internal, pressure_hi, t_baro, 'nearest');
N_baro = length(t_baro);

% Convert accel from g to m/s^2 for output (firmware uses m/s^2)
accel_bx_out_ms2 = accel_bx_out * g;
accel_by_out_ms2 = accel_by_out * g;
accel_bz_out_ms2 = accel_bz_out * g;

% Convert gyro from dps to rad/s for output (firmware uses rad/s)
gyro_x_out_rads = deg2rad(gyro_x_out);
gyro_y_out_rads = deg2rad(gyro_y_out);
gyro_z_out_rads = deg2rad(gyro_z_out);

% Compute altitude from pressure (as firmware would)
slp = baro_config.sea_level_pressure_hpa * 100;  % Pa
altitude_out = 44330 * (1 - (pressure_out / slp).^(1/5.255));

% Placeholder magnetometer (MMC5983MA — not yet modeled with noise)
% and temperature (TMP117 — not yet modeled with noise)
mag_x = 25.0 * ones(N_accel, 1);
mag_y = zeros(N_accel, 1);
mag_z = -45.0 * ones(N_accel, 1);
imu_temp  = 25.0 * ones(N_accel, 1);
baro_temp = 25.0 * ones(N_baro, 1);

fprintf('  Accel samples: %d (%.1f Hz)\n', N_accel, accel_odr);
fprintf('  Gyro samples:  %d (%.1f Hz)\n', N_gyro, gyro_odr);
fprintf('  Baro samples:  %d (%.1f Hz)\n', N_baro, baro_odr);

%% ══════════════════════════════════════════════════════════════════════════
%%  STEP 8b: RESAMPLE BARO TO IMU TICK RATE
%% ══════════════════════════════════════════════════════════════════════════
% Baro was downsampled to baro_odr; resample to accel time base so both
% streams share one frame count in the binary file.
pressure_imu  = interp1(t_baro, pressure_out,  t_accel, 'nearest', pressure_out(end));
altitude_imu  = interp1(t_baro, altitude_out,  t_accel, 'nearest', altitude_out(end));
baro_temp_imu = 25.0 * ones(N_accel, 1);  % placeholder temperature

% Gyro was downsampled to gyro_odr; resample to accel time base
gyro_x_imu = interp1(t_gyro, gyro_x_out_rads, t_accel, 'nearest', gyro_x_out_rads(end));
gyro_y_imu = interp1(t_gyro, gyro_y_out_rads, t_accel, 'nearest', gyro_y_out_rads(end));
gyro_z_imu = interp1(t_gyro, gyro_z_out_rads, t_accel, 'nearest', gyro_z_out_rads(end));

%% ══════════════════════════════════════════════════════════════════════════
%%  STEP 9: WRITE BINARY FILE FOR SD CARD
%% ══════════════════════════════════════════════════════════════════════════
% Format: 12-byte header + N_accel frames of 52 bytes each
%   Header: magic(uint32) + count(uint32) + rate(float32)
%   Frame:  IMUData(40 bytes) + BarometerData(12 bytes)
%
% Field order matches C++ struct layout (no padding, all float32):
%   IMUData:      gyro.x/y/z, accel.x/y/z, mag.x/y/z, temp
%   BarometerData: temperature, pressure, altitude
%
% Copy SIM.BIN to the root of the Teensy SD card before running 'make sim'.

fprintf('Writing binary file to: %s\n', output_file);

fid = fopen(output_file, 'wb');
if fid == -1
    error('Could not open %s for writing.', output_file);
end

% Header
SIM_MAGIC = uint32(hex2dec('424D4953'));  % 'SIMB' little-endian
fwrite(fid, SIM_MAGIC,           'uint32', 0, 'ieee-le');
fwrite(fid, uint32(N_accel),     'uint32', 0, 'ieee-le');
fwrite(fid, single(accel_odr),   'single', 0, 'ieee-le');

% Frames: N_accel x 13 matrix, written row-by-row via transpose
% Column order must match C++ struct field order exactly:
%   [gx, gy, gz, ax, ay, az, mx, my, mz, imu_temp, baro_temp, pressure, altitude]
frame_data = single([
    gyro_x_imu,       gyro_y_imu,       gyro_z_imu, ...
    accel_bx_out_ms2, accel_by_out_ms2, accel_bz_out_ms2, ...
    mag_x,            mag_y,            mag_z, ...
    imu_temp, ...
    baro_temp_imu,    pressure_imu,     altitude_imu
]);  % N_accel x 13

% fwrite on the transpose writes column-major over 13xN_accel,
% which is the same as row-major over N_accel x 13 — one frame at a time.
fwrite(fid, frame_data', 'single', 0, 'ieee-le');

fclose(fid);

expected_bytes = 12 + N_accel * 52;
actual_bytes   = dir(output_file).bytes;

fprintf('\n✓ Generated %s\n', output_file);
fprintf('  Frames: %d (%.1f s at %.1f Hz)\n', N_accel, N_accel/accel_odr, accel_odr);
fprintf('  Expected size: %d bytes, actual: %d bytes — %s\n', ...
        expected_bytes, actual_bytes, ...
        iif(expected_bytes == actual_bytes, 'OK', 'MISMATCH!'));
fprintf('  >>> Copy SIM.BIN to the root of the SD card before running make sim\n');

%% ══════════════════════════════════════════════════════════════════════════
%%  VALIDATION PLOTS
%% ══════════════════════════════════════════════════════════════════════════
figure('Name', 'Spoofed Sensor Validation', 'Position', [100 100 1200 900]);
theme('light');
% ── Accel body-z (along nose) ──
subplot(3,2,1);
plot(or_time, true_accel_bz, 'b-', 'DisplayName', 'Truth');
hold on;
plot(t_accel, accel_bz_out_ms2, 'r.', 'MarkerSize', 2, 'DisplayName', 'Spoofed');
xlabel('Time (s)'); ylabel('Accel Body-Z (m/s^2)');
title('Accelerometer — Axial (Body Z)');
legend('Location', 'best'); grid on;

% ── Accel body-x (lateral) ──
subplot(3,2,2);
plot(or_time, true_accel_bx, 'b-', 'DisplayName', 'Truth');
hold on;
plot(t_accel, accel_bx_out_ms2, 'r.', 'MarkerSize', 2, 'DisplayName', 'Spoofed');
xlabel('Time (s)'); ylabel('Accel Body-X (m/s^2)');
title('Accelerometer — Lateral (Body X)');
legend('Location', 'best'); grid on;

% ── Barometer pressure ──
subplot(3,2,3);
plot(or_time, or_pressure, 'b-', 'DisplayName', 'Truth');
hold on;
plot(t_baro, pressure_out, 'r.', 'MarkerSize', 2, 'DisplayName', 'Spoofed');
xlabel('Time (s)'); ylabel('Pressure (Pa)');
title('Barometer Pressure');
legend('Location', 'best'); grid on;

% ── Altitude (derived from spoofed baro) ──
subplot(3,2,4);
plot(or_time, or_altitude, 'b-', 'DisplayName', 'Truth (OR)');
hold on;
plot(t_baro, altitude_out, 'r.', 'MarkerSize', 2, 'DisplayName', 'From Baro');
xlabel('Time (s)'); ylabel('Altitude (m)');
title('Altitude: Truth vs Baro-Derived');
legend('Location', 'best'); grid on;

% ── Gyro (pitch rate) ──
subplot(3,2,5);
plot(or_time, rad2deg(true_gyro_y), 'b-', 'DisplayName', 'Truth');
hold on;
plot(t_accel, rad2deg(gyro_y_imu), 'r.', 'MarkerSize', 2, 'DisplayName', 'Spoofed');
xlabel('Time (s)'); ylabel('Gyro Y (dps)');
title('Gyroscope — Pitch Rate');
legend('Location', 'best'); grid on;

% ── Flight profile overlay ──
subplot(3,2,6);
yyaxis left;
plot(or_time, or_altitude, 'b-');
ylabel('Altitude (m)');
yyaxis right;
plot(or_time, or_thrust, 'r-');
ylabel('Thrust (N)');
xlabel('Time (s)');
title('Flight Profile');
grid on;

sgtitle('Spoofed Sensor Data Validation');

%% ══════════════════════════════════════════════════════════════════════════
%%  EXPORT TRUTH DATA FOR EKF VALIDATION
%% ══════════════════════════════════════════════════════════════════════════
% Save truth data at accel ODR for post-flight comparison
truth = struct();
truth.t         = t_accel;
truth.x         = interp1(or_time, or_lat_dist, t_accel, 'pchip');
truth.vx        = interp1(or_time, or_vx, t_accel, 'pchip');
truth.y         = interp1(or_time, or_altitude, t_accel, 'pchip');
truth.vy        = interp1(or_time, or_vy, t_accel, 'pchip');
truth.beta      = interp1(or_time, or_zenith, t_accel, 'pchip');
truth.mass      = interp1(or_time, or_mass, t_accel, 'pchip');
truth.thrust    = interp1(or_time, or_thrust, t_accel, 'pchip');
truth.drag      = interp1(or_time, or_drag, t_accel, 'pchip');
truth.cd        = interp1(or_time, or_cd, t_accel, 'pchip');
truth.ref_area  = interp1(or_time, or_ref_area, t_accel, 'pchip');
truth.density   = interp1(or_time, or_density, t_accel, 'pchip');
truth.pressure  = interp1(or_time, or_pressure, t_accel, 'pchip');

save('truth_data.mat', 'truth', ...
    'pressure_out', 't_baro', ...
    'accel_bx_out_ms2', 'accel_by_out_ms2', 'accel_bz_out_ms2', ...
    't_accel', 'altitude_out', ...
    'accel_odr', 'baro_odr', 'baro_osr_n', ...
    'imu_config', 'baro_config');
fprintf('✓ Saved truth_data.mat for EKF validation\n');

%% ══════════════════════════════════════════════════════════════════════════
%%  HELPER FUNCTIONS
%% ══════════════════════════════════════════════════════════════════════════
function out = clamp_and_quantize(signal, lsb, fsr)
    % Clamp to ±fsr, then quantize to LSB steps
    out = max(-fsr, min(fsr, signal));
    out = round(out / lsb) * lsb;
end

function out = iif(cond, a, b)
    if cond, out = a; else, out = b; end
end