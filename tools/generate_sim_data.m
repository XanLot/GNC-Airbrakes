%% generate_sim_data.m
%% Generates SIM.BIN for SD card from an OpenRocket CSV export.
%%
%% Pipeline:
%%   1. Load OpenRocket CSV
%%   2. Compute true body-frame sensor readings (specific force, baro pressure)
%%   3. Add per-channel bias
%%   4. Add white noise (per channel with independent seeds)
%%   5. Apply on-chip LP filter model (cutoff = ODR/4)
%%   6. Quantize to ADC resolution
%%   7. Downsample to sensor ODR
%%   8. Write binary SIM.BIN for SD card
%%
%% Usage:
%%   1. Export OpenRocket CSV with required columns (see below)
%%   2. Configure sensor counts and settings in sim_config.m
%%   3. Run this script — then copy SIM.BIN to SD card root
%%
%% Required OpenRocket CSV columns (in this order):
%%   Time [s], Altitude [m], Vertical velocity [m/s],
%%   Vertical acceleration [m/s^2], Lateral distance [m],
%%   Lateral velocity [m/s], Lateral acceleration [m/s^2],
%%   Vertical orientation (zenith) [rad], Mass [kg], Thrust [N],
%%   Drag force [N], Drag coefficient, Air pressure [Pa],
%%   Air density [kg/m^3], Reference area [m^2]

clear; clc; close all;

%% ══════════════════════════════════════════════════════════════════════════
%%  FILE PATHS
%% ══════════════════════════════════════════════════════════════════════════
openrocket_csv = 'CompSim.csv';
output_file    = 'SIM.BIN';

%% ══════════════════════════════════════════════════════════════════════════
%%  SENSOR COUNT CONFIG
%%  Match to what's physically working on the board.
%%  Inactive slots will be NaN in SensorData (same as a failed hardware init).
%% ══════════════════════════════════════════════════════════════════════════
num_imus  = 2;   % 1, 2, or 4  (SPI0: IMU1/2; SPI1: IMU3/4 — SPI1 not yet working)
num_baros = 1;   % 1 or 2      (SPI0: Baro1; SPI1: Baro2 — SPI1 not yet working)
has_mag   = 0;   % 0 or 1      (MMC5983MA on I2C — not used by state machine yet)
num_temps = 0;   % 0, 1, or 2  (TMP117 on I2C — not used by state machine yet)

%% ══════════════════════════════════════════════════════════════════════════
%%  LOAD SHARED CONFIGURATION
%% ══════════════════════════════════════════════════════════════════════════
[imu_config, baro_config, constants] = sim_config();

g            = constants.g;
accel_lsb_g  = constants.accel_lsb_g;
gyro_lsb_dps = constants.gyro_lsb_dps;
baro_iir_c   = constants.baro_iir_coeff;
baro_osr_n   = constants.baro_osr_n;

% LP filter cutoff = ODR/4 (LSM6DSV16X LP2/LP1 typical for high-performance mode)
accel_lp_bw = imu_config.accel_odr_hz / 4;
gyro_lp_bw  = imu_config.gyro_odr_hz  / 4;

% BMP581 effective noise after oversampling
baro_effective_noise = baro_config.pressure_noise_pa / sqrt(baro_osr_n);

fprintf('═══ Sensor Configuration Summary ═══\n');
fprintf('IMUs:  %d channels, ±%dg, ODR=%d Hz, LP=%d Hz%s\n', ...
    num_imus, imu_config.accel_range_g, imu_config.accel_odr_hz, ...
    round(accel_lp_bw), iif(imu_config.enable_lp_filter, '', ' (disabled)'));
fprintf('Baros: %d channels, ODR=%d Hz, IIR coeff=%d%s\n', ...
    num_baros, baro_config.odr_hz, baro_iir_c, ...
    iif(baro_config.enable_iir, '', ' (disabled)'));
fprintf('Mag:   %d  Temps: %d\n', has_mag, num_temps);
fprintf('══════════════════════════════════════\n\n');

%% ══════════════════════════════════════════════════════════════════════════
%%  STEP 1: LOAD OPENROCKET CSV
%% ══════════════════════════════════════════════════════════════════════════
fprintf('Loading OpenRocket data from: %s\n', openrocket_csv);

raw_data = readmatrix(openrocket_csv, 'NumHeaderLines', 0, 'CommentStyle', '#');
valid    = ~isnan(raw_data(:,1));
raw_data = raw_data(valid, :);
fprintf('  Found %d columns, %d valid rows\n', size(raw_data,2), size(raw_data,1));

% Map columns by index — order matches OpenRocket export format
or_time     = raw_data(:,1);   % s
or_altitude = raw_data(:,2);   % m
or_vy       = raw_data(:,3);   % m/s   vertical velocity
or_ay       = raw_data(:,4);   % m/s^2 vertical acceleration
or_lat_dist = raw_data(:,5);   % m     lateral distance
or_vx       = raw_data(:,6);   % m/s   lateral velocity
or_ax       = raw_data(:,7);   % m/s^2 lateral acceleration
or_zenith   = raw_data(:,8);   % rad   angle from vertical (0 = pointing straight up)
or_mass     = raw_data(:,9);   % kg
or_thrust   = raw_data(:,10);  % N
or_drag     = raw_data(:,11);  % N
or_cd       = raw_data(:,12);
or_pressure = raw_data(:,13);  % Pa
or_density  = raw_data(:,14);  % kg/m^3
or_ref_area = raw_data(:,15);  % m^2

N_or  = length(or_time);
dt_or = median(diff(or_time));
fs_or = 1 / dt_or;
fprintf('  Duration: %.2f s, OR sample rate: ~%.1f Hz\n', or_time(end), fs_or);

%% ══════════════════════════════════════════════════════════════════════════
%%  STEP 2: COMPUTE TRUE SENSOR READINGS AT HIGH RATE
%% ══════════════════════════════════════════════════════════════════════════
% Accelerometer measures specific force in the body frame.
% Specific force in inertial frame:
%   sf_x = ax_lateral           (lateral coord accel = lateral specific force)
%   sf_y = ay_vertical + g      (remove gravity from coord accel)
%
% Rotate to body frame using zenith angle β (angle from vertical, 0 = straight up):
%   body_z (along nose)  =  sf_x·sin(β) + sf_y·cos(β)
%   body_x (lateral)     =  sf_x·cos(β) - sf_y·sin(β)
%   body_y               =  0  (2D assumption)
%
% Check at launchpad: β=0 → body_z = sf_y = g, body_x = 0. Correct.

fprintf('Computing true body-frame sensor readings...\n');

sf_x_inertial = or_ax;
sf_y_inertial = or_ay + g;
beta          = or_zenith;   % already angle from vertical — no conversion needed

true_accel_bz = sf_x_inertial .* sin(beta) + sf_y_inertial .* cos(beta);
true_accel_bx = sf_x_inertial .* cos(beta) - sf_y_inertial .* sin(beta);
true_accel_by = zeros(N_or, 1);

true_accel_bz_g = true_accel_bz / g;
true_accel_bx_g = true_accel_bx / g;
true_accel_by_g = true_accel_by / g;

true_pressure = or_pressure;

true_gyro_y     = gradient(beta, or_time);   % pitch rate rad/s
true_gyro_x     = zeros(N_or, 1);
true_gyro_z     = zeros(N_or, 1);
true_gyro_x_dps = rad2deg(true_gyro_x);
true_gyro_y_dps = rad2deg(true_gyro_y);
true_gyro_z_dps = rad2deg(true_gyro_z);

%% ══════════════════════════════════════════════════════════════════════════
%%  STEP 3: INTERPOLATE TO HIGH INTERNAL RATE
%% ══════════════════════════════════════════════════════════════════════════
fs_internal = max([imu_config.accel_odr_hz, imu_config.gyro_odr_hz, baro_config.odr_hz]) * 4;
fs_internal = max(fs_internal, fs_or);
dt_internal = 1 / fs_internal;
t_internal  = (or_time(1) : dt_internal : or_time(end))';
N_internal  = length(t_internal);
fprintf('Internal simulation rate: %.1f Hz (%d samples)\n', fs_internal, N_internal);

accel_bx_g_hi = interp1(or_time, true_accel_bx_g, t_internal, 'pchip');
accel_by_g_hi = interp1(or_time, true_accel_by_g, t_internal, 'pchip');
accel_bz_g_hi = interp1(or_time, true_accel_bz_g, t_internal, 'pchip');
gyro_x_dps_hi = interp1(or_time, true_gyro_x_dps,  t_internal, 'pchip');
gyro_y_dps_hi = interp1(or_time, true_gyro_y_dps,  t_internal, 'pchip');
gyro_z_dps_hi = interp1(or_time, true_gyro_z_dps,  t_internal, 'pchip');
pressure_hi   = interp1(or_time, true_pressure,    t_internal, 'pchip');

%% ══════════════════════════════════════════════════════════════════════════
%%  STEPS 4-7: PER-CHANNEL IMU PROCESSING
%%  Each channel gets independent noise (different RNG seed per channel).
%%  All channels share the same truth signal — only noise/bias differs.
%% ══════════════════════════════════════════════════════════════════════════
imu_out = cell(num_imus, 1);

for ch = 1:num_imus
    fprintf('\nProcessing IMU channel %d/%d...\n', ch, num_imus);

    ax = accel_bx_g_hi;
    ay = accel_by_g_hi;
    az = accel_bz_g_hi;
    gx = gyro_x_dps_hi;
    gy = gyro_y_dps_hi;
    gz = gyro_z_dps_hi;

    % Step 4: bias (drawn once per channel, constant over run)
    if imu_config.enable_bias
        rng(100 * ch + 1, 'twister');
        abias = (imu_config.accel_bias_mg / 1000) * (2*rand(1,3) - 1);  % g
        rng(100 * ch + 2, 'twister');
        gbias = imu_config.gyro_bias_dps * (2*rand(1,3) - 1);           % dps
        fprintf('  Accel bias (mg): [%.2f %.2f %.2f]\n', abias*1000);
        fprintf('  Gyro bias (dps): [%.2f %.2f %.2f]\n', gbias);
    else
        abias = [0 0 0];
        gbias = [0 0 0];
    end
    ax = ax + abias(1);  ay = ay + abias(2);  az = az + abias(3);
    gx = gx + gbias(1);  gy = gy + gbias(2);  gz = gz + gbias(3);

    % Step 5: white noise
    if imu_config.enable_noise
        rng(100 * ch + 3, 'twister');
        a_noise_rms = imu_config.accel_noise_density * sqrt(fs_internal / 2);
        ax = ax + a_noise_rms * randn(N_internal, 1);
        ay = ay + a_noise_rms * randn(N_internal, 1);
        az = az + a_noise_rms * randn(N_internal, 1);

        rng(100 * ch + 4, 'twister');
        g_noise_rms = imu_config.gyro_noise_density * sqrt(fs_internal / 2);
        gx = gx + g_noise_rms * randn(N_internal, 1);
        gy = gy + g_noise_rms * randn(N_internal, 1);
        gz = gz + g_noise_rms * randn(N_internal, 1);

        fprintf('  Accel noise RMS pre-filter: %.4f mg\n', a_noise_rms * 1000);
        fprintf('  Gyro noise RMS pre-filter:  %.4f dps\n', g_noise_rms);
    end

    % Step 6: LP filter (2nd-order Butterworth at ODR/4 — matches LSM6DSV16X LP2/LP1)
    if imu_config.enable_lp_filter
        [b_a, a_a] = butter(2, accel_lp_bw / (fs_internal/2));
        ax = filtfilt(b_a, a_a, ax);
        ay = filtfilt(b_a, a_a, ay);
        az = filtfilt(b_a, a_a, az);

        [b_g, a_g] = butter(2, gyro_lp_bw / (fs_internal/2));
        gx = filtfilt(b_g, a_g, gx);
        gy = filtfilt(b_g, a_g, gy);
        gz = filtfilt(b_g, a_g, gz);
        fprintf('  LP filter applied (accel BW=%.0f Hz, gyro BW=%.0f Hz)\n', ...
            accel_lp_bw, gyro_lp_bw);
    end

    % Step 7: quantize to 16-bit ADC (LSM6DSV16X)
    if imu_config.enable_quantization
        ax = clamp_and_quantize(ax, accel_lsb_g, imu_config.accel_range_g);
        ay = clamp_and_quantize(ay, accel_lsb_g, imu_config.accel_range_g);
        az = clamp_and_quantize(az, accel_lsb_g, imu_config.accel_range_g);
        gx = clamp_and_quantize(gx, gyro_lsb_dps, imu_config.gyro_range_dps);
        gy = clamp_and_quantize(gy, gyro_lsb_dps, imu_config.gyro_range_dps);
        gz = clamp_and_quantize(gz, gyro_lsb_dps, imu_config.gyro_range_dps);
    end

    % Downsample to IMU ODR
    t_imu  = (t_internal(1) : 1/imu_config.accel_odr_hz : t_internal(end))';
    ax_out = interp1(t_internal, ax, t_imu, 'nearest');
    ay_out = interp1(t_internal, ay, t_imu, 'nearest');
    az_out = interp1(t_internal, az, t_imu, 'nearest');
    gx_out = interp1(t_internal, gx, t_imu, 'nearest');
    gy_out = interp1(t_internal, gy, t_imu, 'nearest');
    gz_out = interp1(t_internal, gz, t_imu, 'nearest');
    N_imu  = length(t_imu);

    imu_out{ch} = struct( ...
        'ax_ms2',  ax_out * g, ...
        'ay_ms2',  ay_out * g, ...
        'az_ms2',  az_out * g, ...
        'gx_rads', deg2rad(gx_out), ...
        'gy_rads', deg2rad(gy_out), ...
        'gz_rads', deg2rad(gz_out), ...
        'temp',    25.0 * ones(N_imu, 1), ...
        'N',       N_imu, ...
        't',       t_imu ...
    );
end

N_frames = imu_out{1}.N;
t_frames = imu_out{1}.t;
fprintf('\n  IMU frames: %d (%.1f s at %.1f Hz)\n', ...
    N_frames, N_frames/imu_config.accel_odr_hz, imu_config.accel_odr_hz);

%% ══════════════════════════════════════════════════════════════════════════
%%  STEPS 4-7: PER-CHANNEL BARO PROCESSING
%% ══════════════════════════════════════════════════════════════════════════
baro_out = cell(num_baros, 1);

for ch = 1:num_baros
    fprintf('Processing Baro channel %d/%d...\n', ch, num_baros);
    p = pressure_hi;

    if baro_config.enable_bias
        rng(200 * ch + 1, 'twister');
        pbias = baro_config.pressure_bias_pa * (2*rand() - 1);
        fprintf('  Pressure bias (Pa): %.2f\n', pbias);
        p = p + pbias;
    end

    if baro_config.enable_noise
        rng(200 * ch + 2, 'twister');
        p = p + baro_effective_noise * randn(N_internal, 1);
        fprintf('  Pressure noise RMS (after OSR): %.4f Pa\n', baro_effective_noise);
    end

    if baro_config.enable_iir && baro_iir_c > 0
        p_filt    = zeros(N_internal, 1);
        p_filt(1) = p(1);
        for i = 2:N_internal
            p_filt(i) = (p_filt(i-1) * baro_iir_c + p(i)) / (baro_iir_c + 1);
        end
        p = p_filt;
    end

    if baro_config.enable_quantization
        % BMP581 24-bit ADC: resolution scales with OSR, floor at 1/64 Pa
        baro_lsb_pa = max(1.0 / baro_osr_n, 1/64);
        p = round(p / baro_lsb_pa) * baro_lsb_pa;
    end

    % Downsample to baro ODR then resample to IMU tick rate
    t_baro = (t_internal(1) : 1/baro_config.odr_hz : t_internal(end))';
    p_baro = interp1(t_internal, p, t_baro, 'nearest');
    p_imu  = interp1(t_baro, p_baro, t_frames, 'nearest', p_baro(end));

    slp     = baro_config.sea_level_hpa * 100;
    alt_imu = 44330 * (1 - (p_imu / slp).^(1/5.255));

    baro_out{ch} = struct( ...
        'temp',     25.0 * ones(N_frames, 1), ...
        'pressure', p_imu, ...
        'altitude', alt_imu, ...
        't_baro',   t_baro, ...
        'p_baro',   p_baro ...
    );
end

%% ══════════════════════════════════════════════════════════════════════════
%%  STEP 8: WRITE BINARY FILE
%%
%%  SIM.BIN format (all values little-endian):
%%    Header (16 bytes):
%%      magic      uint32  0x424D4953
%%      count      uint32  number of frames
%%      rate       float32 sample rate (Hz)
%%      num_imus   uint8
%%      num_baros  uint8
%%      has_mag    uint8
%%      num_temps  uint8
%%
%%    Per frame (all float32, no padding):
%%      IMUData[num_imus]:        accel.x/y/z (m/s^2), gyro.x/y/z (rad/s), temp (°C)
%%      BarometerData[num_baros]: temperature (°C), pressure (Pa), altitude (m)
%%      MagData (if has_mag):     field.x/y/z (Gauss)
%%      TempData[num_temps]:      temperature (°C)
%%
%%  Field order matches C++ struct layout in sensor_data.hpp.
%%  Frame stride = num_imus*28 + num_baros*12 + has_mag*12 + num_temps*4 bytes
%% ══════════════════════════════════════════════════════════════════════════
fprintf('\nWriting binary file: %s\n', output_file);

fid = fopen(output_file, 'wb');
if fid == -1
    error('Could not open %s for writing.', output_file);
end

SIM_MAGIC = uint32(hex2dec('424D4953'));
fwrite(fid, SIM_MAGIC,                       'uint32', 0, 'ieee-le');
fwrite(fid, uint32(N_frames),                'uint32', 0, 'ieee-le');
fwrite(fid, single(imu_config.accel_odr_hz), 'single', 0, 'ieee-le');
fwrite(fid, uint8(num_imus),                 'uint8');
fwrite(fid, uint8(num_baros),                'uint8');
fwrite(fid, uint8(has_mag),                  'uint8');
fwrite(fid, uint8(num_temps),                'uint8');

for f = 1:N_frames
    for ch = 1:num_imus
        d = imu_out{ch};
        % Field order: accel.x/y/z, gyro.x/y/z, temp — matches IMUData struct
        fwrite(fid, single([d.ax_ms2(f),  d.ay_ms2(f),  d.az_ms2(f), ...
                            d.gx_rads(f), d.gy_rads(f), d.gz_rads(f), ...
                            d.temp(f)]), 'single', 0, 'ieee-le');
    end
    for ch = 1:num_baros
        d = baro_out{ch};
        % Field order: temperature, pressure, altitude — matches BarometerData struct
        fwrite(fid, single([d.temp(f), d.pressure(f), d.altitude(f)]), ...
               'single', 0, 'ieee-le');
    end
    % has_mag and num_temps are 0 by default — nothing written for them
end

fclose(fid);

frame_stride   = num_imus*28 + num_baros*12 + has_mag*12 + num_temps*4;
expected_bytes = 16 + N_frames * frame_stride;
actual_bytes   = dir(output_file).bytes;
fprintf('  Frames: %d  Rate: %.1f Hz  Duration: %.1f s\n', ...
    N_frames, imu_config.accel_odr_hz, N_frames / imu_config.accel_odr_hz);
fprintf('  Expected: %d bytes  Actual: %d bytes — %s\n', ...
    expected_bytes, actual_bytes, iif(expected_bytes == actual_bytes, 'OK', 'MISMATCH!'));
fprintf('  >>> Copy SIM.BIN to the root of the SD card, then run: make sim && make upload\n');

%% ══════════════════════════════════════════════════════════════════════════
%%  VALIDATION PLOTS
%% ══════════════════════════════════════════════════════════════════════════
figure('Name', 'Sim Data Validation', 'Position', [100 100 1200 900]);

subplot(3,2,1);
plot(or_time, true_accel_bz, 'b-', 'DisplayName', 'Truth');
hold on;
for ch = 1:num_imus
    plot(imu_out{ch}.t, imu_out{ch}.az_ms2, '.', 'MarkerSize', 2, ...
         'DisplayName', sprintf('IMU%d', ch));
end
xlabel('Time (s)'); ylabel('Accel Body-Z (m/s^2)');
title('Accelerometer — Axial (Body Z)'); legend('Location','best'); grid on;

subplot(3,2,2);
plot(or_time, true_accel_bx, 'b-', 'DisplayName', 'Truth');
hold on;
for ch = 1:num_imus
    plot(imu_out{ch}.t, imu_out{ch}.ax_ms2, '.', 'MarkerSize', 2, ...
         'DisplayName', sprintf('IMU%d', ch));
end
xlabel('Time (s)'); ylabel('Accel Body-X (m/s^2)');
title('Accelerometer — Lateral (Body X)'); legend('Location','best'); grid on;

subplot(3,2,3);
plot(or_time, or_pressure, 'b-', 'DisplayName', 'Truth');
hold on;
for ch = 1:num_baros
    plot(baro_out{ch}.t_baro, baro_out{ch}.p_baro, '.', 'MarkerSize', 2, ...
         'DisplayName', sprintf('Baro%d', ch));
end
xlabel('Time (s)'); ylabel('Pressure (Pa)');
title('Barometer Pressure'); legend('Location','best'); grid on;

subplot(3,2,4);
plot(or_time, or_altitude, 'b-', 'DisplayName', 'Truth (OR)');
hold on;
for ch = 1:num_baros
    plot(t_frames, baro_out{ch}.altitude, '.', 'MarkerSize', 2, ...
         'DisplayName', sprintf('Baro%d alt', ch));
end
xlabel('Time (s)'); ylabel('Altitude (m)');
title('Altitude: Truth vs Baro-Derived'); legend('Location','best'); grid on;

subplot(3,2,5);
plot(or_time, rad2deg(true_gyro_y), 'b-', 'DisplayName', 'Truth');
hold on;
for ch = 1:num_imus
    plot(imu_out{ch}.t, rad2deg(imu_out{ch}.gy_rads), '.', 'MarkerSize', 2, ...
         'DisplayName', sprintf('IMU%d', ch));
end
xlabel('Time (s)'); ylabel('Gyro Y (dps)');
title('Gyroscope — Pitch Rate'); legend('Location','best'); grid on;

subplot(3,2,6);
yyaxis left;  plot(or_time, or_altitude, 'b-'); ylabel('Altitude (m)');
yyaxis right; plot(or_time, or_thrust,   'r-'); ylabel('Thrust (N)');
xlabel('Time (s)'); title('Flight Profile'); grid on;

sgtitle('SIM.BIN Data Validation');

%% ══════════════════════════════════════════════════════════════════════════
%%  EXPORT TRUTH DATA FOR EKF VALIDATION
%% ══════════════════════════════════════════════════════════════════════════
truth = struct();
truth.t        = t_frames;
truth.x        = interp1(or_time, or_lat_dist,  t_frames, 'pchip');
truth.vx       = interp1(or_time, or_vx,        t_frames, 'pchip');
truth.y        = interp1(or_time, or_altitude,  t_frames, 'pchip');
truth.vy       = interp1(or_time, or_vy,        t_frames, 'pchip');
truth.beta     = interp1(or_time, or_zenith,    t_frames, 'pchip');
truth.mass     = interp1(or_time, or_mass,      t_frames, 'pchip');
truth.thrust   = interp1(or_time, or_thrust,    t_frames, 'pchip');
truth.drag     = interp1(or_time, or_drag,      t_frames, 'pchip');
truth.cd       = interp1(or_time, or_cd,        t_frames, 'pchip');
truth.ref_area = interp1(or_time, or_ref_area,  t_frames, 'pchip');
truth.density  = interp1(or_time, or_density,   t_frames, 'pchip');
truth.pressure = interp1(or_time, or_pressure,  t_frames, 'pchip');

% Also save sensor outputs for EKF validation scripts
accel_bx_out_ms2 = imu_out{1}.ax_ms2;
accel_by_out_ms2 = imu_out{1}.ay_ms2;
accel_bz_out_ms2 = imu_out{1}.az_ms2;
t_accel          = t_frames;
t_baro           = baro_out{1}.t_baro;
pressure_out     = baro_out{1}.p_baro;

save('truth_data.mat', 'truth', ...
    'pressure_out', 't_baro', ...
    'accel_bx_out_ms2', 'accel_by_out_ms2', 'accel_bz_out_ms2', ...
    't_accel', ...
    'imu_config', 'baro_config');
fprintf('  Saved truth_data.mat for EKF validation\n');

%% ══════════════════════════════════════════════════════════════════════════
%%  HELPER FUNCTIONS
%% ══════════════════════════════════════════════════════════════════════════
function out = clamp_and_quantize(signal, lsb, fsr)
    out = max(-fsr, min(fsr, signal));
    out = round(out / lsb) * lsb;
end

function out = iif(cond, a, b)
    if cond, out = a; else, out = b; end
end
