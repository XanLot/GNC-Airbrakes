#include <SD.h>
#include "sd_log_file.hpp"

// -----------------------------------------------------------------------------
// Internal implementation struct (hidden from the header)
// -----------------------------------------------------------------------------
struct sd_log::Impl {
    File logFile;
    bool initialized = false;

    // Store the most recent readings from each sensor
    Vec3 lastMag;
    Vec3 lastGyro;
    Vec3 lastAccel;

    // Flags to track whether each sensor has updated this cycle
    bool magReady = false;
    bool gyroReady = false;
    bool accelReady = false;
};

// -----------------------------------------------------------------------------
// Constructor: allocate the Impl object
// -----------------------------------------------------------------------------
sd_log::sd_log() : pimpl(new Impl) {}

// -----------------------------------------------------------------------------
// init(): initialize SD card and open the CSV file
// -----------------------------------------------------------------------------
void sd_log::init() {
    if (!SD.begin(BUILTIN_SDCARD)) {
        return; // SD card not found or failed to initialize
    }

    pimpl->logFile = SD.open("imu.csv", FILE_WRITE);
    if (!pimpl->logFile) {
        return; // File could not be opened
    }

    pimpl->initialized = true;

    // Write CSV header for grouped IMU data
    pimpl->logFile.println(
        "timestamp_us,"
        "mag_x,mag_y,mag_z,"
        "gyro_x,gyro_y,gyro_z,"
        "accel_x,accel_y,accel_z"
    );
}

// -----------------------------------------------------------------------------
// Store magnetometer data (do NOT write to SD yet)
// -----------------------------------------------------------------------------
void sd_log::logMagData(const Vec3& v) {
    if (!pimpl->initialized) return;

    pimpl->lastMag = v;
    pimpl->magReady = true;
}

// -----------------------------------------------------------------------------
// Store gyroscope data
// -----------------------------------------------------------------------------
void sd_log::logGyroData(const Vec3& v) {
    if (!pimpl->initialized) return;

    pimpl->lastGyro = v;
    pimpl->gyroReady = true;
}

// -----------------------------------------------------------------------------
// Store accelerometer data
// -----------------------------------------------------------------------------
void sd_log::logAccelData(const Vec3& v) {
    if (!pimpl->initialized) return;

    pimpl->lastAccel = v;
    pimpl->accelReady = true;
}

// -----------------------------------------------------------------------------
// NEW FUNCTION: write a combined row once all three sensors have updated
// -----------------------------------------------------------------------------
void sd_log::writeCombinedRow() {
    if (!pimpl->initialized) return;

    // Only write when all three sensor readings are available
    if (pimpl->magReady && pimpl->gyroReady && pimpl->accelReady) {

        uint32_t t = micros(); // timestamp in microseconds

        // Write one complete IMU snapshot
        pimpl->logFile.printf(
            "%lu,"
            "%f,%f,%f,"      // mag
            "%f,%f,%f,"      // gyro
            "%f,%f,%f\n",    // accel
            t,
            pimpl->lastMag.x,   pimpl->lastMag.y,   pimpl->lastMag.z,
            pimpl->lastGyro.x,  pimpl->lastGyro.y,  pimpl->lastGyro.z,
            pimpl->lastAccel.x, pimpl->lastAccel.y, pimpl->lastAccel.z
        );

        // Reset flags for the next IMU cycle
        pimpl->magReady = false;
        pimpl->gyroReady = false;
        pimpl->accelReady = false;
    }
}