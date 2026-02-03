#ifndef SD_LOG_FILE_HPP
#define SD_LOG_FILE_HPP

#include "imu.hpp"

// -----------------------------------------------------------------------------
// sd_log: Collects IMU readings (mag, gyro, accel) and writes them to a CSV file.
//
// This header only declares the public interface. The actual implementation
// details (SD card, File objects, buffering, etc.) are hidden inside the .cpp
// using the PIMPL pattern so the header stays clean and portable.
// -----------------------------------------------------------------------------
class sd_log {
public:
    sd_log();       // Constructor: sets up internal state (no SD access yet)
    void init();    // Initializes SD card and opens the CSV file

    // Store the latest sensor readings (does NOT write to SD immediately)
    void logMagData(const Vec3& data);
    void logGyroData(const Vec3& data);
    void logAccelData(const Vec3& data);

    // Writes one combined CSV row using the latest available values.
    // If a sensor didn't update this cycle, its last known value is reused.
    void writeCombinedRow();

private:
    // Forward declaration of the hidden implementation struct
    struct Impl;

    // Pointer to the hidden implementation (allocated in the .cpp)
    Impl* pimpl;
};

#endif // SD_LOG_FILE_HPP
