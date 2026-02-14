#ifndef SD_LOG_FILE_HPP
#define SD_LOG_FILE_HPP

#include "imu.hpp"
#include "barometer.hpp"

class sd_log {
public:
    sd_log();

    // Initialize SD card and open a new session CSV file.
    // Returns true on success.
    bool init();

    // Format one CSV row with all sensor data and append to internal buffer.
    // When the buffer is full, it is written to SD in one block.
    void log(const IMUData& imu, const BarometerData& baro);

    // Force-write any buffered data to SD and sync to card.
    void flush();

private:
    struct Impl;
    Impl* pimpl;
};

#endif // SD_LOG_FILE_HPP
