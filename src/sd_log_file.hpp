#ifndef SD_LOG_FILE_HPP
#define SD_LOG_FILE_HPP

#include "sensor_data.hpp"

class sd_log {
public:
    sd_log();

    bool init();
    void log(const SensorData& data);
    void flush();

private:
    struct Impl;
    Impl* pimpl;
};

#endif // SD_LOG_FILE_HPP
