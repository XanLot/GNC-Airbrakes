#ifndef sd_log_file_hpp
#define sd_log_file_hpp
#include "imu.hpp"

class sd_log {
public:
    sd_log();
    void init();
    void logIMUData(const IMUData& data);
private:
    File logFile;
}


#endif /* sd_log_file_hpp */