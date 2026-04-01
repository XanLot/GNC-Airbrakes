#include <SD.h>
#include <cstdio>
#include <cstring>
#include "sd_log_file.hpp"

static constexpr size_t BUF_SIZE = 4096;

static constexpr unsigned long FLUSH_INTERVAL_US = 1000000; // 1 second

struct sd_log::Impl {
    File logFile;
    bool initialized = false;
    char buffer[BUF_SIZE];
    size_t bufPos = 0;
    unsigned long lastFlushUs = 0;
};

sd_log::sd_log() : pimpl(new Impl) {}

bool sd_log::init() {
    if (!SD.begin(BUILTIN_SDCARD)) {
        return false;
    }

    char filename[32];
    int index = 0;

    do {
        snprintf(filename, sizeof(filename), "log_%04d.csv", index++);
    } while (SD.exists(filename));

    pimpl->logFile = SD.open(filename, FILE_WRITE);
    if (!pimpl->logFile) {
        return false;
    }

    pimpl->initialized = true;
    pimpl->bufPos = 0;

    const char header[] =
        "timestamp_us,"
        "accel_x,accel_y,accel_z,"
        "gyro_x,gyro_y,gyro_z,"
        "mag_x,mag_y,mag_z,"
        "temp_c,pressure_pa,altitude_m\n";

    // Write header directly to SD so the file isn't empty if power is cut early
    pimpl->logFile.write(header, strlen(header));
    pimpl->logFile.flush();
    pimpl->lastFlushUs = micros();

    return true;
}

// Append a float as text into buf at position pos. Returns new position.
static size_t appendFloat(char* buf, size_t pos, size_t max, float val, uint8_t decimals) {
    char tmp[16];
    dtostrf(val, 0, decimals, tmp);
    size_t tlen = strlen(tmp);
    if (pos + tlen < max) {
        memcpy(buf + pos, tmp, tlen);
        pos += tlen;
    }
    return pos;
}

void sd_log::log(const IMUData& imu, const BarometerData& baro) {
    if (!pimpl->initialized) return;

    char row[256];
    size_t p = 0;

    // Timestamp
    p += snprintf(row + p, sizeof(row) - p, "%lu,", (unsigned long)micros());

    // Accel x,y,z (4 decimals)
    p = appendFloat(row, p, sizeof(row), imu.accel.x, 4); row[p++] = ',';
    p = appendFloat(row, p, sizeof(row), imu.accel.y, 4); row[p++] = ',';
    p = appendFloat(row, p, sizeof(row), imu.accel.z, 4); row[p++] = ',';

    // Gyro x,y,z (4 decimals)
    p = appendFloat(row, p, sizeof(row), imu.gyro.x, 4); row[p++] = ',';
    p = appendFloat(row, p, sizeof(row), imu.gyro.y, 4); row[p++] = ',';
    p = appendFloat(row, p, sizeof(row), imu.gyro.z, 4); row[p++] = ',';

    // Mag x,y,z (2 decimals)
    p = appendFloat(row, p, sizeof(row), imu.mag.x, 2); row[p++] = ',';
    p = appendFloat(row, p, sizeof(row), imu.mag.y, 2); row[p++] = ',';
    p = appendFloat(row, p, sizeof(row), imu.mag.z, 2); row[p++] = ',';

    // Temp (2 decimals)
    p = appendFloat(row, p, sizeof(row), imu.temp, 2); row[p++] = ',';

    // Pressure (1 decimal), Altitude (2 decimals)
    p = appendFloat(row, p, sizeof(row), baro.pressure, 1); row[p++] = ',';
    p = appendFloat(row, p, sizeof(row), baro.altitude, 2);

    row[p++] = '\n';

    size_t rowLen = p;

    // If appending would overflow, flush current buffer first
    if (pimpl->bufPos + rowLen > BUF_SIZE) {
        pimpl->logFile.write(pimpl->buffer, pimpl->bufPos);
        pimpl->bufPos = 0;
    }

    memcpy(pimpl->buffer + pimpl->bufPos, row, rowLen);
    pimpl->bufPos += rowLen;

    // Periodic flush so data survives power loss
    unsigned long now = micros();
    if (now - pimpl->lastFlushUs >= FLUSH_INTERVAL_US) {
        pimpl->logFile.write(pimpl->buffer, pimpl->bufPos);
        pimpl->bufPos = 0;
        pimpl->logFile.flush();
        pimpl->lastFlushUs = now;
    }
}

void sd_log::flush() {
    if (!pimpl->initialized) return;

    if (pimpl->bufPos > 0) {
        pimpl->logFile.write(pimpl->buffer, pimpl->bufPos);
        pimpl->bufPos = 0;
    }
    pimpl->logFile.flush();
}
