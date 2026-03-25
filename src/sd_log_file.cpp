#include <SD.h>
#include <cstdio>
#include <cstring>
#include "sd_log_file.hpp"

static constexpr size_t BUF_SIZE = 16384;
static constexpr unsigned long FLUSH_INTERVAL_US = 1000000;

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
        "imu1_ax,imu1_ay,imu1_az,imu1_gx,imu1_gy,imu1_gz,imu1_temp,"
        "imu2_ax,imu2_ay,imu2_az,imu2_gx,imu2_gy,imu2_gz,imu2_temp,"
        "imu3_ax,imu3_ay,imu3_az,imu3_gx,imu3_gy,imu3_gz,imu3_temp,"
        "imu4_ax,imu4_ay,imu4_az,imu4_gx,imu4_gy,imu4_gz,imu4_temp,"
        "baro1_temp,baro1_pres,baro1_alt,"
        "baro2_temp,baro2_pres,baro2_alt,"
        "mag_x,mag_y,mag_z,"
        "tmp1_temp,tmp2_temp\n";

    pimpl->logFile.write(header, strlen(header));
    pimpl->logFile.flush();
    pimpl->lastFlushUs = micros();
    return true;
}

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

void sd_log::log(const SensorData& data) {
    if (!pimpl->initialized) {
        return;
    }

    char row[600];
    size_t p = 0;

    p += snprintf(row + p, sizeof(row) - p, "%lu,", data.timestamp_us);

    for (int i = 0; i < 4; i++) {
        const auto& imu = data.imu[i];
        p = appendFloat(row, p, sizeof(row), imu.accel.x, 4);
        row[p++] = ',';
        p = appendFloat(row, p, sizeof(row), imu.accel.y, 4);
        row[p++] = ',';
        p = appendFloat(row, p, sizeof(row), imu.accel.z, 4);
        row[p++] = ',';
        p = appendFloat(row, p, sizeof(row), imu.gyro.x, 4);
        row[p++] = ',';
        p = appendFloat(row, p, sizeof(row), imu.gyro.y, 4);
        row[p++] = ',';
        p = appendFloat(row, p, sizeof(row), imu.gyro.z, 4);
        row[p++] = ',';
        p = appendFloat(row, p, sizeof(row), imu.temp, 2);
        row[p++] = ',';
    }

    for (int i = 0; i < 2; i++) {
        const auto& baro = data.baro[i];
        p = appendFloat(row, p, sizeof(row), baro.temperature, 2);
        row[p++] = ',';
        p = appendFloat(row, p, sizeof(row), baro.pressure, 1);
        row[p++] = ',';
        p = appendFloat(row, p, sizeof(row), baro.altitude, 2);
        row[p++] = ',';
    }

    p = appendFloat(row, p, sizeof(row), data.mag.field.x, 4);
    row[p++] = ',';
    p = appendFloat(row, p, sizeof(row), data.mag.field.y, 4);
    row[p++] = ',';
    p = appendFloat(row, p, sizeof(row), data.mag.field.z, 4);
    row[p++] = ',';

    p = appendFloat(row, p, sizeof(row), data.tmp[0].temperature, 4);
    row[p++] = ',';
    p = appendFloat(row, p, sizeof(row), data.tmp[1].temperature, 4);
    row[p++] = '\n';

    size_t rowLen = p;

    if (pimpl->bufPos + rowLen > BUF_SIZE) {
        pimpl->logFile.write(pimpl->buffer, pimpl->bufPos);
        pimpl->bufPos = 0;
    }

    memcpy(pimpl->buffer + pimpl->bufPos, row, rowLen);
    pimpl->bufPos += rowLen;

    unsigned long now = micros();
    if (now - pimpl->lastFlushUs >= FLUSH_INTERVAL_US) {
        pimpl->logFile.write(pimpl->buffer, pimpl->bufPos);
        pimpl->bufPos = 0;
        pimpl->logFile.flush();
        pimpl->lastFlushUs = now;
    }
}

void sd_log::flush() {
    if (!pimpl->initialized) {
        return;
    }
    if (pimpl->bufPos > 0) {
        pimpl->logFile.write(pimpl->buffer, pimpl->bufPos);
        pimpl->bufPos = 0;
    }
    pimpl->logFile.flush();
}
