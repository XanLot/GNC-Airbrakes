#ifdef SIM_MODE

#include "sim_data.hpp"
#include <SD.h>

// SIM.BIN header layout (16 bytes):
//   magic      uint32  0x424D4953 ("SIMB" LE)
//   count      uint32  number of frames
//   rate       float32 sample rate in Hz
//   num_imus   uint8   1, 2, or 4
//   num_baros  uint8   1 or 2
//   has_mag    uint8   0 or 1
//   num_temps  uint8   0, 1, or 2
//
// Per-frame layout (variable, no padding):
//   IMUData[num_imus]        each = 28 bytes (Vec3 accel, Vec3 gyro, float temp)
//   BarometerData[num_baros] each = 12 bytes (float temp, float pressure, float altitude)
//   MagData (if has_mag)     12 bytes (Vec3 field)
//   TempData[num_temps]      each = 4 bytes (float temperature)

static constexpr uint32_t SIM_MAGIC   = 0x424D4953;
static constexpr uint32_t HEADER_SIZE = 16;

static File      simFile;
static int       simCount    = 0;
static int       simNumIMUs  = 0;
static int       simNumBaros = 0;
static int       simHasMag   = 0;
static int       simNumTemps = 0;
static uint32_t  frameStride = 0;

static int        lastTick     = -1;
static SensorData cachedFrame{};

bool simInit() {
    simFile = SD.open("SIM.BIN", FILE_READ);
    if (!simFile) return false;

    uint32_t magic, count;
    float    rate;
    uint8_t  nimus, nbaros, hmag, ntemps;

    if (simFile.read(&magic,  4) != 4 || magic != SIM_MAGIC) { simFile.close(); return false; }
    if (simFile.read(&count,  4) != 4) { simFile.close(); return false; }
    if (simFile.read(&rate,   4) != 4) { simFile.close(); return false; }
    if (simFile.read(&nimus,  1) != 1) { simFile.close(); return false; }
    if (simFile.read(&nbaros, 1) != 1) { simFile.close(); return false; }
    if (simFile.read(&hmag,   1) != 1) { simFile.close(); return false; }
    if (simFile.read(&ntemps, 1) != 1) { simFile.close(); return false; }

    simCount    = (int)count;
    simNumIMUs  = (int)nimus;
    simNumBaros = (int)nbaros;
    simHasMag   = (int)hmag;
    simNumTemps = (int)ntemps;
    frameStride = (uint32_t)(simNumIMUs  * sizeof(IMUData) +
                             simNumBaros * sizeof(BarometerData) +
                             simHasMag   * sizeof(MagData) +
                             simNumTemps * sizeof(TempData));

    return simCount > 0 && frameStride > 0;
}

int getSimLength() { return simCount; }

static void loadFrame(int tick) {
    if (tick == lastTick) return;

    if (tick != lastTick + 1) {
        if (!simFile.seek(HEADER_SIZE + (uint32_t)tick * frameStride)) return;
    }

    SensorData d{};
    d.timestamp_us = 0;

    for (int i = 0; i < simNumIMUs; i++)
        simFile.read(&d.imu[i], sizeof(IMUData));
    for (int i = simNumIMUs; i < 4; i++)
        d.imu[i] = nanIMU();

    for (int i = 0; i < simNumBaros; i++)
        simFile.read(&d.baro[i], sizeof(BarometerData));
    for (int i = simNumBaros; i < 2; i++)
        d.baro[i] = nanBaro();

    if (simHasMag)
        simFile.read(&d.mag, sizeof(MagData));
    else
        d.mag = nanMag();

    for (int i = 0; i < simNumTemps; i++)
        simFile.read(&d.tmp[i], sizeof(TempData));
    for (int i = simNumTemps; i < 2; i++)
        d.tmp[i] = nanTemp();

    cachedFrame = d;
    lastTick    = tick;
}

SensorData getSimData(int tick) {
    if (tick < 0)         tick = 0;
    if (tick >= simCount) tick = simCount - 1;
    loadFrame(tick);
    return cachedFrame;
}

#endif // SIM_MODE
