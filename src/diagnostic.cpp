#ifdef DIAGNOSTIC_MODE

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <SparkFun_LSM6DSV16X.h>
#include <SparkFun_BMP581_Arduino_Library.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>
#include <SparkFun_TMP117.h>

// raw SPI WHO_AM_I probe — doesn't need library init
// LSM6DSV16X: reg 0x0F should return 0x70
// BMP581:     reg 0x01 should return 0x50
static uint8_t spiReadByte(SPIClass& bus, uint8_t cs, uint8_t reg) {
    SPISettings s(1000000, MSBFIRST, SPI_MODE3);
    bus.beginTransaction(s);
    digitalWrite(cs, LOW);
    bus.transfer(reg | 0x80);  // read bit
    uint8_t val = bus.transfer(0x00);
    digitalWrite(cs, HIGH);
    bus.endTransaction();
    return val;
}

static void probeIMU(SPIClass& bus, const char* busName, uint8_t cs, const char* label) {
    pinMode(cs, OUTPUT);
    digitalWrite(cs, HIGH);
    delay(10);

    uint8_t whoami = spiReadByte(bus, cs, 0x0F);
    Serial.print("  ");
    Serial.print(label);
    Serial.print(" (CS=");
    Serial.print(cs);
    Serial.print(", ");
    Serial.print(busName);
    Serial.print("): WHO_AM_I=0x");
    Serial.print(whoami, HEX);

    if (whoami == 0x70) {
        Serial.println(" -> LSM6DSV16X OK");
    } else if (whoami == 0xFF || whoami == 0x00) {
        Serial.println(" -> no response");
    } else {
        Serial.println(" -> unexpected value");
    }

    // also try library init
    SparkFun_LSM6DSV16X_SPI sensor;
    SPISettings settings(4000000, MSBFIRST, SPI_MODE3);
    bool ok = sensor.begin(bus, settings, cs);
    Serial.print("    library begin: ");
    Serial.println(ok ? "OK" : "FAILED");
}

static void probeBaro(SPIClass& bus, const char* busName, uint8_t cs, const char* label) {
    pinMode(cs, OUTPUT);
    digitalWrite(cs, HIGH);
    delay(10);

    uint8_t chipid = spiReadByte(bus, cs, 0x01);
    Serial.print("  ");
    Serial.print(label);
    Serial.print(" (CS=");
    Serial.print(cs);
    Serial.print(", ");
    Serial.print(busName);
    Serial.print("): CHIP_ID=0x");
    Serial.print(chipid, HEX);

    if (chipid == 0x50) {
        Serial.println(" -> BMP581 OK");
    } else if (chipid == 0xFF || chipid == 0x00) {
        Serial.println(" -> no response");
    } else {
        Serial.println(" -> unexpected value");
    }

    BMP581 baro;
    int8_t err = baro.beginSPI(cs, 1000000, bus);
    Serial.print("    library begin: ");
    Serial.print(err == BMP5_OK ? "OK" : "FAILED");
    Serial.print(" (err=");
    Serial.print(err);
    Serial.println(")");
}

static void i2cScan() {
    Serial.println("\n--- I2C scan (Wire, SDA=18, SCL=19) ---");
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        uint8_t err = Wire.endTransmission();
        if (err == 0) {
            Serial.print("  device at 0x");
            Serial.print(addr, HEX);
            if (addr == 0x30) {
                Serial.print(" -> MMC5983MA");
            } else if (addr == 0x48 || addr == 0x49) {
                Serial.print(" -> TMP117");
            }
            Serial.println();
            found++;
        }
    }
    if (found == 0) {
        Serial.println("  no I2C devices found");
    }
}

static void probeMag() {
    Serial.println("\n--- Magnetometer (MMC5983MA, I2C 0x30) ---");
    SFE_MMC5983MA mag;
    bool ok = mag.begin(Wire);
    Serial.print("  library begin: ");
    Serial.println(ok ? "OK" : "FAILED");
}

static void probeTMP(uint8_t addr, const char* label) {
    Serial.print("\n--- ");
    Serial.print(label);
    Serial.print(" (TMP117, I2C 0x");
    Serial.print(addr, HEX);
    Serial.println(") ---");
    TMP117 tmp;
    bool ok = tmp.begin(addr, Wire);
    Serial.print("  library begin: ");
    if (ok) {
        Serial.println("OK");
        Serial.print("  temp: ");
        Serial.print(tmp.readTempC());
        Serial.println(" C");
    } else {
        Serial.println("FAILED");
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n========== GNC SENSOR DIAGNOSTIC ==========");

    SPI.begin();
    SPI1.begin();
    Wire.begin();
    Wire.setClock(400000);

    // --- SPI0 sensors ---
    Serial.println("\n--- SPI0 IMUs (MOSI=11, MISO=12, SCK=13) ---");
    probeIMU(SPI, "SPI0", 37, "IMU1");
    probeIMU(SPI, "SPI0", 38, "IMU2");

    Serial.println("\n--- SPI0 Barometer ---");
    probeBaro(SPI, "SPI0", 10, "Baro1");

    // --- SPI1 sensors ---
    Serial.println("\n--- SPI1 IMUs (MOSI=26, MISO=39, SCK=27) ---");
    probeIMU(SPI1, "SPI1", 28, "IMU3");
    probeIMU(SPI1, "SPI1", 33, "IMU4");

    Serial.println("\n--- SPI1 Barometer ---");
    probeBaro(SPI1, "SPI1", 34, "Baro2");

    // --- I2C ---
    i2cScan();
    probeMag();
    probeTMP(0x48, "TMP117 (primary)");
    probeTMP(0x49, "TMP117 (secondary)");

    Serial.println("\n========== DIAGNOSTIC COMPLETE ==========");
}

void loop() {
    // nothing
}

#endif // DIAGNOSTIC_MODE
