#include "Teensy-ICM-20948.h"
#include <time.h> // TODO: change to teensy4 Time header


struct SensorData {
  time_t timestamp;
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float magX, magY, magZ;
  float temperature;
};



