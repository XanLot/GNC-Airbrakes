#ifndef DEBUG_MODE_HPP
#define DEBUG_MODE_HPP

#include "sensor_data.hpp"

// print all sensor data as tagged CSV lines over serial
void debugPrint(const SensorData& data);

#endif // DEBUG_MODE_HPP
