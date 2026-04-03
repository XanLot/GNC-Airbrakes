#ifdef SIM_MODE
#ifndef SIM_DATA_HPP
#define SIM_DATA_HPP

#include "sensor_data.hpp"

// Initialize sim from SIM.BIN on SD card.
// Must be called after sdLog.init() (which calls SD.begin()).
// Returns false if SIM.BIN is missing or has an invalid header.
bool simInit();

// Total frames available in SIM.BIN (read from header).
int getSimLength();

// Get sensor data for a given tick.
// Fills only the slots declared in the SIM.BIN header; remaining slots are NaN.
// Clamps tick to [0, getSimLength()-1].
SensorData getSimData(int tick);

#endif // SIM_DATA_HPP
#endif // SIM_MODE
