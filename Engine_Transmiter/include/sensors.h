#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include "params.h"

// External object if needed elsewhere, otherwise keep internal to .cpp
extern Adafruit_ADS1115 ads;

/**
 * @brief Initialize the ADS1115 sensor on Wire1 (I2C1)
 * @return true if successful, false otherwise
 */
bool initSensors();

/**
 * @brief Reads all sensors and updates the passed variables by reference
 * @param pt Pressure Transducer value (out)
 * @param tankThrust Tank Load Cell value (out)
 * @param engineThrust Engine Load Cell value (out)
 */
void readSensors(double& pt, double& tankThrust, double& engineThrust);

/**
 * @brief Checks continuity on pyro channels
 * @param c1 Continuity 1 status (1.0 or 0.0) (out)
 * @param c2 Continuity 2 status (1.0 or 0.0) (out)
 */
void readContinuity(double& c1, double& c2);

#endif
