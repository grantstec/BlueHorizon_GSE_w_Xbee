#include "../include/sensors.h"
#include <Wire.h>

Adafruit_ADS1115 ads;

bool initSensors() {
    // Initialize I2C1 (Wire1) for Teensy 4.1 pins 16 (SCL1) and 17 (SDA1)
    Wire1.begin();

    // Initialize ADS1115 on Wire1 (Address 0x48)
    if (!ads.begin(ADS1115_I2C_ADDR, &Wire1)) {
        Serial.println("Failed to initialize ADS1115!");
        return false;
    }
    
    // Set Data Rate (defined in params.h)
    // Options: RATE_ADS1115_8SPS, 16SPS, 32SPS, 64SPS, 128SPS, 250SPS, 475SPS, 860SPS
    ads.setDataRate(ADS1115_DATA_RATE);

    Serial.println("ADS1115 initialized.");
    return true;
}

void readSensors(double& pt, double& tankThrust, double& engineThrust) {
    // Read raw values (16-bit signed)
    int16_t adc0 = ads.readADC_SingleEnded(0); // Tank Load Cell (A0)
    int16_t adc1 = ads.readADC_SingleEnded(1); // Engine Load Cell (A1)
    int16_t adc2 = ads.readADC_SingleEnded(2); // Pressure Transducer (A2)
    int16_t adc3 = ads.readADC_SingleEnded(3); // Unused

    // Convert to Volts
    float volts0 = ads.computeVolts(adc0);
    float volts1 = ads.computeVolts(adc1);
    float volts2 = ads.computeVolts(adc2);
    float volts3 = ads.computeVolts(adc3);

    // DEBUG: Print Raw Voltages to Serial (Teensy USB)
    // Format: "RAW_VOLTS: A0=... A1=... A2=... A3=..."
    Serial.print("RAW_VOLTS: A0(Tank)="); Serial.print(volts0, 4);
    Serial.print(" A1(Eng)="); Serial.print(volts1, 4);
    Serial.print(" A2(PT)="); Serial.print(volts2, 4);
    Serial.print(" A3(Unused)="); Serial.println(volts3, 4);

    // Tank Load Cell Calculation (A0)
    float tankVolts = volts0 * TANK_DIVIDER_RATIO;
    tankThrust = tankVolts * TANK_LC_CALIBRATION;

    // Engine Load Cell Calculation (A1)
    float engineVolts = volts1 * ENGINE_DIVIDER_RATIO;
    engineThrust = engineVolts * ENGINE_LC_CALIBRATION;

    // Pressure Transducer Calculation (A2)
    pt = volts2 * PT_CALIBRATION;
}

void readContinuity(double& c1, double& c2) {
    // Manual check using raw analog reads
    float contRaw1 = analogRead(LAUNCH_PYRO_CONT) * (3.3 / 1023.0);
    float contRaw2 = analogRead(SEP_PYRO_CONT) * (3.3 / 1023.0);
    
    c1 = (contRaw1 > CONTINUITY_THRESHOLD) ? 1.0 : 0.0;
    c2 = (contRaw2 > CONTINUITY_THRESHOLD) ? 1.0 : 0.0;
}