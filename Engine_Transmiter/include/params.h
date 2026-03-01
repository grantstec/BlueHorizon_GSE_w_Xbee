#ifndef CONFIG_H
#define CONFIG_H

// Other Constants
#define TELEMETRY_INTERVAL 10 // Send serial data every 10ms for 100Hz

// Pyro Pins
#define LAUNCH_PYRO_FIRE 28
#define SEP_PYRO_FIRE 29
#define LAUNCH_PYRO_CONT A11  // continuity pyro 1
#define SEP_PYRO_CONT A12  // continuity pyro 2
#define PYRO_DURATION 2000  // 2 seconds
#define CONTINUITY_THRESHOLD 2.0 // Threshold for continuity detection (in volts)

// Buzzer Pins
#define BUZZER_HIGH 4
#define BUZZER_LOW 5

// State Definitions make systemState enum
enum SystemState {
    UNARMED = 0,
    ARMED = 1,
    LOADING = 2,
    LOADED = 3,
    FIRE = 4,
    PURGE = 5,
    PURGED = 6
};

// Command Codes (Received from GSE)
// Format: CMD,[CODE],[ID]
#define CMD_DISARM 0
#define CMD_ARM 1
#define CMD_LOAD 2
#define CMD_LOADED 3
#define CMD_FIRE 4
#define CMD_PURGE 5
#define CMD_PURGED 6

// Status Message Codes (Sent to GSE)
// Format: [PT],[Tank],[Engine],[Cont1],[Cont2],[State],[StatusMsg]
// Example: 2500.0,10.5,50.2,1,1,1,2
#define STATUS_INIT 0
#define STATUS_NOT_ARMED 1
#define STATUS_ARMED 2
#define STATUS_DISARMED 3
#define STATUS_LOADING 4
#define STATUS_LOADED 5
#define STATUS_FIRED 6
#define STATUS_PURGING 7
#define STATUS_PURGED 8
#define STATUS_FIRE_DONE 9
#define STATUS_UNKNOWN 99

#define CO2PURGE 5

// ADS1115 Configuration
#define ADS1115_I2C_ADDR 0x48 // Default address

/**
 * ADS1115 Data Rate Selection
 * 
 * Available rates (Samples Per Second):
 * RATE_ADS1115_8SPS    - 8 samples per second
 * RATE_ADS1115_16SPS   - 16 samples per second
 * RATE_ADS1115_32SPS   - 32 samples per second
 * RATE_ADS1115_64SPS   - 64 samples per second
 * RATE_ADS1115_128SPS  - 128 samples per second (Default)
 * RATE_ADS1115_250SPS  - 250 samples per second
 * RATE_ADS1115_475SPS  - 475 samples per second
 * RATE_ADS1115_860SPS  - 860 samples per second
 * 
 * Note: Faster rates increase noise. 860SPS is good if you need speed (e.g. catch peaks).
 * Since we are polling 3 channels, the effective rate per channel is Rate / 3.
 * To maintain 100Hz loop, we ideally need fast reads, but I2C transaction time also matters.
 * With 3 channels, 860SPS gives ~286Hz per channel max. 
 */
#define ADS1115_DATA_RATE RATE_ADS1115_475SPS

// Tank Load Cell (ADS1115 A1)
// Voltage Divider: R1=4.62k, R2=4.6k
// Vout = Vin * R2 / (R1 + R2) => Vin = Vout * (R1 + R2) / R2
// Factor = (4.62 + 4.6) / 4.6 = 2.0043
// Calibration: ~2 lbs/volt (applied to input voltage before divider?? User said 2lbs/volt)
// Assuming user meant 2lbs per volt read at the ADC, or 2lbs per volt from sensor?
// Usually calibration is physical unit / sensor voltage.
// Let's assume the "2lbs/volt" is the final calibration factor AFTER accounting for voltage divider or relative to the sensor output. 
// User said: "tank loadcell is about 2lbs/volt" and "engine is about 10lbs per volt"
// Voltage divider is to bring 10V down to 5V (ADS1115 max is VDD+0.3, Teensy is 3.3V logic but ADS1115 can be powered by 5V if communicating with 3.3V I2C requires level shifting or if ADS1115 is powered by 3.3V. 
// If ADS1115 powered by 3.3V, max input is 3.3V. 
// 10V * 4.6 / (4.62+4.6) = 4.98V. This is too high for a 3.3V powered ADS1115. 
// Teensy 4.1 pins 18/19 are I2C0, 16/17 are I2C1.
// User said "i2c 1 on the teensy 4.1 so pins 16 and 17".
// NOTE: ADS1115 max input voltage depends on PGA gain and Supply Voltage. 
// If VDD=5V, max input is 5V. If VDD=3.3V, max input is 3.3V. 
// I will assume the hardware is set up correctly to not fry the ADS1115.

// Calibration Factors (Units per Volt)
#define TANK_LC_CALIBRATION 2.34     // lbs/volt
#define ENGINE_LC_CALIBRATION 10.0  // lbs/volt

// Resistor Dividers for restoring original voltage
// Tank: R1=4.62k, R2=4.6k
#define TANK_DIVIDER_RATIO ((4.62 + 4.6) / 4.6) 

// Engine: R1=4.59k, R2=4.72k
#define ENGINE_DIVIDER_RATIO ((4.59 + 4.72) / 4.72)

// Pressure Transducer (ADS1115 A2)
// Calibration Points (User Adjusted):
// Original: 0V = -127.78, 10V = 2427.78 -> Offset -127.78
// Observed: 95 PSI actual reading as 88.7 PSI -> Error of -6.3 PSI
// Correction: Add 6.3 to offset -> New Offset = -121.48
#define PT_SLOPE 255.556
#define PT_OFFSET -121.48

// PT Divider: Ratio is 1.0 (No divider)
#define PT_DIVIDER_RATIO 1.0



#endif // CONFIG_H