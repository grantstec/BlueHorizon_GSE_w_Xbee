/** hardware.h
* ===========================================================
* Name: Flight Controller Hardware Interface
* Section: TVC499
* Project: Flight Controller
* Purpose: LED and pyro channel control
* ===========================================================
*/

#ifndef HARDWARE_H
#define HARDWARE_H

#include <Arduino.h>
#include <FastLED.h>



/**
 * @brief Play alert tone using the buzzer
 * @param frequency Frequency of the tone in Hz
 * @param duration Duration of the tone in milliseconds
 */
void playAlertTone(int frequency, int duration);

/**
 * @brief Start a non-blocking buzzer tone
 * @param frequency Frequency of the tone in Hz
 */
void startBuzzerTone(int frequency);

/**
 * @brief Stop the non-blocking buzzer tone
 */
void stopBuzzerTone();

/**
 * @brief Initialize hardware components (LEDs, pyro channels, buzzer)
 */
void initializeHardware();

/**
 * @brief Check pyro continuity and buzz if continuity is detected
 * @return True if continuity is detected, false otherwise
 */

void checkPyroContinuity(double* continuity);


/**
 * @brief Trigger launch sequence (fire pyro channels 1)
 * @param launchTriggered Pointer to launch flag
 */
void triggerFire();

void checkPT(double* pt);

void countDownMusic();


#endif // HARDWARE_H