/** communication.h
* ===========================================================
* Name: Flight Controller Communication Interface
* Section: TVC499
* Project: Flight Controller
* Purpose: LoRa radio communication and serial interface
* ===========================================================
*/

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>
#include "params.h"

extern double lastTelemetryTime;


/**
 * @brief Initialize XBee communication on Serial1
 * @return True if initialization successful
 */
bool initializeCommunication();

/**
* @brief Check for commands from XBee radio (Transparent Mode)
* @return Command string received from XBee radio
 */
String checkForCommands();

/**
 * @brief Send ACK packet
 * @param ackNum ACK number from ground station
 */
void sendAck(int ackNum);

/**
 * @brief Send telemetry data over XBee (Transparent Mode)
 * @param pt Pressure transducer value (PSI)
 * @param tankThrust Tank Load Cell value (lbs)
 * @param engineThrust Engine Load Cell value (lbs)
 * @param cont1 Continuity 1 value
 * @param cont2 Continuity 2 value
 * @param state System state (Enum)
 */
void sendData(double pt, double tankThrust, double engineThrust, double cont1, double cont2, int state);


#endif // COMMUNICATION_H