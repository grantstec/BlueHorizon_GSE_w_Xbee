#include <Arduino.h>
#include "../include/radio.h"
#include "../include/params.h"

#define XBeeSerial Serial1

bool initializeCommunication() {
    // Initialize Serial1 for XBee at 9600 baud
    XBeeSerial.begin(9600); 
    
    // Explicitly flush the buffer and wait
    delay(100);
    while (XBeeSerial.available()) XBeeSerial.read();

    XBeeSerial.setTimeout(5); // Set timeout to 5ms for non-blocking read
    
    // Check if port is open (standard verification)
    if (!XBeeSerial) {
        Serial.println("Error: Serial1 could not be opened.");
        return false;
    }

    Serial.println("XBee radio initialized on Serial1 (Transparent Mode)");
    // Send a startup packet to verify TX
    XBeeSerial.println("STARTUP_OK"); 
    return true;
}

String checkForCommands() {
    String command = ""; 
    
    // Check if data is available on the serial port
    if (XBeeSerial.available()) {
        // Read everything available to debug
        // String received = XBeeSerial.readStringUntil('\n'); 
        
        // Let's read raw bytes to see if it's garbage or API frames
        String received = "";
        while (XBeeSerial.available()) {
            char c = XBeeSerial.read();
            // If we are in transparent mode, we expect ASCII command frames
            // But let's swallow control chars like CR/LF if they are isolated
            if (c != '\r' && c != '\n') {
                 received += c;
            }
            delay(1); // Small delay to allow buffer to fill
        }

        received.trim(); 
        
        if (received.length() > 0) {
            Serial.print("XBee Raw Read: '");
            Serial.print(received);
            Serial.println("'");
            
            // Filter out echo or AT responses
            if (received == "OK") {
                Serial.println("Warning: XBee sent 'OK' (Command Mode Response?). Ignoring.");
                return ""; 
            }
            
            command = received;
        }
    }
    
    return command; 
}



void sendAck(int ackNum) {
    XBeeSerial.print("ACK,");
    XBeeSerial.println(ackNum);
}

void sendData(double pt, double tankThrust, double engineThrust, double cont1, double cont2, int state) {
    // Create a buffer to hold the data
    char buf[128]; 
    // Format the data into a CSV string using only numbers
    // Order: PT, Tank, Engine, Cont1, Cont2, State
    // Example: 2500.0,10.5,50.2,1,1,1
    int len = snprintf(buf, sizeof(buf), "%.1f,%.1f,%.1f,%.0f,%.0f,%d", 
                       pt, tankThrust, engineThrust, cont1, cont2, state);
    
    if (len > 0) {
        // Transparent mode: Just print to the serial port!
        XBeeSerial.println(buf);
    }
}

