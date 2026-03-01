#include <Arduino.h>
#include <radio.h>
#include <params.h>
#include <hardware.h>
#include <sensors.h>
#include <PWMServo.h> 

PWMServo Fuel_Servo;  
PWMServo Ox_Servo;  
PWMServo Ox_load; 
PWMServo CO2_Servo;  
PWMServo Cam_Servo;

SystemState systemState = UNARMED;

double continuity1 = 0.0; 
double continuity2 = 0.0;
double PT = 0.0; 
double tankThrust = 0.0;
double engineThrust = 0.0;
double initialTankWeight = 0.0; // Captured at ARM

unsigned long load_timer = 0; 
unsigned long servo_timer = 0; 
unsigned long CO2_timer = 0; 
unsigned long cam_timer = 0;
bool camservo_fired = false;

unsigned long last_telemetry_time = 0;

int count = 0; 


void setup() {

  pinMode(LAUNCH_PYRO_CONT, INPUT); 
  pinMode(SEP_PYRO_CONT, INPUT);

  pinMode(LAUNCH_PYRO_FIRE, OUTPUT);
  pinMode(SEP_PYRO_FIRE, OUTPUT);
  digitalWrite(LAUNCH_PYRO_FIRE, LOW); 
  digitalWrite(SEP_PYRO_FIRE, LOW);

  Serial.begin(9600);
  
  if (!initSensors()) {
      Serial.println("Sensors init failed!");
  }

  Fuel_Servo.attach(8);  // J2
  Ox_Servo.attach(9);  // J1
  Ox_load.attach(7); //J3
  CO2_Servo.attach(6);   //J4
  Cam_Servo.attach(24);  //J5

  Fuel_Servo.write(8); //5 close 70 open
  Ox_Servo.write(5); //5 close 70 open
  Ox_load.write(5);  //5 close 70 open
  CO2_Servo.write(0); //0 close 65 open
  Cam_Servo.write(70); 


  Serial.println("System initialized, waiting for commands...");

  if (!initializeCommunication()) {
    Serial.println("XBee radio init failed!");
    while (1);
  }
  Serial.println("XBee radio init OK!");
  
  // Send initial packets to wake up radio or verify comms
  delay(100);
  sendData(0, 0, 0, 0, 0, 0); 
  delay(100);

}

void loop() {
    String commandStr = checkForCommands();
    int commandVal = -1; // -1 indicates no valid command

    if (commandStr.length() > 0) {
      Serial.print("Data received: ");
      Serial.println(commandStr);
      
      // Handle "CMD,[CODE],[ID]" format e.g. "CMD,1,10" (1=ARM, 10=ID)
      if (commandStr.startsWith("CMD,")) {
          int firstComma = commandStr.indexOf(',');
          int secondComma = commandStr.indexOf(',', firstComma + 1);
          
          if (secondComma > 0) {
              String opStr = commandStr.substring(firstComma + 1, secondComma);
              opStr.trim(); 
              commandVal = opStr.toInt(); // Convert command logic to int
              
              String idStr = commandStr.substring(secondComma + 1);
              idStr.trim(); 
              int msgId = idStr.toInt();
              
              sendAck(msgId); // Send "ACK,[ID]" immediately
              
              Serial.print("Parsed Op Code: ");
              Serial.println(commandVal);
          }
      }
    }

    // Read Sensors
    readSensors(PT, tankThrust, engineThrust);
    readContinuity(continuity1, continuity2);

    // Send Telemetry at 100Hz
    if (millis() - last_telemetry_time >= TELEMETRY_INTERVAL) {
        last_telemetry_time = millis();
        // Send strictly numeric CSV data
        sendData(PT, tankThrust, engineThrust, continuity1, continuity2, (int)systemState);
        // Debug: Confirm main loop is running
        // Serial.println("Telemetry Sent"); 
    }

    // Logic State Machine
    
    // Continuous Weight Check (Exit LOADING if weight changes by >= 5.0 lbs from ARM value)
    if (systemState == LOADING) {
       if (abs(tankThrust - initialTankWeight) >= 5.0) { 
          systemState = LOADED;
          Ox_load.write(5); 
          Serial.println("Loading Complete (Weight Trigger)");
       }
    }

    if (commandVal != -1) { 
        
      if(systemState == UNARMED && commandVal != CMD_ARM) {
         // Do nothing or log error
      }
      else if (commandVal == CMD_ARM) {
        systemState = ARMED;
        initialTankWeight = tankThrust; 
        Serial.print("Armed. Initial Tank Weight: ");
        Serial.println(initialTankWeight);
      }
      // DISARM
      else if (commandVal == CMD_DISARM) {
        systemState = UNARMED;
        Fuel_Servo.write(5); 
        Ox_Servo.write(5); 
        Ox_load.write(5); 
        CO2_Servo.write(0); 
        camservo_fired = false;
      }
      // LOAD
      else if (systemState == ARMED && commandVal == CMD_LOAD) {
        systemState = LOADING;
        Ox_load.write(70); 
        load_timer = millis();
      }
      // LOADED (Manual Command Only)
      else if (systemState == LOADING && commandVal == CMD_LOADED) {
        systemState = LOADED;
        Ox_load.write(5); 
      }
      // FIRE
      else if (commandVal == CMD_FIRE && systemState != FIRE) {
        systemState = FIRE;

        servo_timer = millis(); // Start of FIRE sequence (T=0)
        cam_timer = millis();
        
        triggerFire(); // Fire pyros immediately
        // No blocking delay here!
        
        camservo_fired = true;
      }
      // PURGE
      else if (commandVal == CMD_PURGE) {
        CO2_Servo.write(65); 
        CO2_timer = millis();
        systemState = PURGE; 
      }
      // PURGED
      else if (commandVal == CMD_PURGED) {
        systemState = PURGED;
        CO2_Servo.write(0); 
      }
      else {
          // Unknown or invalid command for current state
      }
    }
    
    // --- FIRE SEQUENCING LOGIC (Non-blocking) ---
    if (systemState == FIRE) {
        unsigned long elapsed = millis() - servo_timer;

        // 1. Pyros (Turn off after 2.5s)
        if (elapsed > 2500) {
            digitalWrite(LAUNCH_PYRO_FIRE, LOW); 
            digitalWrite(SEP_PYRO_FIRE, LOW);
        }

        // 2. Servo Opening Logic (Start after 1s delay, complete over 3s -> T=1s to T=4s)
        if (elapsed >= 1000 && elapsed <= 4000) {
            // Linear Servo Opening
            // Progress 0.0 to 1.0 over the 3 second window
            float progress = (float)(elapsed - 1000) / 3000.0; 
            
            // Map 0.0-1.0 to Angle range (5 to 65) linearly
            int startAngle = 5;
            int endAngle = 65;
            int currentAngle = startAngle + (int)((endAngle - startAngle) * progress); 
            
            Fuel_Servo.write(currentAngle);
            Ox_Servo.write(currentAngle);
        } 
        else if (elapsed > 4000 && elapsed < 15000) {
            // Ensure fully open after the ramp in case loop timing missed the exact end
             Fuel_Servo.write(65);
             Ox_Servo.write(65);
        }

        // 3. Cam Servo Oscillation (Every 1.5s)
        if (millis() - cam_timer >= 1500) {
            cam_timer = millis();
            // Toggle cam servo
            if (camservo_fired) {
                Cam_Servo.write(70);
                camservo_fired = false;
            } else {
                Cam_Servo.write(60);
                camservo_fired = true;
            }
        }

        // 4. Auto-Shutdown (15s)
        if (elapsed > 15000) {
            Fuel_Servo.write(5); 
            Ox_Servo.write(5); 
        }
    }

}




