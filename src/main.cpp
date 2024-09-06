#include <Arduino.h>
#include <esp32_can.h>

#include "boardconfig.h"

short int previous_steering_angle = 0;
short int current_steering_angle = 0;
short int target_steering_angle = 0;
bool current_steering_angle_valid = false;

long last_serialupdate = 0;
long last_steeringupdate = 0;
long last_speedupdate = 0;

// Globals for CANBUS debug / testing
uint32_t counter = 0;

uint16_t user_speed = 0;
uint16_t user_rpm = 0;
uint16_t user_steer = 0;

// Function to process serial commands
void processCommand(const String& command) {
    // Split the command by spaces
    int firstSpace = command.indexOf(' ');
    int secondSpace = command.indexOf(' ', firstSpace + 1);

    // Extract the command type
    String commandType = command.substring(0, firstSpace);
    
    // Extract the parameters
    String param1 = command.substring(firstSpace + 1, secondSpace);
    String param2 = (secondSpace == -1) ? "" : command.substring(secondSpace + 1);

    // Process the command
    if (commandType == "SET") {
        if (param1 == "SPEED") {
            user_speed = param2.toInt();
            Serial.print("Speed updated to: ");
            Serial.println(user_speed);
        } else if (param1 == "RPM") {
            user_rpm = param2.toInt();
            Serial.print("RPM updated to: ");
            Serial.println(user_rpm);
        } else if (param1 == "STEER") {
            user_steer = param2.toInt();
            Serial.print("Steering updated to: ");
            Serial.println(user_steer);
        } else {
            Serial.println("Unknown variable");
        }
    } else {
        Serial.println("Unknown command");
    }
}

void printFrame(CAN_FRAME *message)
{
  Serial.print(message->id, HEX);
  if (message->extended) Serial.print(" X ");
  else Serial.print(" S ");   
  Serial.print(message->length, DEC);
  Serial.print(" ");
  for (int i = 0; i < message->length; i++) {
    Serial.print(message->data.byte[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void setup()
{
  Serial.begin(115200);

  CAN0.setCANPins(CANRX, CANTX); //Set CAN Bus Pins
  if (!CAN0.begin(500000)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }
  Serial.println("CAN Bus Initialized");

  CAN0.watchFor(); //watch for any CAN traffic
}

// Main program loop.  Poll both CAN networks.  If 0x201 received from vehicle network, modify and rebroadcast to pump network.

void loop()
{
  if (millis() > last_serialupdate + 100) {
    last_serialupdate = millis();
    if (Serial.available()) {
        // Read the incoming byte
        String command = Serial.readStringUntil('\n');
        command.trim();  // Trim any trailing newline or spaces
        processCommand(command);
    }
  }

  CAN_FRAME message;
  if (CAN0.read(message)) {

    //printFrame(&message);
  }

  CAN_FRAME txFrame;

  // Update vehicle and engine speed at 4Hz
  if (millis() > last_speedupdate + 250) {
          last_speedupdate = millis();
          uint16_t vehicle_speed = user_speed;
          uint16_t engine_speed = user_rpm;
          // Mazda 3 pump expects RPM as raw bytes, no scalar like MX5
          // Mazda 3 pump expects km/hr*100, no 100 km offset like MX5
          vehicle_speed *= 100;

          // Write modified values back into buffer
          txFrame.rtr = 0;
          txFrame.id = 0x201;
          txFrame.extended = false;
          txFrame.length = 8;
          txFrame.data.uint8[0] = engine_speed / 256;
          txFrame.data.uint8[1] = engine_speed % 256;
          txFrame.data.uint8[4] = vehicle_speed / 256;
          txFrame.data.uint8[5] = vehicle_speed % 256;

          //Serial.print("Sending to pump CAN: ");
          //printFrame(&txFrame);
          CAN0.sendFrame(txFrame);
  }
  //Update steering delta at 20Hz
  if (millis() > last_steeringupdate + 50) { 
          last_steeringupdate = millis();
          // Convert from steering angle message format to degrees
          //uint16_t u_steering_angle = (steering_angle * 10 ) + 1600;

          // Send canbus message for 2 plug Mazda 3 pump (2009+)
          // byte 6 = steering rate of change.  Pump reaches max speed when delta is 0x10. byte 1 must be nonzero? more than 1?
          CAN_FRAME txSteer;
          txSteer.rtr = 0;
          txSteer.id = 0x82;
          txSteer.extended = false;
          txSteer.length = 8;
          txSteer.data.uint8[1] = 50; //byte 1 must not be zero
          txSteer.data.uint8[6] = user_steer; //full rate of 0x10, this value can be calculated with real input
          //printFrame(&txSteer);
          CAN0.sendFrame(txSteer);
  }
}