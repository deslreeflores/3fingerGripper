#include <Servo.h>
#include <SoftwareSerial.h>

#define rede 2
SoftwareSerial rs485(10, 11); 

Servo gripper;
const int gripperPin = 9;
String incoming = "";

void setup() {
  pinMode(rede, OUTPUT);
  // (RO active)
  digitalWrite(rede, LOW);  

  Serial.begin(9600);       
  rs485.begin(9600);         

  gripper.attach(gripperPin);
   // Start in neutral
  gripper.write(90); 

  Serial.println("Gripper ready ");
}

void loop() {
  while (rs485.available()) {
    char ch = rs485.read();

    if (ch == '\n') {
      processCommand(incoming);
      // Reset buffer
      incoming = "";  
    } else {
      incoming += ch;
    }
  }
}

void processCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();

  Serial.print("Received command: '");
  Serial.print(cmd);
  Serial.println("'");

  if (cmd == "open") {
    Serial.println("Opening gripper...");
    gripper.write(180);
      // Fully open
  } else if (cmd == "close") {
    Serial.println("Closing gripper...");
    gripper.write(0);   
     // Fully closed
  } else if (cmd == "stop") {
    Serial.println("Stopping gripper...");
    gripper.write(90); 
   // Neutral
  } else {
    Serial.println("Command unknown.");
  }
}
