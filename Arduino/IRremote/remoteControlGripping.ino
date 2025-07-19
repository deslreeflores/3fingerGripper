#include <IRremote.h>
#include <Servo.h>

#define irpin 3
#define servoPin 9

// IR button codes 
#define closeButton 0x40  // Clockwise to close
#define openButton  0x43  // Counterclockwise to open

Servo gripper;

int currentAngle = 90;           // Start centered
int lastCommand = -1;            // Track last command code
unsigned long lastPressTime = 0; // Track how long button is held
const int stepDelay = 100;       // How often to move (ms)
const int stepSize = 1;          // Move angle by 1° per step

void setup() {
  Serial.begin(9600);
  IrReceiver.begin(irpin, ENABLE_LED_FEEDBACK);
  
  gripper.attach(servoPin);
  gripper.write(currentAngle);

// ready command
  Serial.println("Gripper is ready to move ");
}

void loop() {
  if (IrReceiver.decode()) {
    uint8_t cmd = IrReceiver.decodedIRData.command;

    // Reset press tracking when new button is pressed
    if (cmd != lastCommand) {
      lastPressTime = millis();
      lastCommand = cmd;
    }

    // Handle OPEN 
    if (cmd == openButton) {
      Serial.println("Opening");
      if (millis() - lastPressTime >= stepDelay) {
        currentAngle -= stepSize;
        currentAngle = constrain(currentAngle, 0, 180);
        gripper.write(currentAngle);
        lastPressTime = millis();
      //  Serial.print("Angle: "); 
      //  Serial.println(currentAngle); 
      }
    }

    // Handle CLOSE 
    else if (cmd == closeButton) {
      Serial.println("Closing");
      if (millis() - lastPressTime >= stepDelay) {
        currentAngle += stepSize;
        currentAngle = constrain(currentAngle, 0, 180);
        gripper.write(currentAngle);
        lastPressTime = millis();
  //      Serial.print("Angle: ");
  //      Serial.println(currentAngle);
      }
    }

    IrReceiver.resume();  // Ready for next
  }
}
