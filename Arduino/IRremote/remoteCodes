#include <IRremote.h>

#define irpin 3 

void setup() {
  Serial.begin(9600);
  IrReceiver.begin(irpin, ENABLE_LED_FEEDBACK);
  
}

void loop() {
  if (IrReceiver.decode()) {
    Serial.print("IR Code: 0x");
    Serial.println(IrReceiver.decodedIRData.command, HEX);
    IrReceiver.resume();
  }
}
