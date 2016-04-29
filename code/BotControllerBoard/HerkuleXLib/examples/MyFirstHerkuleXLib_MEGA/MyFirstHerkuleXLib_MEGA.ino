
#include <HkxPosControl.h>

void setup() {
  HkxPrint printout = HkxPrint(Serial, 9600);  // Printout errors on the serial monitor
  HkxCommunication communication = HkxCommunication(HKX_115200, Serial1, printout);  // Communication with the servo on Serial1
  HkxPosControl servo(0, communication, printout);  // control position for the servo ID=253 (factory default value)

  Serial.println(F("--> LED is off, torque is free"));
  delay(10000); // wait 10 seconds

  servo.setTorqueLEDControl(HKX_NO_VALUE, HKX_LED_RED);  // set the LED to red
  Serial.println(F("--> LED is now red"));
  delay(10000); // wait 10 seconds

  Serial.println(F("--> Move to 45 degrees in 2 seconds, blue LED"));
  servo.movePosition(450, 2000, HKX_LED_BLUE, true);  // set the servo to 45° in 2 seconds, led turns to blue during the move
  Serial.println(F("--> Move is finished, torque is free and LED back to red"));

  uint16_t inputVoltage;
  int16_t position;
  // get the current behaviour of the servo
  servo.getBehaviour(&inputVoltage, HKX_NO_VALUE, &position, HKX_NO_VALUE, HKX_NO_VALUE, HKX_NO_VALUE, HKX_NO_VALUE, HKX_NO_VALUE);
  Serial.print(F("--> Input voltage is:"));
  Serial.println((float)inputVoltage/1000);
  Serial.print(F("--> Current position (degree) is:"));
  Serial.println((float)position/10);
}

void loop() {
  // put your main code here, to run repeatedly:

}
