
#include <HkxPosControl.h>

void setup() {
  HkxPrint printout = HkxPrint();  // No printout with Arduino UNO
  HkxCommunication communication = HkxCommunication(HKX_115200, Serial, printout);  // Communication with the servo on Serial1
  HkxPosControl servo(253, communication, printout);  // control position for the servo ID=253 (factory default value)

  delay(10000); // wait 10 seconds

  servo.setTorqueLEDControl(HKX_NO_VALUE, HKX_LED_RED);  // set the LED to red
  delay(10000); // wait 10 seconds

  servo.movePosition(450, 2000, HKX_LED_BLUE, true);  // set the servo to 45° in 2 seconds, led turns to blue during the move

  uint16_t inputVoltage;
  int16_t position;
  // get the current behaviour of the servo
  servo.getBehaviour(&inputVoltage, HKX_NO_VALUE, &position, HKX_NO_VALUE, HKX_NO_VALUE, HKX_NO_VALUE, HKX_NO_VALUE, HKX_NO_VALUE);
  // do whatever you need with inputVoltage and position variables
}

void loop() {
  // put your main code here, to run repeatedly:

}
