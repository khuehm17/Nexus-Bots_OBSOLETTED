// #include <PinChangeIntConfig.h>
#include <PID_Beta6.h>
#include <MotorWheel.h>
#include "main.h"

void setup()
{
  TCCR1B = TCCR1B & (0xf8 | 0x01); // Pin9,Pin10 PWM 31250Hz, Silent PWM
  TCCR2B = TCCR2B & (0xf8 | 0x01);
  wheel1.PIDEnable(KC, TAUI, TAUD, 10); // used wheel1 to call the PIDEnable
  wheel2.PIDEnable(KC, TAUI, TAUD, 10); // used wheel2 to call the PIDEnable
  wheel3.PIDEnable(KC, TAUI, TAUD, 10); // used wheel3 to call the PIDEnable
  Serial.begin(19200);
}

void loop()
{
  wheel1.setSpeedMMPS(500, DIR_ADVANCE); // Set the pwm speed 100 direction
  wheel1.PIDRegulate();                  // regulate the PID
  wheel2.setSpeedMMPS(500, DIR_ADVANCE); // Set the pwm speed 100 direction
  wheel2.PIDRegulate();                  // regulate the PID
  wheel3.setSpeedMMPS(300, DIR_ADVANCE); // Set the pwm speed 100 direction
  wheel3.PIDRegulate();                  // regulate the PID
  if (millis() % 500 == 0)
  {
    //    Serial.print("speedRPM1>");
    //    Serial.println(wheel1.getSpeedRPM(),DEC);//display the speed of the MotorWheel
    //    Serial.print("MMPS1 -->");
    //    Serial.println(wheel1.getSpeedMMPS(),DEC);//display the speed of the motor
    //    Serial.println("");
    //    Serial.print("speedRPM2>");
    //    Serial.println(wheel2.getSpeedRPM(),DEC);//display the speed of the MotorWheel
    //    Serial.print("MMPS2 -->");
    //    Serial.println(wheel2.getSpeedMMPS(),DEC);//display the speed of the motor
    //    Serial.println("");

    Serial.print(wheel1.getSpeedRPM(), DEC); // display the speed of the MotorWheel
    Serial.print(",");
    Serial.print(wheel1.getSpeedMMPS(), DEC); // display the speed of the motor
    Serial.print(",");
    Serial.print(wheel2.getSpeedRPM(), DEC); // display the speed of the MotorWheel
    Serial.print(",");
    Serial.print(wheel2.getSpeedMMPS(), DEC); // display the speed of the motor
    Serial.print("\n");
    // wheel1.debugger();
  }
}