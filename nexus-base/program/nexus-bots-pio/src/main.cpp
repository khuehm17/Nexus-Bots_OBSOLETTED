// #include <PinChangeIntConfig.h>
#include <PID_Beta6.h>
#include <MotorWheel.h>
#include "main.h"

#define ROSSER_TEST

/***************************************************************************
 * ROSSER_TEST
 **************************************************************************/
#ifdef ROSSER_TEST
#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle hNode;

std_msgs::String strMessage;
ros::Publisher chatter("Chatter", &strMessage);

char msgHello[13] = "hello world";

void setup()
{
  hNode.initNode();
  hNode.advertise(chatter);
}

void loop()
{
  strMessage.data = msgHello;
  chatter.publish(&strMessage);
  hNode.spinOnce();
  delay(500);
}

#endif // ROSSER_TEST

/***************************************************************************
 * RS485_TEST
 **************************************************************************/
#ifdef RS485_TEST

#define MASTER_EN   13   // connected to RS485 Enable pin

void setup() {
  pinMode(MASTER_EN , OUTPUT);      // Declare Enable pin as output
  Serial.begin(19200);               // set serial communication baudrate 
  digitalWrite(MASTER_EN , LOW);    // Make Enable pin low
                                    // Receiving mode ON 
}

void loop() {
  digitalWrite(MASTER_EN , HIGH);     // Make Enable pin high to send Data
  delay(5);                           // required minimum delay of 5ms
  Serial.println("Hello");                // Send character A serially
  Serial.flush();                     // wait for transmission of data
  delay(1000);
  digitalWrite(MASTER_EN, LOW);      // Receiving mode ON
  delay(1000);
  // digitalWrite(MASTER_EN , HIGH);
  // delay(1000);
  // digitalWrite(MASTER_EN, LOW);
  // delay(1000);
}

#endif // RS485_TEST

/***************************************************************************
 * DRIVE_TEST
 **************************************************************************/
#ifdef DRIVE_TEST
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
  if (millis() % 5 == 0)
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

#endif // DRIVE_TEST
