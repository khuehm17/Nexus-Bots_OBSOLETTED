// #include <PinChangeIntConfig.h>
#include <PID_Beta6.h>
#include <MotorWheel.h>
#include "main.h"

#define CONTROL_3W_TEST

/***************************************************************************
 * CONTROL_3W_TEST
 **************************************************************************/
#ifdef CONTROL_3W_TEST

#define SPEED_COMMON      (200)
#define	CMD_STOP          (1)
#define	CMD_ADVANCE       (2)
#define	CMD_BACKOFF       (3)
#define	CMD_LEFT          (4)
#define	CMD_RIGHT         (5)
#define	CMD_ROTATELEFT    (6)
#define	CMD_ROTATERIGHT   (7)

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt16.h>
#include <MotorWheel.h>
#include <Omni3WD.h>
#include <PID_Beta6.h>
#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>
#include <MatrixMath.h>
#include <math.h>

#define NR_OMNIWHEELS 3

float speed_kp = 0.35;
float speed_kd = 0.2;
float speed_ki = 0.01;
int sampling_time = SAMPLETIME; //ms

/* wheelBack:   wheel-2
   wheelRight:  wheel-3
   wheelLeft:   wheel-1
*/
Omni3WD Omni(&wheel2, &wheel3, &wheel1);

ros::NodeHandle hNode;

void cmdrobotCallback( const std_msgs::UInt16& cmd_msg)
{
  if (cmd_msg.data == CMD_STOP)
  {
    /* Stop */
    Omni.setCarSlow2Stop(1000);
    //Omni.setCarStop();
    Omni.setMotorAllStop();
    Omni.PIDRegulate();              // regulate the PID
  } 
  else if (cmd_msg.data == CMD_ADVANCE)
  {
    /* go straight ahead */
    Omni.setCarAdvance(SPEED_COMMON);
    Omni.PIDRegulate();
  }
  else if (cmd_msg.data == CMD_BACKOFF)
  {
    /* Back */
    Omni.setCarBackoff(SPEED_COMMON);
    Omni.PIDRegulate();
  }
  else if (cmd_msg.data == CMD_LEFT)
  {
    /* Turn left */
    Omni.setCarLeft(SPEED_COMMON);
    Omni.PIDRegulate();
  }
  else if (cmd_msg.data == CMD_RIGHT)
  {
    /* Turn Right */
    Omni.setCarRight(SPEED_COMMON);
    Omni.PIDRegulate(); 
  }
  else if (cmd_msg.data == CMD_ROTATELEFT)
  {
    /* ROTATE left */
    Omni.setCarRotateLeft(SPEED_COMMON);
    Omni.PIDRegulate();
  }
  else if (cmd_msg.data == CMD_ROTATERIGHT)
  {
    /* ROTATE right */
    Omni.setCarRotateRight(SPEED_COMMON);
    Omni.PIDRegulate();
  }  
}

ros::Subscriber<std_msgs::UInt16> sub("cmd_robot_control", cmdrobotCallback);

void setup()
{
  hNode.initNode();
  hNode.subscribe(sub);

  TCCR1B = TCCR1B & (0xf8 | 0x01); // Pin9,Pin10 PWM 31250Hz, Silent PWM
  TCCR2B = TCCR2B & (0xf8 | 0x01);
  Omni.PIDEnable(speed_kp, speed_ki, speed_kd, sampling_time);
}

void loop()
{
  hNode.spinOnce();
  delay(1);
}

#endif // CONTROL_3W_TEST

/***************************************************************************
 * CONTROL_TEST
 **************************************************************************/
#ifdef CONTROL_TEST

#define CMD_TURNLEFT    1
#define CMD_TURNRIGHT   2
#define CMD_STOP        3
#define SPEED           500

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle hNode;

void servo_cb( const std_msgs::UInt16& cmd_msg)
{
  // servo.write(cmd_msg.data); //set servo angle, should be from 0-180  
  // digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
  if (cmd_msg.data == CMD_TURNLEFT)
  {
    /* turn left */
    wheel1.setSpeedMMPS(SPEED, DIR_ADVANCE); // Set the pwm speed 100 direction
    wheel1.PIDRegulate();                  // regulate the PID
  } 
  else if (cmd_msg.data == CMD_TURNRIGHT)
  {
    /* turn right */
    wheel1.setSpeedMMPS(SPEED, DIR_BACKOFF); // Set the pwm speed 100 direction
    wheel1.PIDRegulate();                  // regulate the PID
  }
  else if (cmd_msg.data == CMD_STOP)
  {
    for (int i = SPEED; i >= 0; i--){
    wheel1.setSpeedMMPS(0, DIR_ADVANCE); 
    wheel1.PIDRegulate();                  // regulate the PID
    }
  }
  
}

ros::Subscriber<std_msgs::UInt16> sub("servo", servo_cb);

void setup(){
  pinMode(13, OUTPUT);

  hNode.initNode();
  hNode.subscribe(sub);

  TCCR1B = TCCR1B & (0xf8 | 0x01); // Pin9,Pin10 PWM 31250Hz, Silent PWM
  TCCR2B = TCCR2B & (0xf8 | 0x01);
  wheel1.PIDEnable(KC, TAUI, TAUD, 10); // used wheel1 to call the PIDEnable
}

void loop(){
  hNode.spinOnce();
  delay(1);
}

#endif // CONTROL_TEST

/***************************************************************************
 * ROSSER_SUB_TEST
 **************************************************************************/
#ifdef ROSSER_SUB_TEST
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

ros::NodeHandle hNode;

void messageCallback (const std_msgs::Empty& msgs){
  // digitalWrite(13, HIGH);
  wheel1.setSpeedMMPS(500, DIR_ADVANCE - digitalRead(M1_DIR)); // Set the pwm speed 100 direction
  wheel1.PIDRegulate();                  // regulate the PID
}

ros::Subscriber<std_msgs::Empty> ledBlink("led_blink", messageCallback);

char msgBuffer[64] = {};
int msgCounter = 0;

void setup()
{
  pinMode(13, OUTPUT);
  hNode.initNode();
  hNode.subscribe(ledBlink);
  TCCR1B = TCCR1B & (0xf8 | 0x01); // Pin9,Pin10 PWM 31250Hz, Silent PWM
  TCCR2B = TCCR2B & (0xf8 | 0x01);
  wheel1.PIDEnable(KC, TAUI, TAUD, 10); // used wheel1 to call the PIDEnable
}

void loop()
{
  hNode.spinOnce();
  delay(1);
}

#endif // ROSSER_SUB_TEST

/***************************************************************************
 * ROSSER_PUB_TEST
 **************************************************************************/
#ifdef ROSSER_PUB_TEST
#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle hNode;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char msgBuffer[64] = {};
int msgCounter = 0;

void setup()
{
  hNode.initNode();
  hNode.advertise(chatter);
}

void loop()
{
  sprintf(msgBuffer, "Hello from Arduino controller board to ROS, counter %d", msgCounter);
  str_msg.data = msgBuffer;
  chatter.publish(&str_msg);
  hNode.spinOnce();
  delay(100);
  msgCounter += 1;
  if (msgCounter == 100)
  {
    msgCounter = 0;
  }
  
}

#endif // ROSSER_PUB_TEST

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
