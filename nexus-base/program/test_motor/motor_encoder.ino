#include <PinChangeIntConfig.h>
#include <PID_Beta6.h>
#include <MotorWheel.h>
#ifndef MICRO_PER_SEC
#define MICRO_PER_SEC 1000000
#endif

// Motor 3
#define M3_PWM    10
#define M3_DIR    11
#define M3_ENCA   12
#define M3_ENCB   13
// Motor 1
#define M1_PWM    5
#define M1_DIR    4
#define M1_ENCA   2
#define M1_ENCB   3
// Motor 2
#define M2_PWM    6
#define M2_DIR    7
#define M2_ENCA   8
#define M2_ENCB   9

irqISR(irq1, isr1);         //This will create a MotorWheel object called Wheel1
irqISR(irq2, isr2);         //This will create a MotorWheel object called Wheel2
irqISR(irq3, isr3);         //This will create a MotorWheel object called Wheel3
MotorWheel wheel1(M1_PWM,M1_DIR,M1_ENCA,M1_ENCB,&irq1); 
MotorWheel wheel2(M2_PWM,M2_DIR,M2_ENCA,M2_ENCB,&irq2); 
MotorWheel wheel3(M3_PWM,M3_DIR,M3_ENCA,M3_ENCB,&irq3); 

void setup() {
  TCCR1B=TCCR1B&0xf8|0x01;                //Pin9,Pin10 PWM 31250Hz, Silent PWM
  TCCR2B=TCCR2B&0xf8|0x01;
  wheel1.PIDEnable(KC,TAUI,TAUD,10);     //used wheel1 to call the PIDEnable
  wheel2.PIDEnable(KC,TAUI,TAUD,10);     //used wheel2 to call the PIDEnable
  wheel3.PIDEnable(KC,TAUI,TAUD,10);     //used wheel3 to call the PIDEnable
  Serial.begin(19200);
}

void loop() {
  wheel1.setSpeedMMPS(500, DIR_ADVANCE);  //Set the pwm speed 100 direction
  wheel1.PIDRegulate();     //regulate the PID
  wheel2.setSpeedMMPS(500, DIR_ADVANCE);  //Set the pwm speed 100 direction
  wheel2.PIDRegulate();     //regulate the PID
  wheel3.setSpeedMMPS(300, DIR_ADVANCE);  //Set the pwm speed 100 direction
  wheel3.PIDRegulate();     //regulate the PID
  if(millis()%500==0){
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

    Serial.print(wheel1.getSpeedRPM(),DEC);//display the speed of the MotorWheel
    Serial.print(",");
    Serial.print(wheel1.getSpeedMMPS(),DEC);//display the speed of the motor
    Serial.print(",");
    Serial.print(wheel2.getSpeedRPM(),DEC);//display the speed of the MotorWheel
    Serial.print(",");
    Serial.print(wheel2.getSpeedMMPS(),DEC);//display the speed of the motor
    Serial.print("\n");
    //wheel1.debugger();
  }
}