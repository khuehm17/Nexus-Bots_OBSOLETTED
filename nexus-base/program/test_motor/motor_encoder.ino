#include <PinChangeIntConfig.h>
#include <PID_Beta6.h>
#include <MotorWheel.h>
#ifndef MICRO_PER_SEC
#define MICRO_PER_SEC 1000000
#endif

#define M1_PWM    6
#define M1_DIR    7
#define M1_ENCA   8
#define M1_ENCB   9

irqISR(irq1, isr1);         //This will create a MotorWheel object called Wheel1
MotorWheel wheel1(M1_PWM,M1_DIR,M1_ENCA,M1_ENCB,&irq1); //Motor PWM:Pin9, DIR: Pin8, Encoder A:Pin6, B:Pin7

void setup() {
  TCCR1B=TCCR1B&0xf8|0x01;                //Pin9,Pin10 PWM 31250Hz, Silent PWM
  wheel1.PIDEnable(KC,TAUI,TAUD,10);     //used wheel1 to call the PIDEnable
  Serial.begin(19200);
}

void loop() {
  wheel1.setSpeedMMPS(200, DIR_ADVANCE);  //Set the pwm speed 100 direction
  wheel1.PIDRegulate();     //regulate the PID
  if(millis()%500==0){
//    Serial.print("speedRPM>");
//    Serial.println(wheel1.getSpeedRPM(),DEC);//display the speed of the MotorWheel
//    Serial.print("MMPS -->");
    Serial.println(wheel1.getSpeedMMPS(),DEC);//display the speed of the motor
    Serial.println("");
    //wheel1.debugger();
  }
}
