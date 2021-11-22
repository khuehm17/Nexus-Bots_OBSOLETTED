#define E2    6 //the pin to control mator’s speed
#define M2    7 //the pin to control direction
#define E1    5 //the pin to control mator’s speed
#define M1    4 //the pin to control direction
#define E3    10 //the pin to control mator’s speed
#define M3    11 //the pin to control direction
void setup()
{
pinMode(M1,OUTPUT); //M1 direction control
pinMode(E1,OUTPUT); //E1 PWM speed control
analogWrite(E1,100);
pinMode(M2,OUTPUT); //M2 direction control
pinMode(E2,OUTPUT); //E2 PWM speed control
analogWrite(E2,100);
pinMode(M3,OUTPUT); //M1 direction control
pinMode(E3,OUTPUT); //E1 PWM speed control
analogWrite(E3,100);
TCCR2B = TCCR2B & 0b11111000 | 0x01;
//set the timer1 as the work intrrupt timer
// to use the timer will defualt at the function of setup;
}
void loop() { 
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);
  digitalWrite(M3, HIGH);
  delay(2000);
  digitalWrite(M1, LOW);
  delay(2000);
  
  
  }
