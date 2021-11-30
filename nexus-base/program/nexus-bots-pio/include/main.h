#ifndef MAIN_H
#define MAIN_H

#include <MotorWheel.h>

#ifndef MICRO_PER_SEC
#define MICRO_PER_SEC 1000000
#endif

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
// Motor 3
#define M3_PWM    10
#define M3_DIR    11
#define M3_ENCA   12
#define M3_ENCB   13

irqISR(irq1, isr1); // This will create a MotorWheel object called Wheel1
irqISR(irq2, isr2); // This will create a MotorWheel object called Wheel2
irqISR(irq3, isr3); // This will create a MotorWheel object called Wheel3
MotorWheel wheel1(M1_PWM, M1_DIR, M1_ENCA, M1_ENCB, &irq1);
MotorWheel wheel2(M2_PWM, M2_DIR, M2_ENCA, M2_ENCB, &irq2);
MotorWheel wheel3(M3_PWM, M3_DIR, M3_ENCA, M3_ENCB, &irq3);

#endif