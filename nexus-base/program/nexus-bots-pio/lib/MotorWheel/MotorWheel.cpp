#include <MotorWheel.h>

Motor::Motor(unsigned char _pinPWM, unsigned char _pinDir,
			 unsigned char _pinIRQ, unsigned char _pinIRQB,
			 struct ISRVars *_isr)
	: PID(&speedRPMInput, &speedRPMOutput, &speedRPMDesired, KC, TAUI, TAUD),
	  pinPWM(_pinPWM), pinDir(_pinDir), isr(_isr)
{
	debug();

	isr->pinIRQ = _pinIRQ;
	isr->pinIRQB = _pinIRQB;

/*for maple*/
#if defined(BOARD_maple) || defined(BOARD_maple_native) || defined(BOARD_maple_mini)
	pinMode(pinPWM, PWM);
	pinMode(pinDir, OUTPUT);
	pinMode(isr->pinIRQ, INPUT_FLOATING);

/*for arduino*/
#else
	pinMode(pinPWM, OUTPUT);
	pinMode(pinDir, OUTPUT);
	pinMode(isr->pinIRQ, INPUT);

#endif
	if (isr->pinIRQB != PIN_UNDEFINED)
	{
		pinMode(isr->pinIRQB, INPUT);
	}

	PIDDisable();
}

void Motor::setupInterrupt()
{
/*for maple*/
#if defined(BOARD_maple) || defined(BOARD_maple_native) || defined(BOARD_maple_mini)
	attachInterrupt(isr->pinIRQ, isr->ISRfunc, TRIGGER); // RISING --> CHANGE 201207

/*for arduino*/
#else
	if (isr->pinIRQ == 2 || isr->pinIRQ == 3)
		attachInterrupt(isr->pinIRQ - 2, isr->ISRfunc, TRIGGER);
	else
	{
		PCattachInterrupt(isr->pinIRQ, isr->ISRfunc, TRIGGER); // RISING --> CHANGE 201207
	}

#endif
}

unsigned char Motor::getPinPWM() const
{
	debug();
	return pinPWM;
}

unsigned char Motor::getPinDir() const
{
	debug();
	return pinDir;
}

unsigned char Motor::getPinIRQ() const
{
	debug();
	return isr->pinIRQ;
}

unsigned char Motor::getPinIRQB() const
{
	debug();
	return isr->pinIRQB;
}

unsigned int Motor::runPWM(unsigned int PWM, bool dir, bool saveDir)
{
	debug();
	speedPWM = PWM;
	if (saveDir)
		desiredDirection = dir;
	analogWrite(pinPWM, PWM);
	digitalWrite(pinDir, dir);
	return PWM;
}

unsigned int Motor::advancePWM(unsigned int PWM)
{
	debug();
	return runPWM(PWM, DIR_ADVANCE);
}

unsigned int Motor::backoffPWM(unsigned int PWM)
{
	debug();
	return runPWM(PWM, DIR_BACKOFF);
}

unsigned int Motor::getPWM() const
{
	debug();
	return speedPWM;
}

bool Motor::setDesiredDir(bool dir)
{
	debug();
	// runPWM(getPWM(),dir);	// error
	desiredDirection = dir;
	return getDesiredDir();
}

bool Motor::getDesiredDir() const
{
	debug();
	return desiredDirection;
}

bool Motor::reverseDesiredDir()
{
	debug();
	runPWM(getPWM(), !getDesiredDir());
	return getDesiredDir();
}

bool Motor::getCurrDir() const
{
	return isr->currDirection;
}

bool Motor::setCurrDir()
{
	if (getPinIRQB() != PIN_UNDEFINED)
		return isr->currDirection = digitalRead(isr->pinIRQB);
	return false;
}

unsigned int Motor::setSpeedRPM(int speedRPM, bool dir)
{
	debug();
	PIDSetSpeedRPMDesired(speedRPM);
	setDesiredDir(dir);
	return abs(getSpeedRPM());
}

// direction sensitive 201208
int Motor::getSpeedRPM() const
{
	debug();
	if (getCurrDir() == DIR_ADVANCE)
		return SPEEDPPS2SPEEDRPM(isr->speedPPS);
	return -SPEEDPPS2SPEEDRPM(isr->speedPPS);
}

int Motor::setSpeedRPM(int speedRPM)
{
	debug();
	if (speedRPM >= 0)
		return setSpeedRPM(speedRPM, DIR_ADVANCE);
	else
		return setSpeedRPM(abs(speedRPM), DIR_BACKOFF);
}

bool Motor::PIDGetStatus() const
{
	debug();
	return pidCtrl;
}

bool Motor::PIDEnable(float kc, float taui, float taud, unsigned int sampleTime)
{
	debug();
	setupInterrupt();
	PIDSetup(kc, taui, taud, sampleTime);
	return pidCtrl = true;
}

bool Motor::PIDDisable()
{
	debug();

	return pidCtrl = false;
}

bool Motor::PIDReset()
{
	debug();
	if (PIDGetStatus() == false)
		return false;
	PID::Reset();
	return true;
}

bool Motor::PIDSetup(float kc, float taui, float taud, unsigned int sampleTime)
{
	debug();
	PID::SetTunings(kc, taui, taud);
	PID::SetInputLimits(0, MAX_SPEEDRPM);
	PID::SetOutputLimits(0, MAX_SPEEDRPM);
	PID::SetSampleTime(sampleTime);
	PID::SetMode(AUTO);
	return true;
}

unsigned int Motor::PIDSetSpeedRPMDesired(unsigned int speedRPM)
{
	debug();
	if (speedRPM > MAX_SPEEDRPM)
		speedRPMDesired = MAX_SPEEDRPM;
	else
		speedRPMDesired = speedRPM;
	return PIDGetSpeedRPMDesired();
}

unsigned int Motor::PIDGetSpeedRPMDesired() const
{
	debug();
	return speedRPMDesired;
}

bool Motor::PIDRegulate(bool doRegulate)
{
	debug();
	if (PIDGetStatus() == false)
		return false;
	if (getPinIRQB() != PIN_UNDEFINED && getDesiredDir() != getCurrDir())
	{
		speedRPMInput = -SPEEDPPS2SPEEDRPM(isr->speedPPS);
	}
	else
	{
		speedRPMInput = SPEEDPPS2SPEEDRPM(isr->speedPPS);
	}

	PID::Compute();
	if (doRegulate && PID::JustCalculated())
	{
		speed2DutyCycle += speedRPMOutput;

		if (speed2DutyCycle >= MAX_SPEEDRPM)
			speed2DutyCycle = MAX_SPEEDRPM;
		else if (speed2DutyCycle <= -MAX_SPEEDRPM)
			speed2DutyCycle = -MAX_SPEEDRPM;
		if (speed2DutyCycle >= 0)
		{
			runPWM(map(speed2DutyCycle, 0, MAX_SPEEDRPM, 0, MAX_PWM), getDesiredDir(), false);
		}
		else
		{
			runPWM(map(abs(speed2DutyCycle), 0, MAX_SPEEDRPM, 0, MAX_PWM), !getDesiredDir(), false);
		}
		return true;
	}
	return false;
}

int Motor::getSpeedPPS() const
{
	return isr->speedPPS;
}

long Motor::getCurrPulse() const
{
	return isr->pulses;
}

long Motor::setCurrPulse(long _pulse)
{
	isr->pulses = _pulse;
	return getCurrPulse();
}

long Motor::resetCurrPulse()
{
	return setCurrPulse(0);
}

void Motor::delayMS(unsigned int ms, bool debug)
{
	for (unsigned long endTime = millis() + ms; millis() < endTime;)
	{
		PIDRegulate();
		if (debug && (millis() % 500 == 0))
			debugger();
		if (endTime - millis() >= SAMPLETIME)
			delay(SAMPLETIME);
		else
			delay(endTime - millis());
	}
}

// to reduce the binary for Atmega168 (16KB), Serial output is disabled
void Motor::debugger() const
{
	debug()

#ifdef DEBUG
		if (!Serial.available()) Serial.begin(Baudrate);
	/*
		Serial.print("pinPWM -> ");
		Serial.println(pinPWM,DEC);
		Serial.print("pinDir -> ");
		Serial.println(pinDir,DEC);
		Serial.print("pinIRQ -> ");
		Serial.println(pinIRQ,DEC);
		Serial.print("pinIRQB-> ");
		Serial.println(pinIRQB,DEC);
	 */

	Serial.print("DesiredDir -> ");
	Serial.println(desiredDirection);
	Serial.print("currDir ->");
	Serial.println(isr->currDirection);

	Serial.print("PWM    -> ");
	Serial.println(speedPWM, DEC);
	Serial.print("Input  -> ");
	Serial.println(speedRPMInput, DEC);
	Serial.print("Output -> ");
	Serial.println(speedRPMOutput, DEC);
	Serial.print("Desired-> ");
	Serial.println(speedRPMDesired, DEC);

	/*
		Serial.print("speed2DutyCycle-> ");
		Serial.println(speed2DutyCycle);
		Serial.print("speedPPS> ");
		Serial.println(isr->speedPPS,DEC);
		Serial.print("pulses -> ");
		Serial.println(isr->pulses,DEC);
	 */

#endif
}

GearedMotor::GearedMotor(unsigned char _pinPWM, unsigned char _pinDir,
						 unsigned char _pinIRQ, unsigned char _pinIRQB,
						 struct ISRVars *_isr, unsigned int ratio) : Motor(_pinPWM, _pinDir, _pinIRQ, _pinIRQB, _isr), _ratio(ratio)
{
	debug();
}

unsigned int GearedMotor::getRatio() const
{
	return _ratio;
}

unsigned int GearedMotor::setRatio(unsigned int ratio)
{
	_ratio = ratio;
	return getRatio();
}

/*
float GearedMotor::getGearedAccRPMM() const {
	debug();
	return (float)Motor::getAccRPMM()/getRatio();
}
 */
float GearedMotor::getGearedSpeedRPM() const
{
	debug();
	// return (float)Motor::getSpeedRPM()/REDUCTION_RATIO;
	// return (float)Motor::getSpeedRPM()/_ratio;
	return (float)Motor::getSpeedRPM() / getRatio();
}

float GearedMotor::setGearedSpeedRPM(float gearedSpeedRPM, bool dir)
{
	debug();
	// Motor::setSpeedRPM(abs(gearedSpeedRPM*REDUCTION_RATIO),dir);
	Motor::setSpeedRPM(abs(round(gearedSpeedRPM * _ratio)), dir);
	return getGearedSpeedRPM();
}

// direction sensitive, 201208
float GearedMotor::setGearedSpeedRPM(float gearedSpeedRPM)
{
	debug();
	// Motor::setSpeedRPM(gearedSpeedRPM*_ratio);
	Motor::setSpeedRPM(round(gearedSpeedRPM * _ratio));
	return getGearedSpeedRPM();
}

MotorWheel::MotorWheel(unsigned char _pinPWM, unsigned char _pinDir,
					   unsigned char _pinIRQ, unsigned char _pinIRQB,
					   struct ISRVars *_isr,
					   unsigned int ratio, unsigned int cirMM) : GearedMotor(_pinPWM, _pinDir, _pinIRQ, _pinIRQB, _isr, ratio), _cirMM(cirMM)
{
	debug();
}
unsigned int MotorWheel::getCirMM() const
{
	return _cirMM;
}
unsigned int MotorWheel::setCirMM(unsigned int cirMM)
{
	if (cirMM > 0)
		_cirMM = cirMM;
	return getCirMM();
}
/*
int MotorWheel::getAccCMPMM() const {
	debug();
	return int(GearedMotor::getGearedAccRPMM()*getCirMM()/10);
}
 */
/*
unsigned int MotorWheel::getSpeedCMPM() const {
	debug();
	//return int(GearedMotor::getGearedSpeedRPM()*CIR);
	return int(GearedMotor::getGearedSpeedRPM()*getCirMM()/10);
}
 */
// direction sensitive, 201208
int MotorWheel::getSpeedCMPM() const
{
	debug();
	// return int(GearedMotor::getGearedSpeedRPM()*CIR);
	return int(GearedMotor::getGearedSpeedRPM() * getCirMM() / 10);
}

int MotorWheel::setSpeedCMPM(unsigned int cm, bool dir)
{
	debug();
	// GearedMotor::setGearedSpeedRPM(cm/CIR,dir);
	GearedMotor::setGearedSpeedRPM(cm * 10.0 / getCirMM(), dir);
	return getSpeedCMPM();
}

// direction sensitive, 201208
int MotorWheel::setSpeedCMPM(int cm)
{
	debug();
	// GearedMotor::setGearedSpeedRPM(cm/CIR,dir);
	GearedMotor::setGearedSpeedRPM(cm * 10.0 / getCirMM());
	return getSpeedCMPM();
}

/*
int MotorWheel::getAccMMPSS() const {
	debug();
	return int(getAccCMPMM()/6);
}
 */
// direction sensitive, 201208
int MotorWheel::getSpeedMMPS() const
{
	debug();
	return int(getSpeedCMPM() / 6); //(mm/sec)/(cm/min) = 6
}

int MotorWheel::setSpeedMMPS(unsigned int mm, bool dir)
{
	debug();
	setSpeedCMPM(mm * 6, dir);
	return getSpeedMMPS();
}

// direction sensitive, 201208
int MotorWheel::setSpeedMMPS(int mm)
{
	debug();
	setSpeedCMPM(mm * 6);
	return getSpeedMMPS();
}
