
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "wiring.h"
#endif

#include <PID_Beta6.h>
#include "fuzzy_table.h"
#include <wiring_private.h>
#include <HardwareSerial.h>

PID::PID(int *Input, int *Output, int *Setpoint, float Kc, float TauI, float TauD)
{
	PID::ConstructorCommon(Input, Output, Setpoint, Kc, TauI, TauD);
	UsingFeedForward = false;
	PID::Reset();
}

PID::PID(int *Input, int *Output, int *Setpoint, int *FFBias, float Kc, float TauI, float TauD)
{
	PID::ConstructorCommon(Input, Output, Setpoint, Kc, TauI, TauD);
	UsingFeedForward = true; // tell the controller that we'll be using an external
	myBias = FFBias;		 // bias, and where to find it
	PID::Reset();
}

void PID::ConstructorCommon(int *Input, int *Output, int *Setpoint, float Kc, float TauI, float TauD)
{
	PID::SetInputLimits(0, 1023); // default the limits to the
	PID::SetOutputLimits(0, 255); // full ranges of the I/O

	tSample = 1000; // default Controller Sample Time is 1 second

	PID::SetTunings(Kc, TauI, TauD);

	nextCompTime = millis();
	inAuto = false;
	myOutput = Output;
	myInput = Input;
	mySetpoint = Setpoint;

	Err = lastErr = prevErr = 0;
}

void PID::SetInputLimits(int INMin, int INMax)
{
	// after verifying that mins are smaller than maxes, set the values
	if (INMin >= INMax)
		return;

	inMin = INMin;
	inSpan = INMax - INMin;
}

void PID::SetOutputLimits(int OUTMin, int OUTMax)
{
	// after verifying that mins are smaller than maxes, set the values
	if (OUTMin >= OUTMax)
		return;

	outMin = OUTMin;
	outSpan = OUTMax - OUTMin;
}

void PID::SetTunings(float Kc, float TauI, float TauD)
{
	// verify that the tunings make sense
	if (Kc == 0.0 || TauI < 0.0 || TauD < 0.0)
		return;

	// we're going to do some funky things to the input numbers so all
	// our math works out, but we want to store the numbers intact
	// so we can return them to the user when asked.
	P_Param = Kc;
	I_Param = TauI;
	D_Param = TauD;

	// convert Reset Time into Reset Rate, and compensate for Calculation frequency
	float tSampleInSec = ((float)tSample / 1000.0);
	float tempTauR;
	if (TauI == 0.0)
		tempTauR = 0.0;
	else
		tempTauR = (1.0 / TauI) * tSampleInSec;

	kc = Kc;
	taur = tempTauR;
	taud = TauD / tSampleInSec;

	cof_A = kc * (1 + taur + taud);
	cof_B = kc * (1 + 2 * taud);
	cof_C = kc * taud;
}

void PID::Reset()
{

	if (UsingFeedForward)
		bias = (*myBias - outMin) / outSpan;
	else
		bias = (*myOutput - outMin) / outSpan;
}

void PID::SetMode(int Mode)
{
	if (Mode != 0 && !inAuto)
	{ // we were in manual, and we just got set to auto.
		// reset the controller internals
		PID::Reset();
	}
	inAuto = (Mode != 0);
}

void PID::SetSampleTime(int NewSampleTime)
{
	if (NewSampleTime > 0)
	{
		// convert the time-based tunings to reflect this change
		taur *= ((float)NewSampleTime) / ((float)tSample);
		taud *= ((float)NewSampleTime) / ((float)tSample);
		tSample = (unsigned long)NewSampleTime;

		cof_A = kc * (1 + taur + taud);
		cof_B = kc * (1 + 2 * taud);
		cof_C = kc * taud;
	}
}

void PID::Compute()
{
	justCalced = false;
	if (!inAuto)
		return; // if we're in manual just leave;

	unsigned long now = millis();

	// millis() wraps around to 0 at some point.  depending on the version of the
	// Arduino Program you are using, it could be in 9 hours or 50 days.
	// this is not currently addressed by this algorithm.

	//...Perform PID Computations if it's time...
	if (now >= nextCompTime)
	{

		Err = *mySetpoint - *myInput;
		// if we're using an external bias (i.e. the user used the
		// overloaded constructor,) then pull that in now
		if (UsingFeedForward)
		{
			bias = *myBias - outMin;
		}

		// perform the PID calculation.
		// float output = bias + kc * ((Err - lastErr)+ (taur * Err) + (taud * (Err - 2*lastErr + prevErr)));
		noInterrupts();
		int output = bias + (cof_A * Err - cof_B * lastErr + cof_C * prevErr);
		interrupts();

		// make sure the computed output is within output constraints
		if (output < -outSpan)
			output = -outSpan;
		else if (output > outSpan)
			output = outSpan;

		prevErr = lastErr;
		lastErr = Err;

		// scale the output from percent span back out to a real world number
		*myOutput = output;

		nextCompTime += tSample; // determine the next time the computation
		if (nextCompTime < now)
			nextCompTime = now + tSample; // should be performed

		justCalced = true; // set the flag that will tell the outside world that the output was just computed
	}
}

/*****************************************************************************
 * STATUS SECTION
 * These functions allow the outside world to query the status of the PID
 *****************************************************************************/

bool PID::JustCalculated()
{
	return justCalced;
}

int PID::GetMode()
{
	if (inAuto)
		return 1;
	else
		return 0;
}

int PID::GetINMin()
{
	return inMin;
}

int PID::GetINMax()
{
	return inMin + inSpan;
}

int PID::GetOUTMin()
{
	return outMin;
}

int PID::GetOUTMax()
{
	return outMin + outSpan;
}

int PID::GetSampleTime()
{
	return tSample;
}

float PID::GetP_Param()
{
	return P_Param;
}

float PID::GetI_Param()
{
	return I_Param;
}

float PID::GetD_Param()
{
	return D_Param;
}
