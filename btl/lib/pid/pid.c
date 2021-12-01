/**********************************************************************************************
 * C PID Library - Version 1.0.1
 * modified my Matthew Blythe <mblythester@gmail.com> mjblythe.com/hacks
 * originally by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#include "pid.h"

void PID_Initialize(PidType* pid);

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up 
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
void PID_Init(	PidType *pid,
				float *Input,
				float *Output,
				float *Setpoint,
				float Kp,
				float Ki,
				float Kd,
				PidDirectionType controllerDirection)
{
  pid->myInput = Input;
  pid->myOutput = Output;
  pid->mySetpoint = Setpoint;
  pid->ITerm = 0;
  pid->lastInput = 0;
  pid->inAuto = true;

  PID_SetOutputLimits(pid, 0, 0xffff);

  //default Controller Sample Time is 0.1 seconds
  pid->SampleTime = 100;

  PID_SetControllerDirection(pid, controllerDirection);
  PID_SetTunings(pid, Kp, Ki, Kd);

//  pid->lastTime = millis() - pid->SampleTime;
}
 
 
/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/ 
bool PID_Compute(PidType* pid)
{
	if (!pid->inAuto)
	{
		return false;
	}

	float input = *pid->myInput;
	float error = *pid->mySetpoint - input;

	pid->ITerm += (pid->ki * error);
	if (pid->ITerm > pid->outMax)
	{
		pid->ITerm = pid->outMax;
	}
	else if (pid->ITerm < pid->outMin)
	{
		pid->ITerm = pid->outMin;
	}
	float dInput = (input - pid->lastInput);

	/*Compute PID Output*/
	float output = pid->kp * error + pid->ITerm - pid->kd * dInput;

	if (output > pid->outMax)
	{
		output = pid->outMax;
	}
	else if (output < pid->outMin)
	{
		output = pid->outMin;
	}
	*pid->myOutput = output;

	/*Remember some variables for next time*/
	pid->lastInput = input;
	//    pid->lastTime = now;

	return true;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted. 
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/ 

void PID_SetTunings(PidType* pid, float Kp, float Ki, float Kd) {
  if (Kp < 0 || Ki < 0 || Kd < 0)
  {
    return;
  }

  pid->dispKp = Kp;
  pid->dispKi = Ki;
  pid->dispKd = Kd;

  float SampleTimeInSec = ((float) pid->SampleTime) / 1000;
  pid->kp = Kp;
  pid->ki = Ki * SampleTimeInSec;
  pid->kd = Kd / SampleTimeInSec;

  if (pid->controllerDirection == PID_Direction_Reverse)
  {
    pid->kp = (0 - pid->kp);
    pid->ki = (0 - pid->ki);
    pid->kd = (0 - pid->kd);
  }
}
  
/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void PID_SetSampleTime(PidType* pid, float NewSampleTime) {
  if (NewSampleTime > 0) {
    float ratio = (float) NewSampleTime / (float) pid->SampleTime;
    pid->ki *= ratio;
    pid->kd /= ratio;
    pid->SampleTime = (unsigned long) NewSampleTime;
  }
}
 
/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID_SetOutputLimits(PidType* pid, float Min, float Max) {
  if (Min >= Max) {
    return;
  }
  pid->outMin = Min;
  pid->outMax = Max;

  if (pid->inAuto)
  {
    if(*pid->myOutput > pid->outMax)
    {
      *pid->myOutput = pid->outMax;
    }
    else if(*pid->myOutput < pid->outMin)
    {
      *pid->myOutput = pid->outMin;
    }

    if (pid->ITerm > pid->outMax)
    {
      pid->ITerm = pid->outMax;
    }
    else if(pid->ITerm < pid->outMin)
    {
      pid->ITerm = pid->outMin;
    }
  }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/ 
void PID_SetMode(PidType* pid, PidModeType Mode)
{
    bool newAuto = (Mode == PID_Mode_Automatic);
    if(newAuto == !pid->inAuto)
    {
    	/*we just went from manual to auto*/
        PID_Initialize(pid);
    }
    pid->inAuto = newAuto;
}
 
/* Initialize()****************************************************************
 *  does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/ 
void PID_Initialize(PidType* pid)
{
	pid->ITerm = *pid->myOutput;
	pid->lastInput = *pid->myInput;

	if(pid->ITerm > pid->outMax)
	{
		pid->ITerm = pid->outMax;
	}
	else if(pid->ITerm < pid->outMin)
	{
		pid->ITerm = pid->outMin;
	}
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads 
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID_SetControllerDirection(PidType* pid, PidDirectionType Direction) {
  if (pid->inAuto && Direction != pid->controllerDirection) {
    pid->kp = (0 - pid->kp);
    pid->ki = (0 - pid->ki);
    pid->kd = (0 - pid->kd);
  }
  pid->controllerDirection = Direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display 
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
float PID_GetKp(PidType* pid)
{
  return pid->dispKp;
}
float PID_GetKi(PidType* pid)
{
  return pid->dispKi;
}
float PID_GetKd(PidType* pid)
{
  return pid->dispKd;
}
PidModeType PID_GetMode(PidType* pid)
{
  return pid->inAuto ? PID_Mode_Automatic : PID_Mode_Manual;
}
PidDirectionType PID_GetDirection(PidType* pid)
{
  return pid->controllerDirection;
}

