
#ifndef PID_H
#define PID_H

//typedef double float;
#include <stdbool.h>

//Constants used in some of the functions below
typedef enum
{
  PID_Mode_Automatic = 1,
  PID_Mode_Manual    = 0
} PidModeType;

typedef enum
{
  PID_Direction_Direct  = 0,
  PID_Direction_Reverse = 1
} PidDirectionType;

typedef struct
{
	float dispKp; // * we'll hold on to the tuning parameters in user-entered
	float dispKi; //   format for display purposes
	float dispKd; //

	float kp; // * (P)roportional Tuning Parameter
	float ki; // * (I)ntegral Tuning Parameter
	float kd; // * (D)erivative Tuning Parameter

	PidDirectionType controllerDirection;

	/* * Pointers to the Input, Output, and Setpoint variables
	*   This creates a hard link between the variables and the
	PID, freeing the user from having to constantly tell us
	what these values are.  with pointers we'll just know. */
	float *myInput;
	float *myOutput;
	float *mySetpoint;

	//  unsigned long lastTime;
	volatile float ITerm, lastInput;

	unsigned long SampleTime;
	float outMin, outMax;
	bool inAuto;
} PidType;

//	Commonly used functions **************************************************************************

//  Constructor.  links the PID to the Input, Output, and
//  Setpoint.  Initial tuning parameters are also set here
void PID_Init(	PidType *pid,
				float *Input,
				float *Output,
				float *Setpoint,
				float Kp,
				float Ki,
				float Kd,
				PidDirectionType controllerDirection);

// sets PID to either Manual (0) or Auto (non-0)
void PID_SetMode(PidType *pid, PidModeType mode);

// performs the PID calculation.  it should be
// called every time loop() cycles. ON/OFF and
// calculation frequency can be set using SetMode
// SetSampleTime respectively
bool PID_Compute(PidType *pid);

// clamps the output to a specific range. 0-255 by default, but
// it's likely the user will want to change this depending on
// the application
void PID_SetOutputLimits(PidType *pid, float min, float max);

//available but not commonly used functions ********************************************************

// While most users will set the tunings once in the
// constructor, this function gives the user the option
// of changing tunings during runtime for Adaptive control
void PID_SetTunings(PidType *pid, float kp, float ki, float kd);

// Sets the Direction, or "Action" of the controller. DIRECT
// means the output will increase when error is positive. REVERSE
// means the opposite.  it's very unlikely that this will be needed
// once it is set in the constructor.
void PID_SetControllerDirection(PidType *pid, PidDirectionType Direction);

// sets the frequency, in Milliseconds, with which
// the PID calculation is performed.  default is 100
void PID_SetSampleTime(PidType *pid, float newSampleTime);

//Display functions ****************************************************************
// These functions query the pid for interal values.
// they were created mainly for the pid front-end,
// where it's important to know what is actually
// inside the PID.
float PID_GetKp(PidType *pid);
float PID_GetKi(PidType *pid);
float PID_GetKd(PidType *pid);
PidModeType PID_GetMode(PidType *pid);
PidDirectionType PID_GetDirection(PidType *pid);

//void PID_Initialize(PidType *pid);

#endif //PID_H

