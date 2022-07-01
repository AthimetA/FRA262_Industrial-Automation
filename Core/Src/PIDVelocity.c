/*
 * pid.c
 *
 *  Created on: 4 Jun 2022
 *      Author: mobil
 */

#include "PIDVelocity.h"
#include "Trajectory.h"

#define PID_KP  3.43589013835019f
#define PID_KI  0.936768738675359f
#define PID_KD  0.00054490119125281f
#define PIDVELO_KP  0.00509871617350813f
#define PIDVELO_KI  0.509871617350813f
#define PIDVELO_KD  0.0000127467904337703f
#define PID_LIM_MIN_INT -10000.0f
#define PID_LIM_MAX_INT  10000.0f

void PIDAController_Init(PIDAController *pid)
{
	pid->Last1Error = 0.0f;
	pid->Last2Error = 0.0f;

	pid->differentiatorOutput = 0.0f;
	pid->integratorOutput = 0.0f;
	pid->proportionalOutput = 0.0f;

	pid->ControllerOut = 0.0f;
	pid->ControllerLastOut = 0.0f;

	pid->OutputMax = PID_LIM_MAX_INT;
	pid->OutputMin = PID_LIM_MIN_INT;
}

float PIDAVelocityController_Update(PIDAController *pid, float setpoint, float measurement){

    float error = setpoint - measurement;
    float errorDZ = error;
    float deadzone = 2.5;

//    if(error > deadzone)
//    {
//    	errorDZ = error - deadzone;
//    }
//    else if( (-1.0*deadzone) < error && error < deadzone)
//    {
//    	errorDZ = 0;
//    }
//    else
//    {
//    	errorDZ = error + deadzone;
//    }
//    float errorLow = setpoint*0.1;
//    float errorHigh = setpoint*0.9;

//    if(AbsVal(error) < errorLow || AbsVal(error) > errorHigh)
//    {
//    	pid->KpUse = pid->Kp*10.0;
//    }
//    else
//    {
//    	pid->KpUse = pid->Kp;
//    }

    if(AbsVal(setpoint) <= 10.0) // 10 deg/s
    {
//    	pid->Kp  = 0.2;
//    	pid->Ki  = 6.0;
//    	pid->Kd  = 0.0;
    	pid->Kp  = PIDVELO_KP;
    	pid->Ki  = PIDVELO_KI;
    	pid->Kd  = PIDVELO_KD;
    }
    else
    {
    	pid->Kp  = PIDVELO_KP;
    	pid->Ki  = PIDVELO_KI;
    	pid->Kd  = PIDVELO_KD;
    }

	// Compute error of each term

    pid->proportionalOutput = (pid->Kp*errorDZ) - (pid->Kp * pid->Last1Error);

    pid->integratorOutput = (pid->Ki * errorDZ);

    pid->differentiatorOutput = pid->Kd *(errorDZ -(2.0* pid->Last1Error) + pid->Last2Error)	;

	// Compute output and apply limits

    pid->ControllerOut = pid->proportionalOutput + pid->integratorOutput + pid->differentiatorOutput
    								+ pid->ControllerLastOut;

    if (pid->ControllerOut > pid->OutputMax) {

    	pid->ControllerOut = pid->OutputMax;

    } else if (pid->ControllerOut < pid->OutputMin) {

    	pid->ControllerOut = pid->OutputMin;
    }

    // Controller Memory

    pid->ControllerLastOut = pid->ControllerOut;
	pid->Last2Error = pid->Last1Error;
	pid->Last1Error = errorDZ;

	return pid->ControllerOut;
}

float PIDAPositonController_Update(PIDAController *pid, float setpoint, float measurement)
{
    float error = setpoint - measurement;
    float errorDZ = error;
    float deadzone = 0.4;

//    if(error > deadzone)
//    {
//    	errorDZ = error - deadzone;
//    }
//    else if( (-1.0*deadzone) < error && error < deadzone)
//    {
//    	errorDZ = 0;
//    }
//    else
//    {
//    	errorDZ = error + deadzone;
//    }

    if(AbsVal(setpoint) < 10.0) // 10 deg/s
    {
    	pid->Kp  = PID_KP;
    	pid->Ki  = PID_KI;
    	pid->Kd  = PID_KD;
    }
    else
    {
    	pid->Kp  = PID_KP;
    	pid->Ki  = PID_KI;
    	pid->Kd  = PID_KD;
    }

	// Compute error of each term

    pid->proportionalOutput = (pid->Kp*errorDZ) - (pid->Kp * pid->Last1Error);

    pid->integratorOutput = (pid->Ki * errorDZ);

    pid->differentiatorOutput = pid->Kd *(errorDZ -(2.0* pid->Last1Error) + pid->Last2Error)	;

	// Compute output and apply limits

    pid->ControllerOut = pid->proportionalOutput + pid->integratorOutput + pid->differentiatorOutput
    								+ pid->ControllerLastOut;

    if (pid->ControllerOut > pid->OutputMax) {

    	pid->ControllerOut = pid->OutputMax;

    } else if (pid->ControllerOut < pid->OutputMin) {

    	pid->ControllerOut = pid->OutputMin;
    }

    // Controller Memory

    pid->ControllerLastOut = pid->ControllerOut;
	pid->Last2Error = pid->Last1Error;
	pid->Last1Error = errorDZ;

	return pid->ControllerOut;
}
