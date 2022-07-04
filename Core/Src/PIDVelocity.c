/*
 * pid.c
 *
 *  Created on: 4 Jun 2022
 *      Author: mobil
 */

#include "PIDVelocity.h"
#include "Trajectory.h"

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

float PIDAPositonController_Update(PIDAController *pid, float setpoint, float measurement)
{
    float error = setpoint - measurement;
    float errorDZ = error;

//    if(AbsVal(setpoint) < 20.0) // 10 deg/s
//    {
//    	pid->Kp  = 14.0;
//    	pid->Ki  = 0.2;
//    	pid->Kd  = 5.0;
//    }
//    else
//    {
//    	pid->Kp  = 14.0;
//    	pid->Ki  = 0.000;
//    	pid->Kd  = 0.0;
//    }
	pid->Kp  = 14.0;
	pid->Ki  = 0.000;
	pid->Kd  = 0.0;

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



float PIDAVelocityController_Update(PIDAController *pid, float setpoint, float measurement){

    float error = setpoint - measurement;
    float errorDZ = error;

//    if(AbsVal(setpoint) < 51.0) // 10 deg/s
//    {
//    	pid->Kp  = 20.0;
//    	pid->Ki  = 1.6;
//    	pid->Kd  = 2.5;
//    }
//    else
//    {
//    	pid->Kp  = 0.2;
//    	pid->Ki  = 0.1;
//    	pid->Kd  = 0;
//    }
	pid->Kp  = 20.0;
	pid->Ki  = 1.6;
	pid->Kd  = 2.5;

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
