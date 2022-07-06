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

float PIDAPositonController_Update(PIDAController *pid, float setpoint, float measurement, float Distance)
{
    float error = setpoint - measurement;
    float errorDZ = error;

    // Quintic Trajectory
//    if(AbsVal(setpoint) < AbsVal(0.05*Distance)) // 10 deg/s
//    {
//    	pid->Kp  = 1.0;
//    	pid->Ki  = 0.0;
//    	pid->Kd  = 0.0;
//    }
//    else
//    {
//    	pid->Kp  = 3.0;
//    	pid->Ki  = 0.0;
//    	pid->Kd  = 0.0;
//    }
//	pid->Kp  = 6.0;
//	pid->Ki  = 0.0;
//	pid->Kd  = 0.0;

    // Scurve Trajectory
    if(AbsVal(setpoint) <= AbsVal(0.1*Distance)) // 10 deg/s
    {
//    	pid->Kp  = 0.2;
//    	pid->Ki  = 6.0;
//    	pid->Kd  = 0.0;
    	pid->Kp  = 6.0;
    	pid->Ki  = 0.05;
    	pid->Kd  = 0.0;
    }
    else
    {
    	pid->Kp  = 6.0;
    	pid->Ki  = 0.05;
    	pid->Kd  = 0.0;
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



float PIDAVelocityController_Update(PIDAController *pid, float setpoint, float measurement,float VMCal){

    float error = setpoint - measurement;
    float errorDZ = error;

//    if(AbsVal(setpoint) < AbsVal(0.07*VMCal)) // 10 deg/s
//    {
//		pid->Kp  = 20.0;
//		pid->Ki  = 4.2;
//		pid->Kd  = 2.25;
//    }
//    else
//    {
//		pid->Kp  = 0.27;
//		pid->Ki  = 2.2;
//		pid->Kd  = 0.0095;
//    }
//	pid->Kp  = 0.27;
//	pid->Ki  = 2.2;
//	pid->Kd  = 0.0095;

    // Scurve Trajectory
//    if(AbsVal(setpoint) < VMCal) // 10 deg/s
//    {
//    	pid->Kp  = 0.27;
//		pid->Ki  = 2.2;
//		pid->Kd  = 0.0095;
//    }
//    else
//    {
//    	pid->Kp  = 0.2;
//    	pid->Ki  = 0.1;
//    	pid->Kd  = 0;
//    }
	pid->Kp  = 0.160041136848727;
	pid->Ki  = 3.13946329365331;
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
