/*
 * pid.c
 *
 *  Created on: 4 Jun 2022
 *      Author: mobil
 */

#include "PIDVelocity.h"

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

float PIDAPositonController_Update(PIDAController *pid,TrajectoryG *traject, float setpoint, float measurement,float VfromTraject,float VMCal)
{
    float error = setpoint - measurement;
    float errorDZ = error;

	if(traject ->TrajectoryMode == 0) // S-curve
	{
	    if(AbsVal(VfromTraject) < AbsVal(VMCal) && traject->TrajectoryFlag == 0)
	    {
	    	pid->Kp  = 10.0;
	    	pid->Ki  = 0.0;
	    	pid->Kd  = 0.0;
	    }
	    else if (AbsVal(VfromTraject) == AbsVal(VMCal))
	    {
	    	pid->Kp  = 6.0;
	    	pid->Ki  = 0.0;
	    	pid->Kd  = 0.0;
	    	traject->TrajectoryFlag = 1;
	    }
	    else if (AbsVal(VfromTraject) == AbsVal(VMCal) && traject->TrajectoryFlag == 1)
	    {
	    	pid->Kp  = 6.0;
	    	pid->Ki  = 0.0;
	    	pid->Kd  = 0.0;
	    }
	}
	else if(traject ->TrajectoryMode == 1) //Quintic
	{
			pid->Kp  = 6.0;
			pid->Ki  = 0.0;
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



float PIDAVelocityController_Update(PIDAController *pid,TrajectoryG *traject, float setpoint, float measurement,float VfromTraject,float VMCal){

    float error = setpoint - measurement;
    float errorDZ = error;
	if(traject ->TrajectoryMode == 0) // S-curve
	{
	    if(AbsVal(VfromTraject) < AbsVal(VMCal) && traject->TrajectoryFlag == 0) // 10 deg/s
	    {
	    	pid->Kp  = 20.0;
	    	pid->Ki  = 1.6;
	    	pid->Kd  = 2.5;
	    }
	    else if (AbsVal(VfromTraject) == AbsVal(VMCal))
	    {
	    	pid->Kp  = 0.16;
	    	pid->Ki  = 3.14;
	    	pid->Kd  = 0;
	    	traject->TrajectoryFlag = 1;
	    }
	    else if (AbsVal(VfromTraject) == AbsVal(VMCal) && traject->TrajectoryFlag == 1)
	    {
	    	pid->Kp  = 0.27;
	    	pid->Ki  = 2.2;
	    	pid->Kd  = 0;
	    }
	}
	else if(traject ->TrajectoryMode == 1) //Quintic
	{
			pid->Kp  = 0.27;
			pid->Ki  = 2.2;
			pid->Kd  = 0;
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
