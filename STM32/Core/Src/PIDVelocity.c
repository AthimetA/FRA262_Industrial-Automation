/*
 * PIDVelocity.c
 *
 *  Created on: 4 Jun 2022
 *      Author: mobil
 */

#include "PIDVelocity.h"
#include "Trajectory.h"

void  PIDVelocityController_Init(PIDVelocityController *pidVelocity){

	pidVelocity->Last1Error = 0.0f;
	pidVelocity->Last2Error = 0.0f;

	pidVelocity->differentiatorOutput = 0.0f;
	pidVelocity->integratorOutput = 0.0f;
	pidVelocity->proportionalOutput = 0.0f;

	pidVelocity->ControllerOut = 0.0f;
	pidVelocity->ControllerLastOut = 0.0f;
}

float PIDVelocityController_Update(PIDVelocityController *pidVelocity, float setpoint, float measurement){

    float error = setpoint - measurement;
    float errorLow = setpoint*0.1;
    float errorHigh = setpoint*0.9;

//    if(AbsVal(error) < errorLow || AbsVal(error) > errorHigh)
//    {
//    	pidVelocity->KpUse = pidVelocity->Kp*10.0;
//    }
//    else
//    {
//    	pidVelocity->KpUse = pidVelocity->Kp;
//    }

	// Compute error of each term

    pidVelocity->proportionalOutput = (pidVelocity->Kp*error) - (pidVelocity->Kp * pidVelocity->Last1Error);

    pidVelocity->integratorOutput = (pidVelocity->Ki * error);

    pidVelocity->differentiatorOutput = ((pidVelocity->Kd*error)) - ((2.0 * pidVelocity->Kd * pidVelocity->Last1Error))
    									+((pidVelocity->Kd * pidVelocity->Last2Error))	;

	// Compute output and apply limits

    pidVelocity->ControllerOut = pidVelocity->proportionalOutput + pidVelocity->integratorOutput + pidVelocity->differentiatorOutput
    								+ pidVelocity->ControllerLastOut;

    if (pidVelocity->ControllerOut > pidVelocity->OutputMax) {

    	pidVelocity->ControllerOut = pidVelocity->OutputMax;

    } else if (pidVelocity->ControllerOut < pidVelocity->OutputMin) {

    	pidVelocity->ControllerOut = pidVelocity->OutputMin;
    }

    // Controller Memory

    pidVelocity->ControllerLastOut = pidVelocity->ControllerOut;
	pidVelocity->Last1Error = error;
	pidVelocity->Last2Error = pidVelocity->Last1Error;

	return pidVelocity->ControllerOut;
}
