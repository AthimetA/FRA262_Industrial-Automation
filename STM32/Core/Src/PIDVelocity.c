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
    float errorDZ = error;
//    float deadzone = 11.0;

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
//    	pidVelocity->KpUse = pidVelocity->Kp*10.0;
//    }
//    else
//    {
//    	pidVelocity->KpUse = pidVelocity->Kp;
//    }

	// Compute error of each term

    pidVelocity->proportionalOutput = (pidVelocity->Kp*errorDZ) - (pidVelocity->Kp * pidVelocity->Last1Error);

    pidVelocity->integratorOutput = (pidVelocity->Ki * errorDZ);

    pidVelocity->differentiatorOutput = pidVelocity->Kd *(errorDZ -(2.0* pidVelocity->Last1Error) + pidVelocity->Last2Error)	;

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
	pidVelocity->Last2Error = pidVelocity->Last1Error;
	pidVelocity->Last1Error = errorDZ;

	return pidVelocity->ControllerOut;
}
