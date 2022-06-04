/*
 * PIDVelocity.c
 *
 *  Created on: 4 Jun 2022
 *      Author: mobil
 */

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

	// Compute error of each term

    pidVelocity->proportionalOutput = (pidVelocity->Kp*error) - (pidVelocity->Kp * pidVelocity->Last1Error);

    pidVelocity->integratorOutput = (pidVelocity->Ki * pidVelocity->dt * error);

    pidVelocity->differentiatorOutput = ((pidVelocity->Kd*error)/pidVelocity->dt)
    									-((2.0 * pidVelocity->Kd * pidVelocity->Last1Error)/pidVelocity->dt)
    									+((pidVelocity->Kd * pidVelocity->Last2Error)/pidVelocity->dt)	;

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

}
