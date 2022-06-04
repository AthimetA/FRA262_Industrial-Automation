/*
 * PIDVelocity.h
 *
 *  Created on: 4 Jun 2022
 *      Author: mobil
 */

#ifndef INC_PIDVELOCITY_H_
#define INC_PIDVELOCITY_H_

typedef struct {

	/* Controller gains */
	float Kp;
	float Ki;
	float Kd;

	/* Output limits */
	float OutputMin;
	float OutputMax;

	/* Sample time (in seconds) */
	float dt;

	/* Controller "memory" */
	float integratorOutput;
	float differentiatorOutput;
	float proportionalOutput;
	float Last1Error;
	float Last2Error;

	/* Controller output */
	float ControllerOut;
	float ControllerLastOut;

} PIDVelocityController;

void  PIDVelocityController_Init(PIDVelocityController *pidVelocity);
float PIDVelocityController_Update(PIDVelocityController *pidVelocity, float setpoint, float measurement);

#endif /* INC_PIDVELOCITY_H_ */
