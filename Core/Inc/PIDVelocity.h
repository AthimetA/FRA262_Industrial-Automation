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

	/* Controller "memory" */
	float integratorOutput;
	float differentiatorOutput;
	float proportionalOutput;
	float Last1Error;
	float Last2Error;

	/* Controller output */
	float ControllerOut;
	float ControllerLastOut;

	float KpUse;
} PIDAController;

void PIDAController_Init(PIDAController *pid);
float PIDAVelocityController_Update(PIDAController *pid, float setpoint, float measurement,float VMCal);
float PIDAPositonController_Update(PIDAController *pid, float setpoint, float measurement, float Distance);

#endif /* INC_PIDVELOCITY_H_ */
