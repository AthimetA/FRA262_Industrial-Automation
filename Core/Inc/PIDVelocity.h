/*
 * PIDVelocity.h
 *
 *  Created on: 4 Jun 2022
 *      Author: mobil
 */

#ifndef INC_PIDVELOCITY_H_
#define INC_PIDVELOCITY_H_

#include "Trajectory.h"

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

} PIDAController;

void PIDAController_Init(PIDAController *pid);
float PIDAVelocityController_Update(PIDAController *pid, TrajectoryG *traject, float setpoint, float measurement,float VfromTraject,float VMCal);
float PIDAPositonController_Update(PIDAController *pid, TrajectoryG *traject, float setpoint, float measurement,float VfromTraject,float VMCal);

#endif /* INC_PIDVELOCITY_H_ */
