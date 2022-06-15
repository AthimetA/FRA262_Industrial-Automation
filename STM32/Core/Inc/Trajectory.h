/*
 * Trajectory.h
 *
 *  Created on: 5 Jun 2022
 *      Author: mobil
 */

#ifndef INC_TRAJECTORY_H_
#define INC_TRAJECTORY_H_

#include "main.h"
typedef struct {

	// Max Acc
	float Amax;

	// Max jerk
	float Jmax;

	// Max Velocity
	float Vmax;

	// Coefficient A
	float A[7];
	// Coefficient B
	float B[7];
	// Coefficient C
	float C[7];
	// Coefficient D
	float D[7];
	// Time {t1,t2,t3,t4,t5,t6,t7(Tmax)}
	float T[7];

	// Current Location
	float Qin;
	// Current Location {Set as 0 Relative to current position}
	float QRelative;
	// Final Location
	float Qfinal;

	// Current Q
	float QJ;
	float QA;
	float QV;
	float QX;

} TrajectoryG;

float AbsVal(float number);
void CoefficientAndTimeCalculation(TrajectoryG *traject, float Qinitial, float Qfinal);
void TrajectoryEvaluation(TrajectoryG *traject , uint64_t StartTime, uint64_t CurrentTime);

#endif /* INC_TRAJECTORY_H_ */
