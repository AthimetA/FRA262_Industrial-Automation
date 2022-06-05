/*
 * Trajectory.h
 *
 *  Created on: 5 Jun 2022
 *      Author: mobil
 */

#ifndef INC_TRAJECTORY_H_
#define INC_TRAJECTORY_H_

typedef struct {

	// Max Acc
	float Amax;

	// Max jerk
	float Jmax;

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

	// Current Location {Set as 0 Relative}
	float Qin;
	// Final Location
	float Qfinal;

	// Max Velocity
	float Vmax;

} TrajectoryG;

float VmaxOptimization(float Qinitial, float Qfinal);
void CoefficientAndTimeCalculation(TrajectoryG *traject, float Qinitial, float Qfinal);
float TrajectoryEvaluation(TrajectoryG *traject , uint64_t t);

#endif /* INC_TRAJECTORY_H_ */
