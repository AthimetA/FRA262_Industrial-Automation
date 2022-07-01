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
	float QVP;
	float QX;

} TrajectoryG;

typedef struct {
	// Current Status
	float Position;
	float Velocity;
	uint16_t CurrentStation;
	// Goal Position, Station
	float GoalPositon;
	uint16_t GoalStation;
	// Home Data
	float HomePositon;
	// Trajectory Data
	float QX;
	float QV;
	float QVMax;
	// On/Off Motor
	uint8_t MotorIsOn;
	// flag
	uint8_t flagStartTime;
	uint8_t flagSethome;
	uint8_t RunningFlag;
} RobotManagement;

void Robotinit(RobotManagement *Robot);
void RobotSetHome(RobotManagement *Robot , float homePoint);
float AbsVal(float number);
void CoefficientAndTimeCalculation(TrajectoryG *traject, float Qinitial, float Qfinal, float Velomax);
void TrajectoryEvaluation(TrajectoryG *traject , uint64_t StartTime, uint64_t CurrentTime,uint64_t PredictTime);

#endif /* INC_TRAJECTORY_H_ */
