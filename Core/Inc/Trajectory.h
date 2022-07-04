/*
 * Trajectory.h
 *
 *  Created on: 5 Jun 2022
 *      Author: mobil
 */

#ifndef INC_TRAJECTORY_H_
#define INC_TRAJECTORY_H_

#include "main.h"
#include "arm_math.h"

typedef struct {

	float32_t MatTime_Data[36];
	float32_t MatTimeINV_Data[36];
	float32_t MatCondition_Data[6];
	float32_t MatTA_Data[6];

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

	// Matrix A
	float TimeInit;
	float TimeFinal;

	arm_matrix_instance_f32 MatTime;
	arm_matrix_instance_f32 MatTimeINV;
	arm_matrix_instance_f32 MatCondition;
	arm_matrix_instance_f32 MatA;

	// Status
	arm_status Trajectorystatus;

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
void TrajectorInit(TrajectoryG *traject);
void CoefficientAndTimeCalculation(TrajectoryG *traject, float Qinitial, float Qfinal, float Velomax);
void TrajectoryEvaluation(TrajectoryG *traject , uint64_t StartTime, uint64_t CurrentTime,uint64_t PredictTime);

#endif /* INC_TRAJECTORY_H_ */
