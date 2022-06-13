/*
 * Trajectory.c
 *
 *  Created on: 5 Jun 2022
 *      Author: mobil
 */


#include "Trajectory.h"


float VmaxOptimization(float Qinitial, float Qfinal){
	float Qrelative = Qfinal - Qinitial;
	float Vmax = -1;
	if(Qrelative >= 1.0  && Qrelative < 20.0)
	{
		Vmax = 0.7f;
	}
	else if(Qrelative >= 20.0  && Qrelative < 60.0)
	{
		Vmax = 3.0f;
	}
	else if(Qrelative >= 60.0  && Qrelative < 160.0)
	{
		Vmax = 6.0f;
	}
	else if(Qrelative >= 160.0)
	{
		Vmax = 8.0f;
	}
	// RPM to deg/sec
	return Vmax *360.0/60.0;
}

void CoefficientAndTimeCalculation(TrajectoryG *traject, float Qinitial, float Qfinal){

	traject -> Qin = Qinitial;
	traject -> Qfinal = Qfinal;
	// Set initial = 0;
	float Qrelative = Qfinal - Qinitial;
	// Set Vmax
	traject -> Vmax = VmaxOptimization(Qinitial,Qfinal);

	// Calculate time
	traject -> T[6] = (traject -> Amax/traject -> Jmax) + (traject -> Vmax/traject -> Amax) + (Qrelative/traject -> Vmax);
	traject -> T[0] = (traject -> Amax/traject -> Jmax);
	traject -> T[1] = (traject -> Vmax/traject -> Amax);
	traject -> T[2] = (traject -> Amax/traject -> Jmax) + (traject -> Vmax/traject -> Amax);
	traject -> T[3] = traject -> T[6] - traject -> T[2];
	traject -> T[4] = traject -> T[6] - traject -> T[1];
	traject -> T[5] = traject -> T[6] - traject -> T[0];

	traject -> A[0] = traject -> Jmax;
	traject -> A[1] = 0;
	traject -> A[2] = -1.0 * traject -> Jmax;
	traject -> A[3] = 0;
	traject -> A[4] = -1.0 * traject -> Jmax;
	traject -> A[5] = 0;
	traject -> A[6] = traject -> Jmax;

	traject -> B[0] = 0;
	traject -> B[1] = traject -> Amax;
	traject -> B[2] = traject -> Amax + (traject -> Jmax * traject -> T[1]);
	traject -> B[3] = 0;
	traject -> B[4] = traject -> Jmax * traject -> T[3];
	traject -> B[5] = (-1.0 * traject ->Amax);
	traject -> B[6] = (-1.0 * traject ->Amax) - (traject -> Jmax * traject -> T[5]);

	traject -> C[0] = 0;
	traject -> C[1] = ((traject -> A[0]*(traject -> T[0] * traject -> T[0]))/2+ traject -> B[0] * traject -> T[0] + traject -> C[0])
					-((traject -> A[1]*(traject -> T[0] * traject -> T[0]))/2+traject -> B[1]*traject -> T[0]);
	traject -> C[2] = ((traject -> A[1]*(traject -> T[1] * traject -> T[1]))/2+ traject -> B[1] * traject -> T[1] + traject -> C[1])
					-((traject -> A[2]*(traject -> T[1] * traject -> T[1]))/2+traject -> B[2]*traject -> T[1]);
	traject -> C[3] = ((traject -> A[2]*(traject -> T[2] * traject -> T[2]))/2+ traject -> B[2] * traject -> T[2] + traject -> C[2])
					-((traject -> A[3]*(traject -> T[2] * traject -> T[2]))/2+traject -> B[3]*traject -> T[2]);
	traject -> C[4] = ((traject -> A[3]*(traject -> T[3] * traject -> T[3]))/2+ traject -> B[3] * traject -> T[3] + traject -> C[3])
					-((traject -> A[4]*(traject -> T[3] * traject -> T[3]))/2+traject -> B[4]*traject -> T[3]);
	traject -> C[5] = ((traject -> A[4]*(traject -> T[4] * traject -> T[4]))/2+ traject -> B[4] * traject -> T[4] + traject -> C[4])
					-((traject -> A[5]*(traject -> T[4] * traject -> T[4]))/2+traject -> B[5]*traject -> T[4]);
	traject -> C[6] = ((traject -> A[5]*(traject -> T[5] * traject -> T[5]))/2+ traject -> B[5] * traject -> T[5] + traject -> C[5])
					-((traject -> A[6]*(traject -> T[5] * traject -> T[5]))/2+traject -> B[6]*traject -> T[5]);

	traject -> D[0] = 0;
	traject -> D[1] = ((traject -> A[0]*(traject -> T[0] * traject -> T[0] * traject -> T[0]))/6
					+ (traject -> B[0]*(traject -> T[0] * traject -> T[0]))/2 + traject -> C[0]*(traject -> T[0]) + traject -> D[0])
					- ((traject -> A[1]*(traject -> T[0] * traject -> T[0] * traject -> T[0]))/6
					+ (traject -> B[1]*(traject -> T[0] * traject -> T[0]))/2 + traject -> C[1]* traject -> T[0]);

	traject -> D[2] = ((traject -> A[1]*(traject -> T[1] * traject -> T[1] * traject -> T[1]))/6
					+ (traject -> B[1]*(traject -> T[1] * traject -> T[1]))/2 + traject -> C[1]*(traject -> T[1]) + traject -> D[1])
					- ((traject -> A[2]*(traject -> T[1] * traject -> T[1] * traject -> T[1]))/6
					+ (traject -> B[2]*(traject -> T[1] * traject -> T[1]))/2 + traject -> C[2]* traject -> T[1]);

	traject -> D[3] = ((traject -> A[2]*(traject -> T[2] * traject -> T[2] * traject -> T[2]))/6
					+ (traject -> B[2]*(traject -> T[2] * traject -> T[2]))/2 + traject -> C[2]*(traject -> T[2]) + traject -> D[2])
					- ((traject -> A[3]*(traject -> T[2] * traject -> T[2] * traject -> T[2]))/6
					+ (traject -> B[3]*(traject -> T[2] * traject -> T[2]))/2 + traject -> C[3]* traject -> T[2]);
	traject -> D[4] = ((traject -> A[3]*(traject -> T[3] * traject -> T[3] * traject -> T[3]))/6
					+ (traject -> B[3]*(traject -> T[3] * traject -> T[3]))/2 + traject -> C[3]*(traject -> T[3]) + traject -> D[3])
					- ((traject -> A[4]*(traject -> T[3] * traject -> T[3] * traject -> T[3]))/6
					+ (traject -> B[4]*(traject -> T[3] * traject -> T[3]))/2 + traject -> C[4]* traject -> T[3]);
	traject -> D[5] = ((traject -> A[4]*(traject -> T[4] * traject -> T[4] * traject -> T[4]))/6
					+ (traject -> B[4]*(traject -> T[4] * traject -> T[4]))/2 + traject -> C[4]*(traject -> T[4]) + traject -> D[4])
					- ((traject -> A[5]*(traject -> T[4] * traject -> T[4] * traject -> T[4]))/6
					+ (traject -> B[5]*(traject -> T[4] * traject -> T[4]))/2 + traject -> C[5]* traject -> T[4]);
	traject -> D[6] = ((traject -> A[5]*(traject -> T[5] * traject -> T[5] * traject -> T[5]))/6
					+ (traject -> B[5]*(traject -> T[5] * traject -> T[5]))/2 + traject -> C[5]*(traject -> T[5]) + traject -> D[5])
					- ((traject -> A[6]*(traject -> T[5] * traject -> T[5] * traject -> T[5]))/6
					+ (traject -> B[6]*(traject -> T[5] * traject -> T[5]))/2 + traject -> C[6]* traject -> T[5]);
}


float TrajectoryEvaluation(TrajectoryG *traject , uint64_t StartTime, uint64_t CurrentTime){
	// Microsec to sec
	static float t = 0;
	t  = (CurrentTime - StartTime)/1000000.0;

	if(t >= 0 && t < traject -> T[0])
	{
		traject -> QJ = traject -> A[0];
		traject -> QA = traject -> A[0]*t + traject -> B[0];
		traject -> QV = traject -> A[0]*(t*t)/2 + traject -> B[0]*t + traject -> C[0];
		traject -> QX = traject -> A[0]*(t*t*t)/6 + traject -> B[0]*(t*t)/2 + traject -> C[0]*t + traject -> D[0];
	}
	else if( t >= traject -> T[0] && t < traject -> T[1])
	{
		traject -> QJ = traject -> A[1];
		traject -> QA = traject -> A[1]*t + traject -> B[1];
		traject -> QV = traject -> A[1]*(t*t)/2 + traject -> B[1]*t + traject -> C[1];
		traject -> QX = traject -> A[1]*(t*t*t)/6 + traject -> B[1]*(t*t)/2 + traject -> C[1]*t + traject -> D[1];
	}
	else if( t >= traject -> T[1] && t < traject -> T[2])
	{
		traject -> QJ = traject -> A[2];
		traject -> QA = traject -> A[2]*t + traject -> B[2];
		traject -> QV = traject -> A[2]*(t*t)/2 + traject -> B[2]*t + traject -> C[2];
		traject -> QX = traject -> A[2]*(t*t*t)/6 + traject -> B[2]*(t*t)/2 + traject -> C[2]*t + traject -> D[2];
	}
	else if( t >= traject -> T[2] && t < traject -> T[3])
	{
		traject -> QJ = traject -> A[3];
		traject -> QA = traject -> A[3]*t + traject -> B[3];
		traject -> QV = traject -> A[3]*(t*t)/2 + traject -> B[3]*t + traject -> C[3];
		traject -> QX = traject -> A[3]*(t*t*t)/6 + traject -> B[3]*(t*t)/2 + traject -> C[3]*t + traject -> D[3];
	}
	else if( t >= traject -> T[3] && t < traject -> T[4])
	{
		traject -> QJ = traject -> A[4];
		traject -> QA = traject -> A[4]*t + traject -> B[4];
		traject -> QV = traject -> A[4]*(t*t)/2 + traject -> B[4]*t + traject -> C[4];
		traject -> QX = traject -> A[4]*(t*t*t)/6 + traject -> B[4]*(t*t)/2 + traject -> C[4]*t + traject -> D[4];
	}
	else if( t >= traject -> T[4] && t < traject -> T[5])
	{
		traject -> QJ = traject -> A[5];
		traject -> QA = traject -> A[5]*t + traject -> B[5];
		traject -> QV = traject -> A[5]*(t*t)/2 + traject -> B[5]*t + traject -> C[5];
		traject -> QX = traject -> A[5]*(t*t*t)/6 + traject -> B[5]*(t*t)/2 + traject -> C[5]*t + traject -> D[5];
	}
	else if( t >= traject -> T[5] && t < traject -> T[6])
	{
		traject -> QJ = traject -> A[6];
		traject -> QA = traject -> A[6]*t + traject -> B[6];
		traject -> QV = traject -> A[6]*(t*t)/2 + traject -> B[6]*t + traject -> C[6];
		traject -> QX = traject -> A[6]*(t*t*t)/6 + traject -> B[6]*(t*t)/2 + traject -> C[6]*t + traject -> D[6];
	}
	else
	{
		traject -> QJ = 0;
		traject -> QA = 0;
		traject -> QV = 0;
		traject -> QX = traject -> Qfinal;
	}

	return traject -> QV;
}
