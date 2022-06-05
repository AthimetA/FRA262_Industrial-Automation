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
		Vmax = 10.0f;
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


float TrajectoryEvaluation(TrajectoryG *traject , uint64_t t){

}
