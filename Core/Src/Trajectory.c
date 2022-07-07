/*
 * Trajectory.c
 *
 *  Created on: 5 Jun 2022
 *      Author: mobil
 */


#include "Trajectory.h"

void Robotinit(RobotManagement *Robot)
{
	  Robot -> Position = 0.0;
	  Robot -> Velocity = 0.0;
	  Robot -> Acceleration = 0.0;
	  Robot -> CurrentStation = 0;
	  Robot -> GoalPositon = 0.0;
	  Robot -> HomePositon = 0.0;
	  Robot -> QX = 0.0;
	  Robot -> QV = 0.0;
	  Robot -> QVMax = 0.0;

	  Robot -> flagSethome = 0;
	  Robot -> flagStartTime = 0;
	  Robot -> RunningFlag = 0;

	  Robot -> MotorIsOn = 0;

}

void TrajectorInit(TrajectoryG *traject)
{
	arm_mat_init_f32(&(traject ->MatTime), 6, 6, traject ->MatTime_Data);
	arm_mat_init_f32(&(traject ->MatTimeINV), 6, 6, traject ->MatTimeINV_Data);
	arm_mat_init_f32(&(traject ->MatCondition), 6, 1, traject ->MatCondition_Data);
	arm_mat_init_f32(&(traject ->MatA), 6, 1, traject ->MatTA_Data);
	traject -> TrajectoryMode = 0;
}

void RobotSetHome(RobotManagement *Robot , float homePoint)
{
	Robot -> HomePositon = homePoint;
}

float AbsVal(float number)
{
  if(number<0)
  {
    return number*-1.0;
  }
  else
  {
    return number;
  }
}

void CoefficientAndTimeCalculation(TrajectoryG *traject, float Qinitial, float Qfinal, float Veloinput){

	traject -> Qin = Qinitial;
	traject -> Qfinal = Qfinal;
	// Set initial = 0;
	traject -> QRelative = traject -> Qfinal - traject -> Qin;
	// Set Vmax Amax Jmax
	traject -> Vmax = 0.0;
	traject -> Amax = 21.77;
	traject -> Jmax = 114.59 ;
	float gain = 0.0;
	if(traject -> QRelative < 0.0)
	{
		gain = -1.0;
	}
	else
	{
		gain = 1.0;
	}
	// Find Speed limit
	float DistanceABS = AbsVal(traject -> QRelative);
	// Check Trajectory mode
	if(DistanceABS <= 0.0 && DistanceABS >= 0.0)
	{
		traject -> TrajectoryMode = 1;
	}
	else
	{
		traject -> TrajectoryMode = 0;
	}

	if(DistanceABS >= 130)
	{
//		traject -> Vmax = 60;
		traject -> Vmax = 51;
	}
	else
	{
//		traject -> Vmax = (-0.0000003*(DistanceABS*DistanceABS*DistanceABS*DistanceABS))+(0.00009*(DistanceABS*DistanceABS*DistanceABS))-(0.0115*(DistanceABS*DistanceABS))+(0.995*DistanceABS)+7.1259;
		traject -> Vmax = (-0.0000001*(DistanceABS*DistanceABS*DistanceABS*DistanceABS))+(0.00005*(DistanceABS*DistanceABS*DistanceABS))-(0.0082*(DistanceABS*DistanceABS))+(0.8134*DistanceABS)+4.0415;
		//		traject -> Vmax = (-0.0000002*(DistanceABS*DistanceABS*DistanceABS*DistanceABS))+(0.00007*(DistanceABS*DistanceABS*DistanceABS))-(0.0092*(DistanceABS*DistanceABS))+(0.8419*DistanceABS)+4.8492;
	}

	if(traject -> Vmax > Veloinput)
	{
		traject -> Vmax = Veloinput;
	}
	// RPM to deg/sec with Direction
	traject -> Vmax =  traject -> Vmax *gain;
	traject -> Amax =  traject -> Amax *gain;
	traject -> Jmax =  traject -> Jmax *gain;

	if(traject ->TrajectoryMode == 0) // S-curve
	{
		// Calculate time
			traject -> T[6] = (traject -> Amax/traject -> Jmax) + (traject -> Vmax/traject -> Amax) + (traject -> QRelative/traject -> Vmax);
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
			traject -> VMCal = traject -> Vmax;
	}
	else if(traject ->TrajectoryMode == 1) //Quintic
	{
		   // Calculate time
		   traject -> TimeInit = 0.0;
		   traject -> TimeFinal = (traject -> Amax/traject -> Jmax) + (traject -> Vmax/traject -> Amax) + (traject -> QRelative/traject -> Vmax);

		   traject -> MatTime_Data[0] = 1.0;
		   traject -> MatTime_Data[1] = traject -> TimeInit;
		   traject -> MatTime_Data[2] = traject -> TimeInit*traject -> TimeInit;
		   traject -> MatTime_Data[3] = traject -> TimeInit*traject -> TimeInit*traject -> TimeInit;
		   traject -> MatTime_Data[4] = traject -> TimeInit*traject -> TimeInit*traject -> TimeInit*traject -> TimeInit;
		   traject -> MatTime_Data[5] = traject -> TimeInit*traject -> TimeInit*traject -> TimeInit*traject -> TimeInit*traject -> TimeInit;
		   traject -> MatTime_Data[6] = 0.0;
		   traject -> MatTime_Data[7] = 1.0;
		   traject -> MatTime_Data[8] = 2.0*traject -> TimeInit;
		   traject -> MatTime_Data[9] = 3.0*traject -> TimeInit*traject -> TimeInit;
		   traject -> MatTime_Data[10] = 4.0*traject -> TimeInit*traject -> TimeInit*traject -> TimeInit;
		   traject -> MatTime_Data[11] = 5.0*traject -> TimeInit*traject -> TimeInit*traject -> TimeInit*traject -> TimeInit;
		   traject -> MatTime_Data[12] = 0.0;
		   traject -> MatTime_Data[13] = 0.0;
		   traject -> MatTime_Data[14] = 2.0;
		   traject -> MatTime_Data[15] = 6.0*traject -> TimeInit;
		   traject -> MatTime_Data[16] = 12.0*traject -> TimeInit*traject -> TimeInit;
		   traject -> MatTime_Data[17] = 20.0*traject -> TimeInit*traject -> TimeInit*traject -> TimeInit;
		   traject -> MatTime_Data[18] = 1.0;
		   traject -> MatTime_Data[19] = traject -> TimeFinal;
		   traject -> MatTime_Data[20] = traject -> TimeFinal*traject -> TimeFinal;
		   traject -> MatTime_Data[21] = traject -> TimeFinal*traject -> TimeFinal*traject -> TimeFinal;
		   traject -> MatTime_Data[22] = traject -> TimeFinal*traject -> TimeFinal*traject -> TimeFinal*traject -> TimeFinal;
		   traject -> MatTime_Data[23] = traject -> TimeFinal*traject -> TimeFinal*traject -> TimeFinal*traject -> TimeFinal*traject -> TimeFinal;
		   traject -> MatTime_Data[24] = 0.0;
		   traject -> MatTime_Data[25] = 1.0;
		   traject -> MatTime_Data[26] = 2.0*traject -> TimeFinal;
		   traject -> MatTime_Data[27] = 3.0*traject -> TimeFinal*traject -> TimeFinal;
		   traject -> MatTime_Data[28] = 4.0*traject -> TimeFinal*traject -> TimeFinal*traject -> TimeFinal;
		   traject -> MatTime_Data[29] = 5.0*traject -> TimeFinal*traject -> TimeFinal*traject -> TimeFinal*traject -> TimeFinal;
		   traject -> MatTime_Data[30] = 0.0;
		   traject -> MatTime_Data[31] = 0.0;
		   traject -> MatTime_Data[32] = 2.0;
		   traject -> MatTime_Data[33] = 6.0*traject -> TimeFinal;
		   traject -> MatTime_Data[34] = 12.0*traject -> TimeFinal*traject -> TimeFinal;
		   traject -> MatTime_Data[35] = 20.0*traject -> TimeFinal*traject -> TimeFinal*traject -> TimeFinal;

		   traject -> MatCondition_Data[0] = traject -> Qin;
		   traject -> MatCondition_Data[1] = 0;
		   traject -> MatCondition_Data[2] = 0;
		   traject -> MatCondition_Data[3] = traject -> Qfinal;
		   traject -> MatCondition_Data[4] = 0;
		   traject -> MatCondition_Data[5] = 0;

		   traject -> Trajectorystatus = arm_mat_inverse_f32(&(traject ->MatTime), &(traject ->MatTimeINV));
		   traject -> Trajectorystatus = arm_mat_mult_f32(&(traject ->MatTimeINV), &(traject ->MatCondition), &(traject ->MatA));

		   float t = (traject -> TimeFinal)/2.0;
		   traject -> VMCal = (traject -> MatTA_Data[1]) + (2*traject -> MatTA_Data[2]*t) + (3*traject -> MatTA_Data[3]*(t*t)) + (4*traject -> MatTA_Data[4]*(t*t*t)) + (5*traject -> MatTA_Data[5]*(t*t*t*t));
	}
}

void TrajectoryEvaluation(TrajectoryG *traject , uint64_t StartTime, uint64_t CurrentTime, uint64_t PredictTime){
   // Microsec to sec
   static float t = 0;
   static float tP = 0;
   t  = (CurrentTime - StartTime)/1000000.0;
   tP = (PredictTime - StartTime)/1000000.0;

	if(traject ->TrajectoryMode == 0) // S-curve
	{
		if(t >= 0 && t < traject -> T[0])
		   {
		      traject -> QJ = traject -> A[0];
		      traject -> QA = traject -> A[0]*t + traject -> B[0];
		      traject -> QV = traject -> A[0]*(t*t)/2 + traject -> B[0]*t + traject -> C[0];
		      traject -> QVP = traject -> A[0]*(tP*tP)/2 + traject -> B[0]*tP + traject -> C[0];
		      traject -> QX = traject -> Qin + traject -> A[0]*(t*t*t)/6 + traject -> B[0]*(t*t)/2 + traject -> C[0]*t + traject -> D[0];
		   }
		   else if( t >= traject -> T[0] && t < traject -> T[1])
		   {
		      traject -> QJ = traject -> A[1];
		      traject -> QA = traject -> A[1]*t + traject -> B[1];
		      traject -> QV = traject -> A[1]*(t*t)/2 + traject -> B[1]*t + traject -> C[1];
		      traject -> QVP = traject -> A[1]*(tP*tP)/2 + traject -> B[1]*tP + traject -> C[1];
		      traject -> QX = traject -> Qin + traject -> A[1]*(t*t*t)/6 + traject -> B[1]*(t*t)/2 + traject -> C[1]*t + traject -> D[1];
		   }
		   else if( t >= traject -> T[1] && t < traject -> T[2])
		   {
		      traject -> QJ = traject -> A[2];
		      traject -> QA = traject -> A[2]*t + traject -> B[2];
		      traject -> QV = traject -> A[2]*(t*t)/2 + traject -> B[2]*t + traject -> C[2];
		      traject -> QVP = traject -> A[2]*(tP*tP)/2 + traject -> B[2]*tP + traject -> C[2];
		      traject -> QX = traject -> Qin + traject -> A[2]*(t*t*t)/6 + traject -> B[2]*(t*t)/2 + traject -> C[2]*t + traject -> D[2];
		   }
		   else if( t >= traject -> T[2] && t < traject -> T[3])
		   {
		      traject -> QJ = traject -> A[3];
		      traject -> QA = traject -> A[3]*t + traject -> B[3];
		      traject -> QV = traject -> A[3]*(t*t)/2 + traject -> B[3]*t + traject -> C[3];
		      traject -> QVP = traject -> A[3]*(tP*tP)/2 + traject -> B[3]*tP + traject -> C[3];
		      traject -> QX = traject -> Qin + traject -> A[3]*(t*t*t)/6 + traject -> B[3]*(t*t)/2 + traject -> C[3]*t + traject -> D[3];
		   }
		   else if( t >= traject -> T[3] && t < traject -> T[4])
		   {
		      traject -> QJ = traject -> A[4];
		      traject -> QA = traject -> A[4]*t + traject -> B[4];
		      traject -> QV = traject -> A[4]*(t*t)/2 + traject -> B[4]*t + traject -> C[4];
		      traject -> QVP = traject -> A[4]*(tP*tP)/2 + traject -> B[4]*tP + traject -> C[4];
		      traject -> QX = traject -> Qin + traject -> A[4]*(t*t*t)/6 + traject -> B[4]*(t*t)/2 + traject -> C[4]*t + traject -> D[4];
		   }
		   else if( t >= traject -> T[4] && t < traject -> T[5])
		   {
		      traject -> QJ = traject -> A[5];
		      traject -> QA = traject -> A[5]*t + traject -> B[5];
		      traject -> QV = traject -> A[5]*(t*t)/2 + traject -> B[5]*t + traject -> C[5];
		      traject -> QVP = traject -> A[5]*(tP*tP)/2 + traject -> B[5]*tP + traject -> C[5];
		      traject -> QX = traject -> Qin + traject -> A[5]*(t*t*t)/6 + traject -> B[5]*(t*t)/2 + traject -> C[5]*t + traject -> D[5];
		   }
		   else if( t >= traject -> T[5] && t < traject -> T[6])
		   {
		      traject -> QJ = traject -> A[6];
		      traject -> QA = traject -> A[6]*t + traject -> B[6];
		      traject -> QV = traject -> A[6]*(t*t)/2 + traject -> B[6]*t + traject -> C[6];
		      traject -> QVP = traject -> A[6]*(tP*tP)/2 + traject -> B[6]*tP + traject -> C[6];
		      traject -> QX = traject -> Qin + traject -> A[6]*(t*t*t)/6 + traject -> B[6]*(t*t)/2 + traject -> C[6]*t + traject -> D[6];
		   }
		   else
		   {
		      traject -> QJ = 0;
		      traject -> QA = 0;
		      traject -> QV = 0;
		      traject -> QVP = 0;
		      traject -> QX = traject -> Qfinal;
		   }
	}
	else if(traject ->TrajectoryMode == 1) //Quintic
	{
	   if(t >= 0 && t < traject -> TimeFinal)
	   {
			 traject -> QA = (2*traject -> MatTA_Data[2]) + (6*traject -> MatTA_Data[3]*t) + (12*traject -> MatTA_Data[4]*(t*t)) + (20*traject -> MatTA_Data[5]*(t*t*t));

			 traject -> QV = (traject -> MatTA_Data[1]) + (2*traject -> MatTA_Data[2]*t) + (3*traject -> MatTA_Data[3]*(t*t)) + (4*traject -> MatTA_Data[4]*(t*t*t)) + (5*traject -> MatTA_Data[5]*(t*t*t*t));

			 traject -> QVP = (traject -> MatTA_Data[1]) + (2*traject -> MatTA_Data[2]*tP) + (3*traject -> MatTA_Data[3]*(tP*tP)) + (4*traject -> MatTA_Data[4]*(tP*tP*tP)) + (5*traject -> MatTA_Data[5]*(tP*tP*tP*tP));

			 traject -> QX = (traject -> MatTA_Data[0]) + (traject -> MatTA_Data[1]*t) + (traject -> MatTA_Data[2]*(t*t)) + (traject -> MatTA_Data[3]*(t*t*t)) + (traject -> MatTA_Data[4]*(t*t*t*t))+ (traject -> MatTA_Data[5]*(t*t*t*t*t));
	   }
	   else
	   {
		  traject -> QJ = 0;
		  traject -> QA = 0;
		  traject -> QV = 0;
		  traject -> QVP = 0;
		  traject -> QX = traject -> Qfinal;
	   }
	}
   return 1.0;
}

