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

	if(DistanceABS >= 130)
	{
//		traject -> Vmax = 60;
		traject -> Vmax = 51;
	}
	else
	{
//		traject -> Vmax = (-0.0000003*(DistanceABS*DistanceABS*DistanceABS*DistanceABS))+(0.00009*(DistanceABS*DistanceABS*DistanceABS))-(0.0115*(DistanceABS*DistanceABS))+(0.995*DistanceABS)+7.1259;
		traject -> Vmax = (-0.0000002*(DistanceABS*DistanceABS*DistanceABS*DistanceABS))+(0.00007*(DistanceABS*DistanceABS*DistanceABS))-(0.0092*(DistanceABS*DistanceABS))+(0.8419*DistanceABS)+4.8492;
	}

	if(traject -> Vmax > Veloinput)
	{
		traject -> Vmax = Veloinput;
	}
	// RPM to deg/sec with Direction
	traject -> Vmax =  traject -> Vmax *gain;
	traject -> Amax =  traject -> Amax *gain;
	traject -> Jmax =  traject -> Jmax *gain;

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
}


void TrajectoryEvaluation(TrajectoryG *traject , uint64_t StartTime, uint64_t CurrentTime, uint64_t PredictTime){
	// Microsec to sec
	static float t = 0;
	static float tP = 0;
	t  = (CurrentTime - StartTime)/1000000.0;
	tP = (PredictTime - StartTime)/1000000.0;

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
