/*
 * Kalman.h
 *
 *  Created on: Jun 2, 2022
 *      Author: mobil
 */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_
#include "arm_math.h"

typedef struct {

	// Matrix A
	float32_t MatA_Data[9];
	// Matrix B
	float32_t MatB_Data[3];
	// Matrix C
	float32_t MatC_Data[6];
	// Matrix D
	float32_t MatD_Data[1];
	// Matrix Q
	float32_t MatQ_Data[9];
	// Matrix R
	float32_t MatR_Data[4];
	// Matrix G
	float32_t MatG_Data[9];
	// Matrix State
	float32_t MatState_Data[3];
	// Matrix State Last
	float32_t MatStateLast_Data[3];
	// Matrix Predict
	float32_t MatPredict_Data[9];
	// Matrix Predict Last
	float32_t MatPredictLast_Data[9];
	// Matrix Y
	float32_t MatY_Data[2];
	// Matrix Z
	float32_t MatZ_Data[2];
	// Matrix S
	float32_t MatS_Data[4];
	// Matrix Kalman gain
	float32_t MatK_Data[6];
	// Matrix Iden
	float32_t MatI_Data[9];
	/* Matrix Buffer */
	// Matrix At
	float32_t MatAt_Data[9];
	// Matrix Gt
	float32_t MatGt_Data[9];
	// Matrix GQ
	float32_t MatGQ_Data[9];
	// Matrix Ct
	float32_t MatCt_Data[6];
	// Matrix Sinv
	float32_t MatSinv_Data[4];
	// Matrix GQGt
	float32_t MatGQGt_Data[9];
	// Matrix CPk
	float32_t MatCPk_Data[6];
	// Matrix APK
	float32_t MatAPk_Data[9];
	// Matrix APKAt
	float32_t MatAPkAt_Data[9];
	// Matrix CXk
	float32_t MatCXk_Data[2];
	// Matrix CPkCt
	float32_t MatCPkCt_Data[4];
	// Matrix PkCt
	float32_t MatPkCt_Data[6];
	// Matrix KYk
	float32_t MatKYk_Data[3];
	// Matrix KC
	float32_t MatKC_Data[9];
	// Matrix IKC
	float32_t MatI_KC_Data[9];
	arm_matrix_instance_f32 MatA;
	arm_matrix_instance_f32 MatB;
	arm_matrix_instance_f32 MatC;
	arm_matrix_instance_f32 MatD;
	arm_matrix_instance_f32 MatQ;
	arm_matrix_instance_f32 MatR;
	arm_matrix_instance_f32 MatG;
	arm_matrix_instance_f32 MatState;
	arm_matrix_instance_f32 MatStateLast;
	arm_matrix_instance_f32 MatPredict;
	arm_matrix_instance_f32 MatPredictLast;
	arm_matrix_instance_f32 MatY;
	arm_matrix_instance_f32 MatZ;
	arm_matrix_instance_f32 MatS;
	arm_matrix_instance_f32 MatK;
	arm_matrix_instance_f32 MatI;
	arm_matrix_instance_f32 MatAt;
	arm_matrix_instance_f32 MatGt;
	arm_matrix_instance_f32 MatGQ;
	arm_matrix_instance_f32 MatCt;
	arm_matrix_instance_f32 MatSinv;
	arm_matrix_instance_f32 MatGQGt;
	arm_matrix_instance_f32 MatCPk;
	arm_matrix_instance_f32 MatAPk;
	arm_matrix_instance_f32 MatAPkAt;
	arm_matrix_instance_f32 MatCXk;
	arm_matrix_instance_f32 MatCPkCt;
	arm_matrix_instance_f32 MatPkCt;
	arm_matrix_instance_f32 MatKYk;
	arm_matrix_instance_f32 MatKC;
	arm_matrix_instance_f32 MatI_KC;

	// Status
	arm_status Kalmanstatus;

} KalmanFilterVar;

void KalmanMatrixReset(KalmanFilterVar *KalmanVar , float Pvar);
void KalmanMatrixInit(KalmanFilterVar *KalmanVar);
void KalmanFilterFunction(KalmanFilterVar *KalmanVar,float32_t PositionDeg,float32_t VelocityDeg);

#endif /* INC_KALMAN_H_ */
