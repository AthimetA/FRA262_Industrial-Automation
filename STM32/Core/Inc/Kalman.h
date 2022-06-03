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

	float32_t MATA;
	float32_t B;
	float32_t C;
	float32_t D;

} KalmanFilter;

void KalmanFilterFunction(KalmanFilter *kalman);

#endif /* INC_KALMAN_H_ */
