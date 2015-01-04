/** ******************************************************************************
  * @file    kalman.h                                                            *
  * @author  Liu heng                                                            *
  * @version V1.1.0                                                              *
  * @date    27-August-2013                                                      *
  * @brief   Hearder file for kalman filter                                      *
  *                                                                              *
  ********************************************************************************
  *                                                                              *
  ********************************************************************************/
#ifndef _KALMAN_H
#define _KALMAN_H

#include "stdlib.h"

typedef struct {
    float X_last; //optimal value at last moment
    float X_mid;  //predicted value at this moment
    float X_now;  //optimal value at this moment
    float P_mid;  //convariance of predicted value at this moment
    float P_now;  //convariance of optimal value at this moment
    float P_last; //convariance of optimal value at last moment
    float kg;     //kalman gain
    float A;      //arguments
    float Q;
    float R;
    float H;
}kalmanfilter;

void KalmanFilter_Init(kalmanfilter * filter,float T_Q,float T_R);
float KalmanFilter(kalmanfilter* p,float dat);

#endif
