/** ***************************************************************************************
  * @file    kalman.c                                                                     *
  * @author  Liu heng                                                                     *
  * @version V1.1.0                                                                       *
  * @date    27-August-2013                                                               *
  * @brief   The implementation of first-order kalman filter for linear filter
  *****************************************************************************************
  *                          example                                                    *
  *          kalman *p;                                                                   *
  *          float SersorData;                                                            *
  *          p =   kalmanCreate(20,200);                                                  *
  *          while(1)                                                                     *
  *          {                                                                            *
  *             SersorData = sersor();                                                    *
  *             SersorData = KalmanFilter(p,SersorData);                                  *
  *             printf("%2.2f",SersorData);                                               *
  *          }                                                                            *
  *****************************************************************************************/

#include "kalmanfilter.h"

/**
  * @name   kalmanCreate
  * @brief  initialize a kalman filter
  * @param  *filter: filter pointer
  *             T_Q: covariance of process noise
  *             T_R: covariance of observation noise
  * @retval void
  */
void KalmanFilter_Init(kalmanfilter *filter,float T_Q,float T_R)
{
    filter->X_last = (float)0;
    filter->P_last = 0;
    filter->Q = T_Q;
    filter->R = T_R;
    filter->A = 1;
    filter->H = 1;
    filter->X_mid = filter->X_last;
}

/**
  * @name   KalmanFilter
  * @brief  kalman filter
  * @param  p:  filter
  *         dat: raw data
  * @retval filterd data
  */

float KalmanFilter(kalmanfilter* filter,float dat)
{
    filter->X_mid =filter->A*filter->X_last;                     //x(k|k-1) = AX(k-1|k-1)+BU(k)
    filter->P_mid = filter->A*filter->P_last+filter->Q;               //p(k|k-1) = Ap(k-1|k-1)A'+Q
    filter->kg = filter->P_mid/(filter->P_mid+filter->R);             //kg(k) = p(k|k-1)H'/(Hp(k|k-1)'+R)
    filter->X_now = filter->X_mid+filter->kg*(dat-filter->X_mid);     //x(k|k) = X(k|k-1)+kg(k)(Z(k)-HX(k|k-1))
    filter->P_now = (1-filter->kg)*filter->P_mid;                //p(k|k) = (I-kg(k)H)P(k|k-1)
    filter->P_last = filter->P_now;                        
    filter->X_last = filter->X_now;
    return filter->X_now;
}
