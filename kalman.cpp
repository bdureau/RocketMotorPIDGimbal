/*
 * File:        kalman.cpp
 * Project:     RocketMotorPIDGimbal V1.3
 * Author:      Boris du Reau
 * Description: Implementation of the one-dimensional Kalman filter used
 *              to smooth the barometric altitude.
 *
 * Note:        Despite their names, kalman_r is used here as the process
 *              noise (added at the prediction step) and kalman_q as the
 *              measurement noise (used to compute the gain). The tuning
 *              values below were chosen empirically.
 */
#include "kalman.h"

// Kalman filter state
float f_1 = 1.00000;  // constant 1.0 cast as float
float kalman_x;
float kalman_x_last;
float kalman_p;
float kalman_p_last;
float kalman_k;
float kalman_q;
float kalman_r;
float kalman_x_temp;
float kalman_p_temp;

/*
   KalmanInit()
   Reset the filter state. Call once before any call to KalmanCalc().
*/
void KalmanInit()
{
  kalman_q = 4.0001;  // filter parameters, you can play around with them
  kalman_r = .20001;  // but these values appear to be fairly optimal

  kalman_x = 0;
  kalman_p = 0;
  kalman_x_temp = 0;
  kalman_p_temp = 0;

  kalman_x_last = 0;
  kalman_p_last = 0;
}

/*
   KalmanCalc()
   Run one filter iteration on the raw "altitude" sample and return the
   filtered value. This is the ASL altitude during the flight.
*/
float KalmanCalc(float altitude)
{
  // Predict
  kalman_x_temp = kalman_x_last;
  kalman_p_temp = kalman_p_last + kalman_r;

  // Update
  kalman_k = (f_1 / (kalman_p_temp + kalman_q)) * kalman_p_temp;
  kalman_x = kalman_x_temp + (kalman_k * (altitude - kalman_x_temp));
  kalman_p = (f_1 - kalman_k) * kalman_p_temp;

  // Save this state for the next iteration
  kalman_x_last = kalman_x;
  kalman_p_last = kalman_p;

  return kalman_x;
}
