/*
 * File:        kalman.h
 * Project:     RocketMotorPIDGimbal V1.3
 * Author:      Boris du Reau
 * Description: Simple one-dimensional Kalman filter used to smooth the
 *              barometric altitude before it is used for liftoff, apogee
 *              and landing detection.
 *
 * Usage:       Call KalmanInit() once, then feed every raw altitude
 *              sample through KalmanCalc(). Each call advances the filter
 *              by one step, so it must be called exactly once per new
 *              sensor sample.
 */
#ifndef _KALMAN_H
#define _KALMAN_H

// Kalman filter state (defined in kalman.cpp)
extern float f_1;            // constant 1.0 cast as float
extern float kalman_x;       // current filtered estimate
extern float kalman_x_last;  // previous filtered estimate
extern float kalman_p;       // current estimate covariance
extern float kalman_p_last;  // previous estimate covariance
extern float kalman_k;       // Kalman gain
extern float kalman_q;       // measurement noise
extern float kalman_r;       // process noise
extern float kalman_x_temp;  // predicted estimate
extern float kalman_p_temp;  // predicted covariance

// Reset the filter state. Must be called before the first KalmanCalc().
extern void KalmanInit();

// Run one filter iteration on a raw altitude sample and return the
// filtered altitude.
extern float KalmanCalc(float altitude);

#endif
