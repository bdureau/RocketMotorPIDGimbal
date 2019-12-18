#ifndef _KALMAN_H
#define _KALMAN_H
//Kalman Variables
extern float f_1;//=1.00000;  //cast as float
extern float kalman_x;
extern float kalman_x_last;
extern float kalman_p;
extern float kalman_p_last;
extern float kalman_k;
extern float kalman_q;
extern float kalman_r;
extern float kalman_x_temp;
extern float kalman_p_temp;
//end of Kalman Variables

extern void KalmanInit();
extern float KalmanCalc (float altitude);
#endif
