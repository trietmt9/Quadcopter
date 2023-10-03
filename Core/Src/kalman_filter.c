#include <kalman_filter.h>
uint32_t timer;
double Kalman_Filter(kalman_t *KalmanData, double Gyro, double Accel, float dt)
{
    /* Time Update "Predict" */
    double rate = Gyro - KalmanData->theta;
    KalmanData->theta_dot += rate*dt;
    
    // P_k = Ax P_k-1 + Q_theta_dot 
    KalmanData->P[0][0] += dt*(dt*KalmanData->P[1][1]-KalmanData->P[0][1] - KalmanData->P[1][0] + KalmanData->Q_theta_dot);
    KalmanData->P[0][1] -= KalmanData->P[1][1]*dt;
    KalmanData->P[1][0] -= KalmanData->P[1][1]*dt;
    KalmanData->P[1][1] += KalmanData->Q_theta*dt;

    /* Measurement Update "Correct" */
    double S = KalmanData->P[0][0] + KalmanData->R; 
    KalmanData->K[0] = KalmanData->P[0][0] / S;  
    KalmanData->K[1] = KalmanData->P[1][0] / S;

    double y = Accel - KalmanData->theta_dot; // z_k - x_k-1 
    KalmanData->theta_dot += KalmanData->K[0] * y;
    KalmanData->theta += KalmanData->K[1] * y; 

    double P_00 = KalmanData->P[0][0];
    double P_01 = KalmanData->P[0][1];
    KalmanData->P[0][0] -= KalmanData->K[0] * P_00;
    KalmanData->P[0][1] -= KalmanData->K[0] * P_01;
    KalmanData->P[1][0] -= KalmanData->K[1] * P_00;
    KalmanData->P[1][1] -= KalmanData->K[1] * P_01;
    
    return KalmanData->theta_dot;   
}