#include <stm32f4xx.h>

typedef struct
{
    float theta; // bias
    float theta_dot; // angle
    float P[2][2];
    float K[2]; 
    float z_k;

    float Q_theta_dot; // angle Q
    float Q_theta; // bias Q 
    float R;
}kalman_t;

double Kalman_Filter(kalman_t *KalmanData, double Gyro, double Accel);