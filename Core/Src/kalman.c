#include "kalman.h"

float x[6] = {0};
float D[6] = {1000};

float *KalmanFilter(float z[6])
{
	for(int i = 0; i<6; i++)
	{
		float D_ = D[i] + Q;
		float K = D_/(D_ + R);
		x[i] = x[i] + K*(z[i] - x[i]);
		D[i] = (1 - K)*D_;
	}
	return x;
}
