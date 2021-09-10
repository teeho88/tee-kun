#ifndef TINH_TOAN_H
#define TINH_TOAN_H

#include <stdbool.h>	//Boolean
#include <math.h>			//Pow()

struct Quaternion {
	float w, x, y, z;
};
extern float T ;
extern float Pitch, Roll, Head;
extern float ax, ay, az, wx, wy, wz;
extern float Vx, Vy, Vz;

struct Quaternion NhanQuat( struct Quaternion q1, struct Quaternion q2);
struct Quaternion LienhopQuat( struct Quaternion q);
void C_init(float a1, float a2, float a3);
void Q_init(float a1, float a2, float a3);
void updateC(void);
void updateV_Cosin(void);
void goc_Euler_Cosin(void);
void updateQ(void);
void updateV_Quat(void);
void goc_Euler_Quat(void);
void setW(float newWx, float newWy, float newWz);
void seta(float newax, float neway, float newaz);
		
#endif
