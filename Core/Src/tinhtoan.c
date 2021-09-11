#include <tinhtoan.h>
#include <stdbool.h>	//Boolean
#include <math.h>		//Pow()
#include <stdlib.h>

/* Cac truc X -> Pitch / ax 
            Y -> Roll / ay (huong mui)
						Z -> Head / az
*/
struct Quaternion q;
static float C[3][3] = {{0,0,0},{0,0,0},{0,0,0}};

void C_init(float a1, float a2, float a3){
	float g = 9.8;
	Pitch = asin(a2/g);
	Roll = asin(-a1/(g*cos(Pitch)));
	Head = 0;
	
/*	// CNb
	C[0][0] = 1;
  C[0][1] = Pitch*Roll;
	C[0][2] = -Roll;
	C[1][0] = 0;
	C[1][1] = 1;
	C[1][2] = Pitch;
	C[2][0] = Roll;
	C[2][1] = -Pitch*Roll;
	C[2][2] = 1;
*/
	
	// CbN
	C[0][0] = 1;
  C[0][1] = 0;
	C[0][2] = Roll;
	C[1][0] = Pitch*Roll;
	C[1][1] = 1;
	C[1][2] = -Pitch*Roll;
	C[2][0] = -Roll;
	C[2][1] = Pitch;
	C[2][2] = 1;
}

void updateC(void) {
	/* CbN */
	float C00, C01, C02, C10, C11, C12, C20, C21, C22;
	C00 = C[0][0] + T*(wz*C[0][1]-wy*C[0][2]);
	C01 = C[0][1] + T*(-wz*C[0][0]+wx*C[0][2]);
	C02 = C[0][2] + T*(wy*C[0][0]-wx*C[0][1]);
	C10 = C[1][0] + T*(wz*C[1][1]-wy*C[1][2]);
	C11 = C[1][1] + T*(-wz*C[1][0]+wx*C[1][2]);
	C12 = C[1][2] + T*(wy*C[1][0]-wx*C[1][1]);
	C20 = C[2][0] + T*(wz*C[2][1]-wy*C[2][2]);
	C21 = C[2][1] + T*(-wz*C[2][0]+wx*C[2][2]);
	C22 = C[2][2] + T*(wy*C[2][0]-wx*C[2][1]);
	
	// update
	C[0][0] = C00; C[0][1] = C01; C[0][2] = C02;
	C[1][0] = C10; C[1][1] = C11; C[1][2] = C12;
	C[2][0] = C20; C[2][1] = C21; C[2][2] = C22;		
}

void goc_Euler_Cosin(void) {
	 // Note: convert CbN to CNb
	 float C0 = sqrtf(powf(C[2][0],2)+powf(C[2][2],2));
	 Pitch = atanf(C[2][1]/C0);
	 Roll = - atanf(C[2][0]/C[2][2]);
	 Head = atanf(C[0][1]/C[1][1]);
}

void updateV_Cosin(void) {
		/* V in Navigation */
		Vx += T*(C[0][0]*ax + C[0][1]*ay + C[0][2]*az);
		Vy += T*(C[1][0]*ax + C[1][1]*ay + C[1][2]*az);
}

void Q_init(float a1, float a2, float a3){
	float g = 9.8;
	Pitch = asin(a2/g);
	Roll = asin(-a1/(g*cos(Pitch)));
	Head = 0;
	
	// Abbreviations for the various angular functions
  float cy = cos(Head * 0.5);
	float sy = sin(Head * 0.5);
	float cp = cos(Pitch * 0.5);
	float sp = sin(Pitch * 0.5);
	float cr = cos(Roll * 0.5);
	float sr = sin(Roll * 0.5);
	
	// QNb
	q.w = cr * cp * cy + sr * sp * sy;
	q.x = sr * cp * cy - cr * sp * sy;
	q.y = cr * sp * cy + sr * cp * sy;
	q.z = cr * cp * sy - sr * sp * cy;
}

void updateQ(void){
	// QNb
	struct Quaternion q1;
	q1.w = 1;
	q1.x = 0.5*T*wx;
	q1.y = 0.5*T*wy;
	q1.z = 0.5*T*wz;
	
	q = NhanQuat(q, q1);
}

void goc_Euler_Quat(void){
	// Pitch (x-axis rotation)
	float sinp = 2 * (q.w * q.y - q.z * q.x);
	if (abs(sinp) >= 1)
			Pitch = copysign(3.14 / 2, sinp); // use 90 degrees if out of range
	else
			Pitch = asin(sinp);
	
	// Roll (y-axis rotation)
	float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
	float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
	Roll = atan2(sinr_cosp, cosr_cosp);

	// Head (z-axis rotation)
	float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
	double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
	Head = atan2(siny_cosp, cosy_cosp);
}

void updateV_Quat(void){
	// Note: qbN = qNb* -> convert qNb to qbN
	struct Quaternion aN, ab;
	ab.w = 0; ab.x = ax; ab.y = ay; ab.z = az;
	aN = NhanQuat(NhanQuat(LienhopQuat(q), ab), q);
	Vx += T*aN.x;
	Vy += T*aN.y;
}

struct Quaternion NhanQuat( struct Quaternion q1, struct Quaternion q2){
	struct Quaternion q0;
	q0.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
	q0.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
	q0.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
	q0.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
	return q0;
}

struct Quaternion LienhopQuat( struct Quaternion q){
	struct Quaternion qT;
	qT.w = q.w;
	qT.x = -q.x;
	qT.y = -q.y;
	qT.z = -q.z;
	return qT;
}

void setW(float newWx, float newWy, float newWz) { wx = newWx; wy = newWy; wz = newWz; }

void seta(float newax, float neway, float newaz) { ax = newax; ay = neway; az = newaz; }
