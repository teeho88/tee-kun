#include <tinhtoan.h>
#include <stdlib.h>

/* Cac truc X -> Pitch / ax 
            Y -> Roll / ay (huong mui)
						Z -> Head / az
*/
struct Quaternion q;
static float C[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
struct Quaternion aN;
// for madgwick
float beta;
float zeta;
float gbiasx = 0, gbiasy = 0, gbiasz = 0;  			// gyro bias error

void C_init(float a1, float a2, float a3){
	// Normalise accelerometer measurement
	float norm = sqrt(a1 * a1 + a2 * a2 + a3 * a3);
	if (norm == 0.0f) return; // handle NaN
	a1 = a1 / norm;
	a2 = a1 / norm;
	Roll = asin(a2);
	Pitch = asin(-a1/cos(Roll));
	Head = 0;
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
	float g = 9.8;
		Vx += T*(C[0][0]*ax + C[0][1]*ay + C[0][2]*az)*g;
		Vy += T*(C[1][0]*ax + C[1][1]*ay + C[1][2]*az)*g;
}

void Q_init(float a1, float a2, float a3){
	// for madgwick
	float PI = 3.14159f;
	float GyroMeasError = PI * (60.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
	float GyroMeasDrift = PI * (0.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
	beta = sqrt(3.0f/4.0f) * GyroMeasError;  	 // compute beta
	zeta = sqrt(3.0f/4.0f) * GyroMeasDrift;    // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
	// Normalise accelerometer measurement
	float norm = sqrt(a1 * a1 + a2 * a2 + a3 * a3);
	if (norm == 0) return; // handle NaN
	a1 = a1/norm;
	a2 = a2/norm;
	Roll = asin(a2);
	Pitch = asin(-a1/cos(Roll));
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
	norm = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
	q.w = q.w/norm; q.x = q.x/norm; q.y = q.y/norm; q.z = q.z/norm;
}

void updateQ(void){
	// QNb
	struct Quaternion q1;
	q1.w = 1;
	q1.x = 0.5*T*wx;
	q1.y = 0.5*T*wy;
	q1.z = 0.5*T*wz;
	q = NhanQuat(q, q1);
	float norm = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
	q.w /= norm; q.x /= norm; q.y /= norm; q.z /= norm;
}

void goc_Euler_Quat(void){
	  Head   = atan2(2.0f * (q.x * q.y + q.w * q.z),1 - 2.0f * (q.y * q.y + q.z * q.z));   
    Pitch = asin(2.0f * (q.w * q.y - q.x * q.z));
    Roll  = atan2(2.0f * (q.w * q.x + q.y * q.z), 1 - 2.0f * (q.x * q.x + q.y * q.y));
}

void updateV_Quat(void){
	// Note: qNb meaning: V_N = qNb*V_b*(qNb*)
	float g = 9.8f;
	struct Quaternion ab;
	ab.w = 0; ab.x = ax; ab.y = ay; ab.z = az;
	aN = NhanQuat(NhanQuat(q, ab), LienhopQuat(q));
	Vx += T*aN.x*g;
	Vy += T*aN.y*g;
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

void Madgwick(void)
{
	float q1 = q.w, q2 = q.x, q3 = q.y, q4 = q.z;
	float norm;
	float f1, f2, f3;                                         // objective funcyion elements
	float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
	float qDot1, qDot2, qDot3, qDot4;
	float hatDot1, hatDot2, hatDot3, hatDot4;
	float gerrx, gerry, gerrz;  			// gyro bias error
	float a1,a2,a3;

	// Auxiliary variables to avoid repeated arithmetic
	float _halfq1 = 0.5f * q1;
	float _halfq2 = 0.5f * q2;
	float _halfq3 = 0.5f * q3;
	float _halfq4 = 0.5f * q4;
	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;
	float _2q4 = 2.0f * q4;

	// Normalise accelerometer measurement
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0) return; // handle NaN
	a1 = -ax/norm;
	a2 = -ay/norm;
	a3 = -az/norm;
	
	// Compute the objective function and Jacobian
	f1 = _2q2 * q4 - _2q1 * q3 - a1;
	f2 = _2q1 * q2 + _2q3 * q4 - a2;
	f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - a3;
	J_11or24 = _2q3;
	J_12or23 = _2q4;
	J_13or22 = _2q1;
	J_14or21 = _2q2;
	J_32 = 2.0f * J_14or21;
	J_33 = 2.0f * J_11or24;

	// Compute the gradient (matrix multiplication)
	hatDot1 = J_14or21 * f2 - J_11or24 * f1;
	hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
	hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
	hatDot4 = J_14or21 * f1 + J_11or24 * f2;
	
	// Normalize the gradient
	norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
	hatDot1 /= norm;
	hatDot2 /= norm;
	hatDot3 /= norm;
	hatDot4 /= norm;
	
	// Compute estimated gyroscope biases
	gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
	gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
	gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;
	
	// Compute and remove gyroscope biases
	gbiasx += gerrx * T * zeta;
	gbiasy += gerry * T * zeta;
	gbiasz += gerrz * T * zeta;
	wx -= gbiasx;
	wy -= gbiasy;
	wz -= gbiasz;
	
	// Compute the quaternion derivative
	qDot1 = -_halfq2 * wx - _halfq3 * wy - _halfq4 * wz;
	qDot2 =  _halfq1 * wx + _halfq3 * wz - _halfq4 * wy;
	qDot3 =  _halfq1 * wy - _halfq2 * wz + _halfq4 * wx;
	qDot4 =  _halfq1 * wz + _halfq2 * wy - _halfq3 * wx;

	// Compute then integrate estimated quaternion derivative
	q1 += (qDot1 -(beta * hatDot1)) * T;
	q2 += (qDot2 -(beta * hatDot2)) * T;
	q3 += (qDot3 -(beta * hatDot3)) * T;
	q4 += (qDot4 -(beta * hatDot4)) * T;

	// Normalize the quaternion
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
	q.w = q1 / norm;
	q.x = q2 / norm;
	q.y = q3 / norm;
	q.z = q4 / norm;	
}
        
void setW(float newWx, float newWy, float newWz) { wx = newWx; wy = newWy; wz = newWz; }

void seta(float newax, float neway, float newaz) { ax = newax; ay = neway; az = newaz; }



