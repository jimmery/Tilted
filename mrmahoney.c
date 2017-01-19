#include <stdio.h>
#include <mraa/i2c.h>
#include "LSM9DS0.h"

#include <math.h>
#include “MahoneyAHRS.h”

//--------- Original Definitions ---------//
#define SAMPLE_TIME 50000
#define MILLION 1000000


//--------- Mahoney Definitions ---------//

#define sampleFreq	512.0f		// sample frequency in Hz
#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain


//--------- Mahoney Variables ---------//

volatile float twoKp = twoKpDef;	// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;	// 2 * integral gain (Ki)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki


//--------- Mahoney Functions ---------//

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
float invSqrt(float x);


//--------- Main ---------//

int main() {
	data_t accel_data, gyro_data;
	data_t gyro_offset;
	float a_res, g_res;
	float x_ang, y_ang, z_ang;
	mraa_i2c_context accel, gyro;
	accel_scale_t a_scale = A_SCALE_4G;
	gyro_scale_t g_scale = G_SCALE_245DPS;

	//initialize sensors, set scale, and calculate resolution.
	accel = accel_init();
	set_accel_scale(accel, a_scale);	
	a_res = calc_accel_res(a_scale);
	
	gyro = gyro_init();
	set_gyro_scale(gyro, g_scale);
	g_res = calc_gyro_res(g_scale);

	gyro_offset = calc_gyro_offset(gyro, g_res);
	
	printf("x: %f y: %f z: %f\n", gyro_offset.x, gyro_offset.y, gyro_offset.z);

	printf("\n\t\tAccelerometer\t\t\t||");
	printf("\t\t\tGyroscope\t\t\t||");
	
	x_ang = 0;
	y_ang = 0;
	z_ang = 0;

	//Read the sensor data and print them.
//	while(1) {
//		accel_data = read_accel(accel, a_res);
//		gyro_data = read_gyro(gyro, g_res);
//
//		x_ang += (gyro_data.x - gyro_offset.x) * SAMPLE_TIME / MILLION;
//		y_ang += (gyro_data.y - gyro_offset.y) * SAMPLE_TIME / MILLION;
//		z_ang += (gyro_data.z - gyro_offset.z) * SAMPLE_TIME / MILLION;
//
//  		printf("X: %f\t Y: %f\t Z: %f\t||", accel_data.x, accel_data.y, accel_data.z);
//  		printf("\tX: %f\t Y: %f\t Z: %f\t||", gyro_data.x - gyro_offset.x, gyro_data.y - gyro_offset.y, gyro_data.z - gyro_offset.z);
//		printf("\tX: %f\t Y: %f\t Z: %f\n", x_ang, y_ang, z_ang); 
//		usleep(SAMPLE_TIME);
//	}
    
    while(1) {
        MahonyAHRSupdateIMU(gyro_data.x - gyro_offset.x, gyro_data.y - gyro_offset.y, gyro_data.z - gyro_offset.z, accel_data.x, accel_data.y, accel_data.z);
        
        float roll, pitch, yaw;
        Quat2Euler(q0, q1, q2, q3, roll, pitch, yaw);
        printf("x: %f\t y: %f\t z: %f\n", roll, pitch, yaw);
        usleep(SAMPLE_TIME);
    }

	
    


	return 0;	
}

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

void Quat2Euler(const float& q0, const float& q1, const float& q2, const float& q3, 
		float& x, float& y, float& z)
{
	float ysqr = q2 * q2;

	// roll (x-axis rotation)
	double t0 = +2.0f * (q0 * q1 + q2 * q3);
	double t1 = +1.0f - 2.0f * (q1 * q1 + ysqr);
	x = atan2(t0, t1);

	// pitch (y-axis rotation)
	double t2 = +2.0f * (q0 * q2 - q1 * q3);
	t2 = t2 > 1.0f ? 1.0f : t2;
	t2 = t2 < -1.0f ? -1.0f : t2;
	y = asin(t2);

	// yaw (z-axis rotation)
	double t3 = +2.0f * (q0 * q3 + q1 * q2);
	double t4 = +1.0f - 2.0f * (ysqr + q3 * q3);  
	z = atan2(t3, t4);
}

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}