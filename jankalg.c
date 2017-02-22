#include <stdio.h>
#include <mraa/i2c.h>
#include "LSM9DS0.h"
#include "MadgwickAHRS.h"
#include <math.h>
#include <curl/curl.h>

#define degToRad 3.14159265359/180.f //would be faster as a constant

struct Angle
{
	double x;
	double y;
	double z;
}Omega,current_data;

#define microSeconds  40000 //0.1s or 10Hz

const double time_interval = (double)microSeconds/1000000;

	float x_acc_old = 0;
	float y_acc_old = 0;
	float z_acc_old = 0;

	float x_acc = 0;
	float y_acc = 0;
	float z_acc = 0;

	float x_pos = 0;
	float y_pos = 0;
	float z_pos = 0;
	
	int countx = 0;
	int county = 0;
	int countz = 0;
	
	int x_motion = 1;
	int y_motion = 1;
	int z_motion = 1;

	int motion = 1;

	int x_counterP = 0;
	int y_counterP = 0;
	int z_counterP = 0;
	int x_counterN = 0;
	int y_counterN = 0;
	int z_counterN = 0;

	float x_avg = 0;
	float y_avg = 0;
	float z_avg = 0;

int main(int argc, char **argv) {
	int send = 0; 
	//if ( argc > 0 )
	//	send = 1;
	//curl for firebase
	//
	CURL *curl;
	CURLcode res;

	if (send) {
		curl = curl_easy_init();

		if(curl) {
			curl_easy_setopt(curl, CURLOPT_URL, "https://tilted-4d2ee.firebaseio.com/points.json");
		}
	}

	//variables
	data_t accel_data, gyro_data, mag_data;
	data_t gyro_offset;
	int16_t temperature;
	float a_res, g_res, m_res;
	mraa_i2c_context accel, gyro, mag;
	accel_scale_t a_scale = A_SCALE_4G;
	gyro_scale_t g_scale = G_SCALE_2000DPS;
	mag_scale_t m_scale = M_SCALE_2GS;

	//initialize Omega to zero and prev_data
	Omega.x = 0;
	Omega.y = 0;
	Omega.z = 0;

	//initialize sensors, set scale, and calculate resolution.
	accel = accel_init();
	set_accel_scale(accel, a_scale);	
	a_res = calc_accel_res(a_scale);
	
	gyro = gyro_init();
	set_gyro_scale(gyro, g_scale);
	g_res = calc_gyro_res(g_scale);
	
	mag = mag_init();
	set_mag_scale(mag, m_scale);
	m_res = calc_mag_res(m_scale);

	//find offset for the gyro sensor 
	gyro_offset = calc_gyro_offset(gyro, g_res);
	
	printf("x: %f y: %f z: %f\n", gyro_offset.x, gyro_offset.y, gyro_offset.z);
	
	//Read the sensor data and print them.
	while(1) {

		accel_data = read_accel(accel, a_res);
		gyro_data = read_gyro(gyro, g_res);

		//calculates the angular rate of change in degrees per second
		current_data.x = gyro_data.x-gyro_offset.x;
		current_data.y = gyro_data.y-gyro_offset.y;
		current_data.z = gyro_data.z-gyro_offset.z;
		
		//convert angular rate of change to radians per second
		current_data.x *= degToRad;
		current_data.y *= degToRad;
		current_data.z *= degToRad;

		//perform the Madgwick Algorithm
		//with the Madgwick algorithm, in MadgwickAHRS.c
		//you will need to change the variable sampleFreq to the frequency that you are reading data in at
		//this is currently set to 10Hz
		MadgwickAHRSupdateIMU(current_data.x,current_data.y,current_data.z,accel_data.x,accel_data.y,accel_data.z);

		//convert the quaternion representation to Euler Angles in radians
		Omega.x = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
		Omega.y = asinf(-2.0f * (q1*q3 - q0*q2));
		Omega.z = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3); 

		//convert Angles from  radians to degrees
		Omega.x *= 180 / 3.14159265359;
		Omega.y *= 180 / 3.14159265359;
		Omega.z *= 180 / 3.14159265359;
		

		//gravity compensate
		float grav[] = {0.0, 0.0, 0.0};
		grav[0] = 2 * (q1 * q3 - q0 * q2);
		grav[1] = 2 * (q0 * q1 + q2 * q3);
		grav[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

		float newaccX = (accel_data.x - grav[0])*9.8;
		float newaccY = (accel_data.y - grav[1])*9.8;
		float newaccZ = (accel_data.z - grav[2])*9.8;	
		
		x_acc_old = 0.5 * newaccX + 0.5 * x_acc_old;
		y_acc_old = 0.5 * newaccY + 0.5 * y_acc_old;
		z_acc_old = 0.5 * newaccZ + 0.5 * z_acc_old;

		//double integration for position
		if(x_acc_old <= 0.6 && x_acc_old >= -0.6)
			x_acc_old = 0;
		if(y_acc_old <= 0.6 && y_acc_old >= -0.6)
			y_acc_old = 0;
		if(z_acc_old <= 0.6 && z_acc_old >= -0.6)
			z_acc_old = 0;
		
	//movement_end_check
	
	x_motion = 1;
	y_motion = 1;
	z_motion = 1;

	if(x_acc_old == 0)
		countx += 1;
	else
		countx = 0;

	if(countx >= 20) {
		x_motion = 0;
	}

	if(y_acc_old == 0)
		county += 1;
	else
                county = 0;

	if(county >= 20) {
		y_motion = 0;
	}

	if(z_acc_old == 0)
                countz += 1;
	else
		countz = 0;

	if(countz >= 20) {
		z_motion = 0;
	}

	if(countx == 0 && county == 0 && countz == 0)
		motion = 0;
	else
		motion = 1;

	
	//check motion and count accelerations over 1.2 m/s^2
	if((x_acc_old > 1.2 || x_acc_old < -1.2) && x_motion == 1) {
		x_counterP++;
		x_avg += x_acc_old / (x_counterP * x_counterP);
		x_pos += x_avg / 70;
	}
	else {
		x_avg = 0;
		x_counterP = 0;
	}
	if((y_acc_old > 1.2 || y_acc_old < -1.2) && y_motion == 1) {
		y_counterP++;
		y_avg += y_acc_old / (y_counterP * y_counterP);
		y_pos += y_avg / 70;
	}	
	else {
		y_avg = 0;
		y_counterP = 0;
	}
	if((z_acc_old > 1.2 || z_acc_old < -1.2) && z_motion == 1) {
		z_counterP++;
		z_avg += z_acc_old / (z_counterP * z_counterP);
		z_pos += z_avg / 70;
	}
	else {
		z_avg = 0;
		z_counterP = 0;
	}
		//printf("X: %f\t Y: %f\t Z: %f\n\n", gyro_data.x - gyro_offset.x, gyro_data.y - gyro_offset.y, gyro_data.z - gyro_offset.z);
		printf("AccX: %f\t AccY: %f\t AccZ: %f\n\n", accel_data.x, accel_data.y, accel_data.z);
		//printf("OmegaX: %f\n OmegaY: %f\n OmegaZ: %f\n\n", Omega.x,Omega.y,Omega.z);
		//printf("newaccX: %f\t newaccY: %f\t newaccZ: %f\n\n", newaccX, newaccY, newaccZ);
		printf("x_acc: %f\t y_acc: %f\t z_acc: %f\n\n", x_acc_old, y_acc_old, z_acc_old);
		//printf("av_accX: %f\t av_accY: %f\t av_accZ: %f\t mag_acc: %f\n", av_accX, av_accY, av_accZ, mag_av_acc);

		printf("x_pos: %f\t y_pos: %f\t z_pos: %f\n\n", x_pos, y_pos, z_pos);

	if ( send ) {
		//curl send message
		char msg[100] = "";
		sprintf(msg, "{\"X\":\"%f\",\"Y\":\"%f\",\"Z\":\"%f\"}", x_pos, z_pos, y_pos); 
		printf("%s\n", msg);
		curl_easy_setopt(curl, CURLOPT_POSTFIELDS, msg);

		//perform request, res gets return code
		res = curl_easy_perform(curl);
		//check for errors
		if(res != CURLE_OK)
			fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));
	}
	

		usleep(microSeconds);
	}

	//curl cleanup
	if (send) {
		curl_easy_cleanup(curl);
		curl_global_cleanup();
	}	
	return 0;	
}



