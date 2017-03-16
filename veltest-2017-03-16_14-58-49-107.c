#include <stdio.h>
#include <mraa/i2c.h>
#include "LSM9DS0.h"
#include "MadgwickAHRS.h"
#include <math.h>
#include <curl/curl.h>
#include <unistd.h>
#include <mraa/gpio.h>
#include "vector.h"

#define degToRad 3.14159265359/180.f //would be faster as a constant
#define microSeconds  40000 //0.1s or 10Hz

static volatile int run_flag = 1;

struct Angle
{
	double x;
	double y;
	double z;
}Omega,current_data;

typedef struct Quaternion
{
	float w;
	float x;
	float y;
	float z;
} Quat;

struct accel
{
	float x;
	float y;
	float z;
} acc, acc_av;

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

int top_pressed = 0;

mraa_gpio_context top, bot;

void rising_top()
{
	usleep(40);
	if ( !mraa_gpio_read(top) ) {
		printf("top release\n");
		top_pressed = 0;
	}
	else
	{
		printf("top press\n");
		top_pressed = 1;
	}
	run_flag = 0;
}

void falling_top()
{
	usleep(40);
	if ( mraa_gpio_read(top) )
		return;
	printf("top release\n");
}

void rising_bot()
{
	usleep(40);
	if ( !mraa_gpio_read(bot) )
		return;
	printf("bottom press\n");
	run_flag = 0;
}

void falling_bot()
{
	usleep(40);
	if (mraa_gpio_read(top) )
		return;
	printf("bottom release\n");
}

void fillQuat(Quat* quat)
{
	quat->w = q0;
	quat->x = q1;
	quat->y = q2;
	quat->z = q3;
}

void quatinv(const Quat* const quat, Quat* quat_inv)
{
	quat_inv->w = quat->w;
	quat_inv->x = -quat->x;
	quat_inv->y = -quat->y;
	quat_inv->z = -quat->z;
}

void convertToEuler(const Quat* const q, struct Angle* omega)
{
	//convert the quaternion representation to Euler Angles in radians
	Omega.x = atan2f(q->w*q->x + q->y*q->z, 0.5f - q->x*q->x - q->y*q->y);
	Omega.y = asinf(-2.0f * (q->x*q->z - q->w*q->y));
	Omega.z = atan2f(q->x*q->y + q->w*q->z, 0.5f - q->y*q->y - q->z*q->z); 

	//convert Angles from  radians to degrees
	Omega.x *= 180 / 3.14159265359;
	Omega.y *= 180 / 3.14159265359;
	Omega.z *= 180 / 3.14159265359;
}

void quatrotate(const Quat* const q, struct accel* a) {
	float q01 = q->w * q->x;
	float q02 = q->w * q->y;
	float q03 = q->w * q->z;
	float q11 = q->x * q->x;
	float q12 = q->x * q->y;
	float q13 = q->x * q->z;
	float q22 = q->y * q->y;
	float q23 = q->y * q->z;
	float q33 = q->z * q->z;

	float a_new_x = a->x * (1 - 2*q22 - 2*q33) + a->y * (2*(q12 + q03)) + a->z * 2*(q13 - q02);
	float a_new_y = a->x * 2*(q12 - q03) + a->y * (1 - 2*q11 - 2*q33) + a->z * 2*(q23 + q01);
	float a_new_z = a->x * 2*(q13 + q02) + a->y * 2*(q23 - q01) + a->z * (1 - 2*q11 - 2*q22);

	a->x = a_new_x;
	a->y = a_new_y;
	a->z = a_new_z;
}

int main(int argc, char **argv) {
	int send = 0; 
	if ( argc > 0 )
		send = 1;
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

	top = mraa_gpio_init(31);
	bot = mraa_gpio_init(45);

	mraa_gpio_dir(top, MRAA_GPIO_IN);
	mraa_gpio_dir(bot, MRAA_GPIO_IN);

	mraa_gpio_isr(top, MRAA_GPIO_EDGE_BOTH, &rising_top, NULL);
	//mraa_gpio_isr(bot, MRAA_GPIO_EDGE_RISING, &rising_bot, NULL);
	mraa_gpio_isr(top, MRAA_GPIO_EDGE_FALLING, &falling_top, NULL);
	mraa_gpio_isr(bot, MRAA_GPIO_EDGE_FALLING, &falling_bot, NULL);

	//initialize Omega to zero and prev_data
	Omega.x = 0;
	Omega.y = 0;
	Omega.z = 0;

	// initialize q and q_inv. 
	Quat q; 
	q.w = q0;
	q.x = q1;
	q.y = q2;
	q.z = q3;

	// initialize acc_av. 
	acc_av.x = 0;
	acc_av.y = 0;
	acc_av.z = 0;

	//vector v;
	VECTOR_INIT(v);

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
	
	//printf("x: %f y: %f z: %f\n", gyro_offset.x, gyro_offset.y, gyro_offset.z);
	
	//Read the sensor data and print them.
	printf("You can start motion.\n");
	while(1) {
		if ( top_pressed ) {
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

			fillQuat(&q);
			convertToEuler(&q, &Omega);

			// gravity compensate
			// float grav[] = {0.0, 0.0, 0.0};
			// grav[0] = 2 * (q1 * q3 - q0 * q2);
			// grav[1] = 2 * (q0 * q1 + q2 * q3);
			// grav[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

			// float newaccX = (accel_data.x - grav[0])*9.8;
			// float newaccY = (accel_data.y - grav[1])*9.8;
			// float newaccZ = (accel_data.z - grav[2])*9.8;	
			
				

			//	x_acc_old = 0.5 * newaccX + 0.5 * x_acc_old;
			//	y_acc_old = 0.5 * newaccY + 0.5 * y_acc_old;
			//	z_acc_old = 0.5 * newaccZ + 0.5 * z_acc_old;
			
			Quat q_inv;
			quatinv(&q, &q_inv);

			acc.x = accel_data.x;
			acc.y = accel_data.y;
			acc.z = accel_data.z;
			quatrotate(&q_inv, &acc);

			// gravity subtraction. 
			acc.z = acc.z - 1;

			// filtered acceleration. 
			acc_av.x = 0.5 * acc_av.x + (0.5 * acc.x)*9.8;
			acc_av.y = 0.5 * acc_av.y + (0.5 * acc.y)*9.8;
			acc_av.z = 0.5 * acc_av.z + (0.5 * acc.z)*9.8;

			//define new variable just to see it work
			x_acc_old = acc_av.x;
			y_acc_old = acc_av.y;
			z_acc_old = acc_av.z;

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

			if(countx >= 10) {
				x_motion = 0;
			}

			if(y_acc_old == 0)
				county += 1;
			else
				county = 0;

			if(county >= 10) {
				y_motion = 0;
			}

			if(z_acc_old == 0)
				countz += 1;
			else
				countz = 0;

			if(countz >= 10) {
				z_motion = 0;
			}

			if(countx == 0 && county == 0 && countz == 0)
				motion = 0;
			else
				motion = 1;

			
			//check motion and count accelerations over 1.2 m/s^2
			if(x_acc_old <= 1.5 && x_acc_old >= -1.5) {
				x_acc_old = 0;
			}
			else
				x_counterP++;
			if(x_motion == 0) {
				x_avg = 0;
				x_counterP = 0;
			}
			if(x_counterP != 0) {
				x_avg += x_acc_old / x_counterP / x_counterP;//1
				x_pos += x_avg / 80; //180
			}
		/*	else {
			//	x_avg = 0;
				if (x_avg < 10 && x_avg > -10)
					x_avg = 0;
				x_counterP = 0;
			}*/
			if(y_acc_old <= 1.5 && y_acc_old >= -1.5) {
				y_acc_old = 0;
			}
			else
				y_counterP++;
			if(y_motion == 0) {
				y_avg = 0;
				y_counterP = 0;
			}
			if(y_counterP != 0) {
				y_avg += y_acc_old / y_counterP / y_counterP;
				y_pos += y_avg / 160;
			}
				
		/*	else {
			//	y_avg = 0;
				if (y_avg < 10 && y_avg > -10)
					y_avg = 0;
				y_counterP = 0;
			}*/
		
			if(z_acc_old <= 1.5 && z_acc_old >= -1.5)
				z_acc_old = 0;
			if(z_motion == 0)
				z_avg = 0;
			z_avg += z_acc_old / 20;
			z_pos += z_avg / 30;//280
			
			/*	else {
			//	z_avg = 0;
				if (z_avg < 10 && z_avg > -10)
					z_avg = 0;
				z_counterP = 0;
			}*/
			//printf("X: %f\t Y: %f\t Z: %f\n\n", gyro_data.x - gyro_offset.x, gyro_data.y - gyro_offset.y, gyro_data.z - gyro_offset.z);
			//printf("AccX: %f\t AccY: %f\t AccZ: %f\n\n", accel_data.x, accel_data.y, accel_data.z);
			//printf("OmegaX: %f\n OmegaY: %f\n OmegaZ: %f\n\n", Omega.x,Omega.y,Omega.z);
			//printf("newaccX: %f\t newaccY: %f\t newaccZ: %f\n\n", newaccX, newaccY, newaccZ);
			//printf("x_acc: %f\t y_acc: %f\t z_acc: %f\n\n", x_acc_old, y_acc_old, z_acc_old);
			//printf("av_accX: %f\t av_accY: %f\t av_accZ: %f\t mag_acc: %f\n", av_accX, av_accY, av_accZ, mag_av_acc);

			//printf("%f\t %f\t %f\n", x_pos, y_pos, z_pos);
			char msg[100] = "";
			sprintf(msg, "{\"X\":\"%f\",\"Y\":\"%f\",\"Z\":\"%f\"}", x_pos, z_pos, y_pos); 
			VECTOR_ADD(v, msg);
		}
		else { // button has been released. 
			x_avg = 0;
			y_avg = 0;
			z_avg = 0;
			if ( send ) { 
				while (VECTOR_TOTAL(v) > 0)
				{
					curl_easy_setopt(curl, CURLOPT_POSTFIELDS, VECTOR_GET(v, char*, 0));
					//perform request, res gets return code
					res = curl_easy_perform(curl);
					//check for errors
					if(res != CURLE_OK)
						fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));
					VECTOR_DELETE(v, 0); // pretty inefficient but removes race conditions? 
				}
				// for (i = 0; i < VECTOR_TOTAL(v); i++)
				// {
				// 	VECTOR_DELETE(v, i);
				// }
			}
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



