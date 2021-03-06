#include <stdio.h>
#include <mraa/i2c.h>
#include "LSM9DS0.h"
#include <signal.h>

#define SAMPLE_TIME 50000
#define MILLION 1000000

sig_atomic_t volatile run_flag = 1;

void do_when_interrupted(int sig)
{
	if (sig == SIGINT)
		run_flag = 0;
}

int main() {
	signal(SIGINT, do_when_interrupted);

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

	printf("Start moving!\n");

	FILE* fp;
	fp = fopen("data.txt", "w+");

	//Read the sensor data and print them.
	while(run_flag) {
		accel_data = read_accel(accel, a_res);
		gyro_data = read_gyro(gyro, g_res);

		fprintf("%f,\t%f,\t%f,\t", accel_data.x, accel_data.y, accel_data.z);
		printf("%f,\t%f,\t%f\r\n", gyro_data.x - gyro_offset.x, gyro_data.y - gyro_offset.y, gyro_data.z - gyro_offset.z);
		
		usleep(SAMPLE_TIME);
	}
	fclose(fp);
	return 0;
}
