#include <stdio.h>
#include <mraa/i2c.h>
#include "LSM9DS0.h"

#define SAMPLE_TIME 5000
#define MILLION 1000000

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

	float axf = 0;
	float ayf = 0;
	float azf = 0;

	//Read the sensor data and print them.
	while(1) {
		accel_data = read_accel(accel, a_res);
		gyro_data = read_gyro(gyro, g_res);

		x_ang += (gyro_data.x - gyro_offset.x) * SAMPLE_TIME / MILLION;
		y_ang += (gyro_data.y - gyro_offset.y) * SAMPLE_TIME / MILLION;
		z_ang += (gyro_data.z - gyro_offset.z) * SAMPLE_TIME / MILLION;

		axf = 0.5*axf + 0.5*accel_data.x;
		ayf = 0.5*ayf + 0.5*accel_data.y;
		azf = 0.5*azf + 0.5*accel_data.z;

  		printf("%f\t %f\t %f\n", axf, ayf, azf);
  	//	printf("\tX: %f\t Y: %f\t Z: %f\t||", gyro_data.x - gyro_offset.x, gyro_data.y - gyro_offset.y, gyro_data.z - gyro_offset.z);
	//	printf("\tX: %f\t Y: %f\t Z: %f\n", x_ang, y_ang, z_ang); 
		usleep(SAMPLE_TIME);
	}
    
	return 0;	
}
