#include <stdio.h>
#include <mraa/i2c.h>
#include "LSM9DS0.h"

#include <math.h>

//#define ACCELEROMETER_SENSITIVITY 8192.0
//#define GYROSCOPE_SENSITIVITY 65.536

#define ACCELEROMETER_SENSITIVITY 0.125
#define GYROSCOPE_SENSITIVITY 0.125

#define dt 0.01

#define SAMPLE_TIME 50000

void ComplementaryFilter(float accData[3], float gyrData[3], float *pitch, float *roll, float *yaw);

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

	accel_data = read_accel(accel, a_res);
	gyro_data = read_gyro(gyro, g_res);

    float accData[3] = {accel_data.x, accel_data.y, accel_data.z};
    float gyrData[3] = {gyro_data.x - gyro_offset.x, gyro_data.y - gyro_offset.y, gyro_data.z - gyro_offset.z};
    float pitch = 0.0f;
    float roll = 0.0f;
    float yaw = 0.0f;
    
    while(1) {
	
	accel_data = read_accel(accel, a_res);
	gyro_data = read_gyro(gyro, g_res);
        ComplementaryFilter(accData, gyrData, &pitch, &roll, &yaw);
        
        accData[0] = accel_data.x;
        accData[1] = accel_data.y;
        accData[2] = accel_data.z;
        
        gyrData[0] = gyro_data.x - gyro_offset.x;
        gyrData[1] = gyro_data.y - gyro_offset.y;
        gyrData[2] = gyro_data.z - gyro_offset.z;
        
        printf("X: %f\t Y: %f\t Z: %f\t||", accel_data.x, accel_data.y, accel_data.z);
        printf("\tX: %f\t Y: %f\t Z: %f\t||", gyro_data.x - gyro_offset.x, gyro_data.y - gyro_offset.y, gyro_data.z - gyro_offset.z);
        printf("\tX: %f\t Y: %f\t Z: %f\n", roll, pitch, yaw);
        
        usleep(SAMPLE_TIME);
    }
    
    return 0;	
}

void ComplementaryFilter(float accData[3], float gyrData[3], float *pitch, float *roll, float *yaw)
{
    float pitchAcc, rollAcc, yawAcc;
    
    // Integrate the gyroscope data -> int(angularSpeed) = angle
    *pitch += ((float)gyrData[0] / GYROSCOPE_SENSITIVITY) * dt; // Angle around the X-axis
    *roll -= ((float)gyrData[1] / GYROSCOPE_SENSITIVITY) * dt;    // Angle around the Y-axis
    *yaw += ((float)gyrData[2] / GYROSCOPE_SENSITIVITY) * dt;

    // Compensate for drift with accelerometer data if !bullshit
    // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
    int forceMagnitudeApprox = abs(accData[0]) + abs(accData[1]) + abs(accData[2]);
    if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
    {
        // Turning around the X axis results in a vector on the Y-axis
        pitchAcc = atan2f((float)accData[1], (float)accData[2]) * 180 / M_PI;
        *pitch = *pitch * 0.98 + pitchAcc * 0.02;
        
        // Turning around the Y axis results in a vector on the X-axis
        rollAcc = atan2f((float)accData[0], (float)accData[2]) * 180 / M_PI;
        *roll = *roll * 0.98 + rollAcc * 0.02;

	yawAcc = atan2f((float)accData[2], (float)accData[2]) * 180 / M_PI;
	*yaw = *yaw * 0.98 + yawAcc * 0.02;
    }
}
