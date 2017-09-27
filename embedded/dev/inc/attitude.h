#ifndef _ATTITUDE_H_
#define _ATTITUDE_H_

#include "mpu6050.h"

#define ATT_W_ACCEL     0.3f
#define ATT_W_GYRO      0.1f
#define GYRO_BIAS_MAX  0.05f

uint8_t attitude_imu_init(PIMUStruct pIMU, const IMUConfigStruct* const imu_conf);
uint8_t attitude_update(PIMUStruct pIMU);

#endif
