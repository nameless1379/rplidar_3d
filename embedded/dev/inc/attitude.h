#ifndef _ATTITUDE_H_
#define _ATTITUDE_H_

#include "mpu6500.h"

#define ATT_W_ACCEL     0.6f
#define ATT_W_GYRO      0.1f
#define GYRO_BIAS_MAX   0.005f

uint8_t attitude_imu_init(PIMUStruct pIMU);
uint8_t attitude_update(PIMUStruct pIMU);
void    attitude_resetYaw(PIMUStruct pIMU, const float yaw);
void attitude_updateGyroZ(PIMUStruct pIMU, const float gyro_corr_z, const float gyro_Bias_z);

#endif
