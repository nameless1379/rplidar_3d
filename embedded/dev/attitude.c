#include "ch.h"
#include "hal.h"

#include "attitude.h"
#include "math_misc.h"

static lpfilterStruct lpAccelX;
static lpfilterStruct lpAccelY;
static lpfilterStruct lpAccelZ;
static lpfilterStruct lpGyroX;
static lpfilterStruct lpGyroY;
static lpfilterStruct lpGyroZ;

static inline void imu_lpfilterapply(PIMUStruct pIMU)
{
  pIMU->accelFiltered[X] = lpfilter_apply(&lpAccelX,pIMU->accelData[X]);
  pIMU->accelFiltered[Y] = lpfilter_apply(&lpAccelY,pIMU->accelData[Y]);
  pIMU->accelFiltered[Z] = lpfilter_apply(&lpAccelZ,pIMU->accelData[Z]);
  pIMU->gyroFiltered[X] = lpfilter_apply(&lpGyroX,pIMU->gyroData[X]);
  pIMU->gyroFiltered[Y] = lpfilter_apply(&lpGyroY,pIMU->gyroData[Y]);
  pIMU->gyroFiltered[Z] = lpfilter_apply(&lpGyroZ,pIMU->gyroData[Z]);
}

uint8_t attitude_update(PIMUStruct pIMU)
{
  uint8_t error = mpu6050GetData(pIMU);
  if(error)
    return error;

  imu_lpfilterapply(pIMU);

  float corr[3] = {0.0f, 0.0f, 0.0f};
  float spinRate = vector_norm(pIMU->gyroFiltered, 3);
  float accel = vector_norm(pIMU->accelFiltered, 3);

  vector_normalize(pIMU->qIMU, 4);
  uint8_t i;

  if(accel < 12.81f && accel > 6.81f)
  {
    float accel_corr[3], norm_accel[3], v2[3];

    v2[0] = 2.0f * (pIMU->qIMU[1] * pIMU->qIMU[3] - pIMU->qIMU[0] * pIMU->qIMU[2]);
    v2[1] = 2.0f * (pIMU->qIMU[2] * pIMU->qIMU[3] + pIMU->qIMU[0] * pIMU->qIMU[1]);
    v2[2] = pIMU->qIMU[0] * pIMU->qIMU[0] -
            pIMU->qIMU[1] * pIMU->qIMU[1] -
            pIMU->qIMU[2] * pIMU->qIMU[2] +
            pIMU->qIMU[3] * pIMU->qIMU[3];

    for (i = 0; i < 3; i++)
      norm_accel[i] = pIMU->accelFiltered[i]/accel;

    vector3_cross(norm_accel, v2, accel_corr);
    for (i = 0; i < 3; i++)
      corr[i] += accel_corr[i] * ATT_W_ACCEL;

    if(spinRate < 0.175f)
      for (i = 0; i < 3; i++)
      {
        pIMU->gyroBias[i] += corr[i] * (ATT_W_GYRO * pIMU->dt);

        if(pIMU->gyroBias[i] > GYRO_BIAS_MAX)
          pIMU->gyroBias[i] = GYRO_BIAS_MAX;
        if(pIMU->gyroBias[i] < -GYRO_BIAS_MAX)
          pIMU->gyroBias[i] = -GYRO_BIAS_MAX;
      }
  }

  for (i = 0; i < 3; i++)
    corr[i] += pIMU->gyroFiltered[i] + pIMU->gyroBias[i];

  float dq[4];
  q_derivative(pIMU->qIMU, corr, dq);

  float q[4] = {pIMU->qIMU[0], pIMU->qIMU[1], pIMU->qIMU[2], pIMU->qIMU[3]};
  for (i = 0; i < 4; i++)
    q[i] += dq[i] * pIMU->dt;
  vector_normalize(q,4);

  if(isfinite(q[0]) && isfinite(q[1]) && isfinite(q[2]) && isfinite(q[3]))
  {
    for (i = 0; i < 4; i++)
      pIMU->qIMU[i] = q[i];

    quarternion2euler(pIMU->qIMU, pIMU->euler_angle);
    return IMU_OK;
  }
  else
    return IMU_CORRUPTED_Q_DATA;
}

#define ATTITUDE_INIT_TIMEOUT_MS 1000U
uint8_t attitude_imu_init(PIMUStruct pIMU, const IMUConfigStruct* const imu_conf)
{
  if(!pIMU->inited)
  {
    uint8_t init_error = mpu6050Init(pIMU, imu_conf);
    if(init_error)
      return init_error;
  }

  lpfilter_init(&lpAccelX, MPU6050_UPDATE_FREQ, 30.0f);
  lpfilter_init(&lpAccelY, MPU6050_UPDATE_FREQ, 30.0f);
  lpfilter_init(&lpAccelZ, MPU6050_UPDATE_FREQ, 30.0f);
  lpfilter_init(&lpGyroX, MPU6050_UPDATE_FREQ, 30.0f);
  lpfilter_init(&lpGyroY, MPU6050_UPDATE_FREQ, 30.0f);
  lpfilter_init(&lpGyroZ, MPU6050_UPDATE_FREQ, 30.0f);

  uint8_t i;
  uint32_t tick = chVTGetSystemTimeX();

  for (i = 0; i < 200U; i++)
  {
    mpu6050GetData(pIMU);
    if(vector_norm(pIMU->gyroData,3) < 0.175f)
      imu_lpfilterapply(pIMU);

    if(chVTGetSystemTimeX() - tick > MS2ST(ATTITUDE_INIT_TIMEOUT_MS))
      return IMU_ATT_TIMEOUT;
    chThdSleepMilliseconds(1);
  }

  float rot_matrix[3][3];

  float norm = vector_norm(pIMU->accelFiltered,3);
  for (i = 0; i < 3; i++)
    rot_matrix[2][i] = pIMU->accelFiltered[i] / norm;

  norm = sqrtf(rot_matrix[2][2]*rot_matrix[2][2] +
    rot_matrix[2][0]*rot_matrix[2][0]);

  rot_matrix[0][0] = rot_matrix[2][2] / norm;
  rot_matrix[0][1] = 0.0f;
  rot_matrix[0][2] = rot_matrix[2][0] / norm;

  vector3_cross(rot_matrix[2], rot_matrix[0], rot_matrix[1]);
  rotm2quarternion(rot_matrix, pIMU->qIMU);

  return IMU_OK;
}
