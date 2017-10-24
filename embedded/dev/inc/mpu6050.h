#ifndef _MPU6050_H_
#define _MPU6050_H_

#define MPU6050_UPDATE_FREQ                          400.0f
#define IMU_CAL_FLASH                            0x08010000

typedef enum {
  X = 0U,
  Y = 1U,
  Z = 2U
} mpu_axis_mask_t;

typedef enum {
  Roll = 0U,
  Pitch = 1U,
  Yaw = 2U
} mpu_euler_angle_t;

typedef enum {
  MPU6050_I2C_ADDR_A0_LOW = 0x68,
  MPU6050_I2C_ADDR_A0_HIGH = 0x69,
} mpu_i2c_addr_t;

typedef enum {
  MPU6050_GYRO_SCALE_250 = 0,
  MPU6050_GYRO_SCALE_500 = 1,
  MPU6050_GYRO_SCALE_1000 = 2,
  MPU6050_GYRO_SCALE_2000 = 3
} mpu_gyro_scale_t;

typedef enum {
  MPU6050_ACCEL_SCALE_2G = 0,
  MPU6050_ACCEL_SCALE_4G = 1,
  MPU6050_ACCEL_SCALE_8G = 2,
  MPU6050_ACCEL_SCALE_16G = 3
} mpu_accel_scale_t;

typedef enum {
  IMU_OK = 0,
  IMU_I2C_ERROR1 = 1,
  IMU_I2C_ERROR2 = 2,
  IMU_I2C_ERROR3 = 3,
  IMU_ATT_TIMEOUT = 4,
  IMU_CORRUPTED_Q_DATA = 5,
} imu_att_error_t;

#define MPU6050_UPDATE_PERIOD     1000000.0f/MPU6050_UPDATE_FREQ

typedef struct tagIMUStruct {
  float accelData[3];     /* Accelerometer data.             */
  float gyroData[3];      /* Gyroscope data.                 */
  float accelFiltered[3]; /* Filtered accelerometer data.    */
  float gyroFiltered[3];  /* Filtered gyro data.    */

  float qIMU[4];          /* Attitude quaternion of the IMU. */
  float euler_angle[3];   /* Euler angle of the IMU. */
  float d_euler_angle[3]; /* Euler angle changing rate of the IMU. */

  float accelBias[3];     /* Accelerometer bias.             */
  float accelT[3][3];     /* Accelerometer rotational bias matrix       */
  float gyroBias[3];      /* Gyroscope bias.                 */

  I2CDriver* mpu_i2c;
  uint8_t addr;
  float accel_psc;
  float gyro_psc;

  uint8_t inited;
  uint8_t data_invalid;
  uint16_t invalid_data_counter;
  uint32_t tprev;
  float dt;
  thread_reference_t imu_Thd;

  uint8_t accelerometer_not_calibrated;
  uint8_t gyroscope_not_calibrated;
} __attribute__((packed)) IMUStruct, *PIMUStruct;

typedef struct {
  I2CDriver* mpu_i2c;
  const mpu_i2c_addr_t a0_high;
  const mpu_accel_scale_t accelConf;
  const mpu_gyro_scale_t gyroConf;
} IMUConfigStruct;

typedef struct {
  int8_t last_i2c_error;
  uint16_t i2c_error_counter;
  uint8_t errorFlag;
} I2CErrorStruct;

#ifdef __cplusplus
extern "C" {
#endif
  PIMUStruct mpu6050_get(void);
  I2CErrorStruct* mpuGetError(void);

  uint8_t mpu6050Init(PIMUStruct pIMU, const IMUConfigStruct* const imu_conf);
  uint8_t mpu6050GetDataRaw(PIMUStruct pIMU, float AccelRaw[3], float GyroRaw[3]);
  uint8_t mpu6050GetData(PIMUStruct pIMU);

#ifdef __cplusplus
#endif

#endif /* _MPU6050_H_ */
