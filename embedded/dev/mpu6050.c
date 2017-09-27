/**
 * This is device realize "read through write" paradigm. This is not
 * standard, but most of I2C devices use this paradigm.
 * You must write to device reading address, send restart to bus,
 * and then begin reading process.
 */

#include "ch.h"
#include "hal.h"

//#include "telemetry.h"
#include "mpu6050.h"
#include "flash.h"
#include "math_misc.h"

/* C libraries: */
#include <string.h>

#define MPU6050_RX_BUF_SIZE       0x0E
#define MPU6050_TX_BUF_SIZE       0x05

/* MPU6050 useful registers */
#define MPU6050_SMPLRT_DIV        0x19
#define MPU6050_CONFIG            0x1A
#define MPU6050_GYRO_CONFIG       0x1B
#define MPU6050_ACCEL_CONFIG      0x1C
#define MPU6050_ACCEL_XOUT_H      0x3B
#define MPU6050_ACCEL_XOUT_L      0x3C
#define MPU6050_ACCEL_YOUT_H      0x3D
#define MPU6050_ACCEL_YOUT_L      0x3E
#define MPU6050_ACCEL_ZOUT_H      0x3F
#define MPU6050_ACCEL_ZOUT_L      0x40
#define MPU6050_TEMP_OUT_H        0x41
#define MPU6050_TEMP_OUT_L        0x42
#define MPU6050_GYRO_XOUT_H       0x43
#define MPU6050_GYRO_XOUT_L       0x44
#define MPU6050_GYRO_YOUT_H       0x45
#define MPU6050_GYRO_YOUT_L       0x46
#define MPU6050_GYRO_ZOUT_H       0x47
#define MPU6050_GYRO_ZOUT_L       0x48
#define MPU6050_PWR_MGMT_1        0x6B

/* I2C read transaction time-out in milliseconds. */
#define MPU6050_READ_TIMEOUT_MS   0x01
/* I2C write transaction time-out in milliseconds. */
#define MPU6050_WRITE_TIMEOUT_MS  0x01

static const I2CConfig i2cfg = {
    OPMODE_I2C,
    200000,
    FAST_DUTY_CYCLE_2
};

/* IMU data structure. */
IMUStruct g_IMU1;

/* I2C error info structure. */
I2CErrorStruct g_i2cErrorInfo = {0, 0, 0};

PIMUStruct mpu6050_get(void)
{
  return  &g_IMU1;
}

/**
 * Local variables
 */
/* Data buffers */
static int16_t mpu6050Data[6];

static uint8_t mpu6050RXData[MPU6050_RX_BUF_SIZE];
static uint8_t mpu6050TXData[MPU6050_TX_BUF_SIZE];

I2CErrorStruct* mpuGetError(void)
{
  return &g_i2cErrorInfo;
}

/**
 * @brief  Initialization function of IMU data structure.
 * @param  pIMU - pointer to IMU data structure;
 * @param  fAddrLow - IMU address pin A0 is pulled low flag.
 */
static void imuStructureInit(PIMUStruct pIMU, const IMUConfigStruct* const imu_conf)
{
  /* Initialize to zero. */
  memset((void *)pIMU, 0, sizeof(IMUStruct));
  pIMU->imu_Thd = NULL;

  pIMU->mpu_i2c = imu_conf->mpu_i2c;
  pIMU->addr = imu_conf->a0_high;

  pIMU->accelT[0][0] = 1.0f;
  pIMU->accelT[1][1] = 1.0f;
  pIMU->accelT[2][2] = 1.0f;
  pIMU->accelT[0][1] = 0.0f;
  pIMU->accelT[1][2] = 0.0f;
  pIMU->accelT[2][0] = 0.0f;
  pIMU->accelT[0][2] = 0.0f;
  pIMU->accelT[1][0] = 0.0f;
  pIMU->accelT[2][1] = 0.0f;

  pIMU->accelBias[0] = 0.0f;
  pIMU->accelBias[1] = 0.0f;
  pIMU->accelBias[2] = 0.0f;

  float flash_test;
  flashRead(IMU_CAL_FLASH, &flash_test, 4);
  if(isfinite(flash_test))
    flashRead(IMU_CAL_FLASH, (char*)(pIMU->accelBias), 60);

  switch(imu_conf->gyroConf)
  {
    case MPU6050_GYRO_SCALE_250:
      pIMU->gyro_psc = (1.0f / 131.0f) * M_PI/180.0f;
      break;
    case MPU6050_GYRO_SCALE_500:
      pIMU->gyro_psc = (1.0f /  65.5f) * M_PI/180.0f;
      break;
    case MPU6050_GYRO_SCALE_1000:
      pIMU->gyro_psc = (1.0f /  32.8f) * M_PI/180.0f;
      break;
    case MPU6050_GYRO_SCALE_2000:
      pIMU->gyro_psc = (1.0f /  16.4f) * M_PI/180.0f;
      break;
  }

  switch(imu_conf->accelConf)
  {
    case MPU6050_ACCEL_SCALE_2G:
      pIMU->accel_psc = (GRAV / 16384.0f);
      break;
    case MPU6050_ACCEL_SCALE_4G:
      pIMU->accel_psc = (GRAV /  8192.0f);
      break;
    case MPU6050_ACCEL_SCALE_8G:
      pIMU->accel_psc = (GRAV /  4096.0f);
      break;
    case MPU6050_ACCEL_SCALE_16G:
      pIMU->accel_psc = (GRAV /  2048.0f);
      break;
  }
}

static void trans_accel_offset(PIMUStruct pIMU, float accelData[3])
{
  float accelData_temp[3];

  accelData_temp[X] = accelData[X] - pIMU->accelBias[X];
  accelData_temp[Y] = accelData[Y] - pIMU->accelBias[Y];
  accelData_temp[Z] = accelData[Z] - pIMU->accelBias[Z];

  matrix33_multiply_vector3(pIMU->accelT, accelData_temp, pIMU->accelData);
}

/**
 * @brief  Reads new data from the sensor
 * @param  pIMU - pointer to IMU data structure;
 * @return 1 - if reading was successful;
 *         0 - if reading failed.
 */
uint8_t mpu6050GetData(PIMUStruct pIMU)
{
  uint32_t tcurr = chVTGetSystemTimeX();
  pIMU->dt = ST2US(tcurr - pIMU->tprev)/1000000.0f;
  pIMU->tprev = tcurr;

  float accelData[3];
  uint8_t error =  mpu6050GetDataRaw(pIMU, accelData, pIMU->gyroData);

  if(!error)
    trans_accel_offset(pIMU, accelData);

  return error;
}

/**
 * @brief  Reads new data from the sensor
 * @param  pIMU - pointer to IMU data structure;
 * @return 1 - if reading was successful;
 *         0 - if reading failed.
 */
uint8_t mpu6050GetDataRaw(PIMUStruct pIMU, float AccelRaw[3], float GyroRaw[3])
{
  msg_t status = MSG_OK;

  /* Set the start register address for bulk data transfer. */
  mpu6050TXData[0] = MPU6050_ACCEL_XOUT_H;
  i2cAcquireBus(pIMU->mpu_i2c);
  status = i2cMasterTransmitTimeout(pIMU->mpu_i2c, pIMU->addr, mpu6050TXData, 1,
    mpu6050RXData, 14, MS2ST(MPU6050_READ_TIMEOUT_MS));
	i2cReleaseBus(pIMU->mpu_i2c);

  if (status != MSG_OK) {
    g_i2cErrorInfo.last_i2c_error = i2cGetErrors(pIMU->mpu_i2c);
    if (g_i2cErrorInfo.last_i2c_error) {
      g_i2cErrorInfo.i2c_error_counter++;
    }
    return IMU_I2C_ERROR1;
  }

  mpu6050Data[0] = (int16_t)((mpu6050RXData[ 0]<<8) | mpu6050RXData[ 1]); /* Accel X */
  mpu6050Data[1] = (int16_t)((mpu6050RXData[ 2]<<8) | mpu6050RXData[ 3]); /* Accel Y */
  mpu6050Data[2] = (int16_t)((mpu6050RXData[ 4]<<8) | mpu6050RXData[ 5]); /* Accel Z */
  mpu6050Data[3] = (int16_t)((mpu6050RXData[ 8]<<8) | mpu6050RXData[ 9]); /* Gyro X  */
  mpu6050Data[4] = (int16_t)((mpu6050RXData[10]<<8) | mpu6050RXData[11]); /* Gyro Y  */
  mpu6050Data[5] = (int16_t)((mpu6050RXData[12]<<8) | mpu6050RXData[13]); /* Gyro Z  */

  /* X: */
  AccelRaw[X] = (float)mpu6050Data[0] * pIMU->accel_psc;
  GyroRaw[X]  = (float)mpu6050Data[3] * pIMU->gyro_psc;

  /* Y: */
  AccelRaw[Y] = (float)mpu6050Data[1] * pIMU->accel_psc;
  GyroRaw[Y]  = (float)mpu6050Data[4] * pIMU->gyro_psc;

  /* Z: */
  AccelRaw[Z] = (float)mpu6050Data[2] * pIMU->accel_psc;
  GyroRaw[Z]  = (float)mpu6050Data[5] * pIMU->gyro_psc;

  return IMU_OK;
}

/**
 * @brief  Initialization function for the MPU6050 sensor.
 * @param  addr - I2C address of MPU6050 chip.
 * @return 1 - if initialization was successful;
 *         0 - if initialization failed.
 */
uint8_t mpu6050Init(PIMUStruct pIMU, const IMUConfigStruct* const imu_conf)
{
  msg_t status = MSG_OK;

  imuStructureInit(pIMU, imu_conf);

  i2cStart(pIMU->mpu_i2c, &i2cfg);

  /* Reset all MPU6050 registers to their default values */
  mpu6050TXData[0] = MPU6050_PWR_MGMT_1;  // Start register address;
  mpu6050TXData[1] = 0xC0;          // Register value 0b11000000

  i2cAcquireBus(pIMU->mpu_i2c);

  status = i2cMasterTransmitTimeout(pIMU->mpu_i2c, pIMU->addr, mpu6050TXData, 2,
    NULL, 0, MS2ST(MPU6050_WRITE_TIMEOUT_MS));

  if (status != MSG_OK) {
    i2cReleaseBus(pIMU->mpu_i2c);
    g_i2cErrorInfo.last_i2c_error = i2cGetErrors(pIMU->mpu_i2c);
    if (g_i2cErrorInfo.last_i2c_error) {
      g_i2cErrorInfo.i2c_error_counter++;
    }
    g_i2cErrorInfo.errorFlag |= 0x02;
    return IMU_I2C_ERROR1;
  }

  /* Wait 100 ms for the MPU6050 to reset */
  chThdSleepMilliseconds(100);

  /* Clear the SLEEP flag, set the clock and start measuring. */
  mpu6050TXData[0] = MPU6050_PWR_MGMT_1;  // Start register address;
  mpu6050TXData[1] = 0x03;         // Register value CLKSEL = PLL_Z;

  status = i2cMasterTransmitTimeout(pIMU->mpu_i2c, pIMU->addr, mpu6050TXData, 2,
    NULL, 0, MS2ST(MPU6050_WRITE_TIMEOUT_MS));

  if (status != MSG_OK) {
    i2cReleaseBus(pIMU->mpu_i2c);
    g_i2cErrorInfo.last_i2c_error = i2cGetErrors(pIMU->mpu_i2c);
    if (g_i2cErrorInfo.last_i2c_error) {
      g_i2cErrorInfo.i2c_error_counter++;
//      debugLog("E:mpu6050i-rst");
    }
    g_i2cErrorInfo.errorFlag |= 0x04;
    return IMU_I2C_ERROR2;
  }

  /* Configure the MPU6050 sensor        */
  /* NOTE:                               */
  /* - SLEEP flag must be cleared before */
  /*   configuring the sensor.           */
  mpu6050TXData[0] = MPU6050_SMPLRT_DIV;  // Start register address;
  mpu6050TXData[1] = 11;                  // SMPLRT_DIV register value (8000 / (11 + 1) = 666 Hz);
  mpu6050TXData[2] = 0x00;          // CONFIG register value DLPF_CFG = 0 (256-260 Hz);
  mpu6050TXData[3] = (uint8_t)(imu_conf->gyroConf << 3U);          // GYRO_CONFIG register value
  mpu6050TXData[4] = (uint8_t)(imu_conf->accelConf << 3U);          // ACCEL_CONFIG register value
  status = i2cMasterTransmitTimeout(pIMU->mpu_i2c, pIMU->addr, mpu6050TXData, 5,
    NULL, 0, MS2ST(MPU6050_WRITE_TIMEOUT_MS));

  i2cReleaseBus(pIMU->mpu_i2c);

  if (status != MSG_OK) {
    g_i2cErrorInfo.last_i2c_error = i2cGetErrors(pIMU->mpu_i2c);
    if (g_i2cErrorInfo.last_i2c_error) {
      g_i2cErrorInfo.i2c_error_counter++;
     // debugLog("E:mpu6050i-cfg");
    }
    g_i2cErrorInfo.errorFlag |= 0x08;
    return IMU_I2C_ERROR3;
  }

  pIMU->tprev = chVTGetSystemTimeX();
  pIMU->inited = 1;
  return IMU_OK;
}
