#include "main.h"
#include "shell.h"

static THD_WORKING_AREA(Shell_thread_wa, 1024);

static void cmd_print(BaseSequentialStream * chp, int argc, char *argv[]){
  (void) argc,argv;
  PIMUStruct pIMU = mpu6050_get();

  chprintf(chp,"Pitch: %f\r\n", pIMU->euler_angle[Pitch] * 180.0f/M_PI);
  chprintf(chp,"Roll: %f\r\n", pIMU->euler_angle[Roll]* 180.0f/M_PI);
  chprintf(chp,"Yaw: %f\r\n", pIMU->euler_angle[Yaw] * 180.0f/M_PI);
}

static void cmd_app1(BaseSequentialStream * chp, int argc, char *argv[]){
  (void) argc,argv;

  chprintf(chp,"app1\r\n");
}

static void cmd_calibration(BaseSequentialStream * chp, int argc, char *argv[])
{
  PIMUStruct pIMU = mpu6050_get();

  pIMU->gyroscope_not_calibrated = true;
  pIMU->accelerometer_not_calibrated = true;
  chThdSleepMilliseconds(10);

  calibrate_gyroscope(pIMU);
  calibrate_accelerometer(pIMU);

  flashSectorErase(flashSectorAt(IMU_CAL_FLASH));
  flashWrite(IMU_CAL_FLASH, (char*)(pIMU->accelBias), 60);

  chThdResume(&(pIMU->imu_Thd), MSG_OK);
}

static const ShellCommand commands[] =
{
  {"print",cmd_print},
  {"cal",cmd_calibration},
  {"app1",cmd_app1}
};

static const ShellConfig shell_cfg1 =
{
  (BaseSequentialStream*)SERIAL_CMD,
  commands
};

void shellStart(void)
{
  sdStart(SERIAL_CMD, NULL);

  shellInit();
  shellCreateStatic(&shell_cfg1, Shell_thread_wa,
      sizeof(Shell_thread_wa), NORMALPRIO);
}
