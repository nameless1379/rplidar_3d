#include "main.h"
#include "shell.h"
#include <string.h>

static THD_WORKING_AREA(Shell_thread_wa, 1024);

static void cmd_print(BaseSequentialStream * chp, int argc, char *argv[]){
  (void) argc,argv;

  chprintf(chp,"Step: %d\r\n", stepper_get_steps());
  chprintf(chp,"Angle: %f\r\n", stepper_get_angle()*180.0f/M_PI );
}

static void cmd_stepper(BaseSequentialStream * chp, int argc, char *argv[]){
  if(argc)
  {
    if(!strcmp(argv[0],"dir"))
      stepper_changedir();
    else
    {
      char *toNumber = argv[0];
      uint32_t finalNum=0;
      while(*toNumber>='0' && *toNumber<='9')
        finalNum=finalNum*10+*(toNumber++)-'0';

      float vel = finalNum * 2*M_PI/60;
      stepper_setvelocity(finalNum);
    }
  }
}

static void cmd_calibration(BaseSequentialStream * chp, int argc, char *argv[])
{
  PIMUStruct pIMU = mpu6050_get();

  if(pIMU->data_invalid)
  {
    chprintf(chp,"Incorrect IMU reading!\r\n");
    return;
  }

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
  {"step",cmd_stepper}
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
