/**
 * Edward ZHANG
 * @file    shellcfg.c
 * @brief   definitions of shell command functions
 */
#include "main.h"
#include "shell.h"
#include <string.h>

#define SERIAL_CMD       &SDU1
#define SERIAL_DATA      &SDU1

static thread_t* matlab_thread_handler = NULL;
/**
 * @brief Transmit uint32_t and float through serial port to host machine
 * @require Initialization of ChibiOS serial driver before using this function
 *
 * @param[in] chp         pointer to a @p BaseSequentialStream implementing object
 * @param[in] txbuf_d     array of 32-bit integers to tramsmit, can be signed or unsigned
 * @param[in] txbuf_f     array of float point numbers to tramsmit
 * @param[in] num_int     number of 32-bit integers to tramsmit
 * @param[in] num_float   number of float point numbers to tramsmit
 *
 * @TODO improve the transmission protocol to enable easier setup for the host machine
 */
#define SYNC_SEQ  0xaabbccdd
static void transmit_matlab
  (BaseSequentialStream* chp,
    uint32_t* const txbuf_d, float* const txbuf_f,
    const uint8_t num_int, const uint8_t num_float)
{
  uint32_t sync = SYNC_SEQ;
  char* byte = (char*)&sync;

  uint8_t i;
  for (i = 0; i < 4; i++)
    chSequentialStreamPut(chp, *byte++);

  byte = (char*)txbuf_d;
  for (i = 0; i < 4*num_int; i++)
    chSequentialStreamPut(chp, *byte++);

  byte = (char*)txbuf_f;
  for (i = 0; i < 4*num_float; i++)
    chSequentialStreamPut(chp, *byte++);
}

#define HOST_TRANSMIT_FREQ  100U
static THD_WORKING_AREA(matlab_thread_wa, 512);
static THD_FUNCTION(matlab_thread, p)
{
  (void)p;
  chRegSetThreadName("matlab tramsmitter");

  int32_t txbuf_d[16];
  float txbuf_f[16];
  BaseSequentialStream* chp = (BaseSequentialStream*)SERIAL_DATA;

  PIMUStruct pIMU = imu_get();

  uint32_t tick = chVTGetSystemTimeX();
  const uint16_t period = US2ST(1000000/HOST_TRANSMIT_FREQ);
  while (!chThdShouldTerminateX())
  {
    tick += period;
    if(tick > chVTGetSystemTimeX())
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();
    }

    transmit_matlab(chp, NULL, txbuf_f, 0, 2);
  }
}

/*===========================================================================*/
/* Definitions of shell command functions                                    */
/*===========================================================================*/
static THD_WORKING_AREA(Shell_thread_wa, 1024);
void cmd_test(BaseSequentialStream * chp, int argc, char *argv[])
{
  (void) argc,argv;
  PIMUStruct PIMU = imu_get();
  float* odeometry = chassis_wheelOdeometryGet();

  chprintf(chp,"accelFiltered[X]: %f\r\n",PIMU->accelFiltered[X]);
  chprintf(chp,"accelFiltered[Y]: %f\r\n",PIMU->accelFiltered[Y]);
  chprintf(chp,"accelFiltered[Z]: %f\r\n",PIMU->accelFiltered[Z]);

  chprintf(chp,"Roll:  %f\r\n",PIMU->euler_angle[Roll]);
  chprintf(chp,"Pitch: %f\r\n",PIMU->euler_angle[Pitch]);
  chprintf(chp,"Yaw:   %f\r\n",PIMU->euler_angle[Yaw]);

  chprintf(chp,"strafe:  %f\r\n",  odeometry[CHASSIS_STRAFE]);
  chprintf(chp,"drive: %f\r\n",    odeometry[CHASSIS_DRIVE]);
  chprintf(chp,"heading:   %f\r\n",odeometry[CHASSIS_HEADING]);

  chprintf(chp,"Stepper angle: %f\r\n", stepper_get_angle());
}

#ifdef SHOOTER_SETUP
  /**
   * @brief Start the data tramsmission to matlab
   * @note caution of data flooding to the serial port
   */
  void cmd_shooter_setup(BaseSequentialStream * chp, int argc, char *argv[])
  {
    if(argc)
    {
      if(!strcmp(argv[0], "set1"))
      {
        pwm12_setWidth(900);
        chprintf(chp,"Set pwm to max\r\n");
      }
      else if(!strcmp(argv[0], "set2"))
      {
        pwm12_setWidth(100);
        chprintf(chp,"Set pwm to min\r\n");
      }
    }
    else
      chprintf(chp,"cmd: set1, set2\r\n");
  }
#endif

/**
 * @brief Start the data tramsmission to matlab
 * @note caution of data flooding to the serial port
 */
void cmd_data(BaseSequentialStream * chp, int argc, char *argv[])
{
  uint8_t sec = 10;

  if(argc && matlab_thread_handler == NULL)
  {
    char *toNumber = argv[0];
    uint32_t finalNum=0;
    while(*toNumber>='0' && *toNumber<='9')
      finalNum=finalNum*10+*(toNumber++)-'0';

    if(finalNum == 0)
      finalNum = 10;

    sec = (finalNum < 60 ? finalNum : 60);

    chprintf(chp,"Data transmission start in %d seconds...\r\n", sec);
    chThdSleepSeconds(sec);

    matlab_thread_handler = chThdCreateStatic(matlab_thread_wa, sizeof(matlab_thread_wa),
        NORMALPRIO - 3,
        matlab_thread, NULL);
  }
  else if(matlab_thread_handler != NULL)
  {
    chThdTerminate(matlab_thread_handler);
    matlab_thread_handler = NULL;
  }
}

void cmd_calibrate(BaseSequentialStream * chp, int argc, char *argv[])
{
  PIMUStruct pIMU = imu_get();

  if(argc)
  {
    if(!strcmp(argv[0], "accl"))
    {
      if(pIMU->state == IMU_STATE_READY)
      {
        pIMU->accelerometer_not_calibrated = true;

        pIMU->state == IMU_STATE_CALIBRATING;

        chThdSleepMilliseconds(10);
        calibrate_accelerometer(pIMU);
        chThdResume(&(pIMU->imu_Thd), MSG_OK);

        pIMU->state == IMU_STATE_READY;
      }
      else
        chprintf(chp, "IMU initialization not complete\r\n");
    }
    else if(!strcmp(argv[0], "gyro"))
    {
      if(pIMU->state == IMU_STATE_READY)
      {
        pIMU->gyroscope_not_calibrated = true;

        pIMU->state == IMU_STATE_CALIBRATING;

        chThdSleepMilliseconds(10);
        calibrate_gyroscope(pIMU);
        chThdResume(&(pIMU->imu_Thd), MSG_OK);

        pIMU->state == IMU_STATE_READY;
      }
      else
        chprintf(chp, "IMU initialization not complete\r\n");

    }
    param_save_flash();
  }
  else
    chprintf(chp,"Calibration: gyro, accl, adi, adi-full\r\n");
}

void cmd_temp(BaseSequentialStream * chp, int argc, char *argv[])
{
  (void) argc,argv;

  //while(1){ // you can uncomment this so that it continuously send the data out.
  //            // this is useful in tuning the Temperature PID
      PIMUStruct _pimu = imu_get();
      TPIDStruct* _tempPID = TPID_get();
      chprintf(chp,"Temperature: %f\r\n", _pimu->temperature);
      chprintf(chp,"PID_value: %d\r\n", _tempPID->PID_Value);

      chThdSleepMilliseconds(500);
  //}
}

/**
 * @brief array of shell commands, put the corresponding command and functions below
 * {"command", callback_function}
 */
static const ShellCommand commands[] =
{
  {"test", cmd_test},
  {"cal", cmd_calibrate},
  {"temp", cmd_temp},
  {"\xEE", cmd_data},
  #ifdef PARAMS_USE_USB
    {"\xFD",cmd_param_scale},
    {"\xFB",cmd_param_update},
    {"\xFA",cmd_param_tx},
    {"\xF9",cmd_param_rx},
  #endif
  #ifdef SHOOTER_SETUP
    {"shoot", cmd_shooter_setup},
  #endif
  {NULL, NULL}
};

static const ShellConfig shell_cfg1 =
{
  (BaseSequentialStream *)SERIAL_CMD,
  commands
};

#define FLUSH_SDU_IQUEUE() (memset(SDU1.ib, 0, BQ_BUFFER_SIZE(SERIAL_USB_BUFFERS_NUMBER,    \
                                            SERIAL_USB_BUFFERS_SIZE)))

#define FLUSH_SDU_OQUEUE() (memset(SDU1.ob, 0, BQ_BUFFER_SIZE(SERIAL_USB_BUFFERS_NUMBER,    \
                                            SERIAL_USB_BUFFERS_SIZE)))

/**
 * @brief start the shell service
 * @require enable the corresponding serial ports in mcuconf.h and board.h
 *          Select the SERIAL_CMD port in main.h
 *
 * @api
 */
void shellStart(void)
{
  //sdStart(SERIAL_CMD, NULL);
  /*
   * Initializes a serial-over-USB CDC driver.
   */

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */

  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);

  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(1500);

  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);

  chThdSleepMilliseconds(500);
  shellInit();

  chSysLock();
  FLUSH_SDU_IQUEUE();
  FLUSH_SDU_OQUEUE();
  chSysUnlock();

  shellCreateStatic(&shell_cfg1, Shell_thread_wa,
      sizeof(Shell_thread_wa), NORMALPRIO);

}
