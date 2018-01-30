/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/
#include "main.h"

static BaseSequentialStream* chp = (BaseSequentialStream*)SERIAL_CMD;
static PIMUStruct pIMU;
static const IMUConfigStruct imu1_conf = {&I2CD1, MPU6050_I2C_ADDR_A0_LOW,
   MPU6050_ACCEL_SCALE_8G, MPU6050_GYRO_SCALE_1000};

static THD_WORKING_AREA(Attitude_thread_wa, 4096);
static THD_FUNCTION(Attitude_thread, p)
{
  chRegSetThreadName("IMU Attitude Estimator");
  uint8_t errorCode;

  PIMUStruct pIMU_1 = (PIMUStruct)p;

  chThdSleepMilliseconds(100);
  errorCode = attitude_imu_init(pIMU_1, &imu1_conf);

  while(errorCode)
  {
    tft_printf(0,1,"E:IMU Init Failed: %d", errorCode);
    pIMU->data_invalid = true;
    chThdSleepMilliseconds(500);
  }

  uint32_t tick = chVTGetSystemTimeX();
  while(true)
  {
    tick += US2ST(MPU6050_UPDATE_PERIOD);
    if(chVTGetSystemTimeX() < tick)
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();
      palSetPad(GPIOD,GPIOD_LED4);
    }

    errorCode = attitude_update(pIMU_1);

    while(errorCode)
    {
      palSetPad(GPIOD,GPIOD_LED3);
      pIMU->data_invalid = true;
      chprintf(0,1,"E:IMU Reading Error %d", errorCode);
      chThdSleepMilliseconds(500);
    }

    if(pIMU_1->accelerometer_not_calibrated || pIMU_1->gyroscope_not_calibrated)
    {
      chSysLock();
      chThdSuspendS(&(pIMU_1->imu_Thd));
      chSysUnlock();
    }
  }
}

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  shellStart();
  tft_init(TFT_HORIZONTAL, CYAN,BLACK,BLACK);
  tft_printf(1,0,"RPLIDAR CONTROLLER");
  tft_printf(3,7,"Edward ZHANG");

  stepper_init(STEPPER_CCW);
  stepper_setvelocity(2*M_PI);

  pIMU = mpu6050_get();

  chThdCreateStatic(Attitude_thread_wa, sizeof(Attitude_thread_wa),
                    NORMALPRIO + 5,
                    Attitude_thread, pIMU);

  chThdSleepMilliseconds(10);
  uart_host_init();

  while (true)
  {
    palTogglePad(GPIOB,GPIOB_LED);
    chThdSleepMilliseconds(200);
  }

  return 0;
}
