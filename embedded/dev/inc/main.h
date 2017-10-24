#ifndef _MAIN_H_
#define _MAIN_H_

#include "ch.h"
#include "hal.h"

#include "uart_host.h"
#include "mpu6050.h"
#include "calibrate_imu.h"
#include "attitude.h"
#include "stepper.h"

#include "flash.h"
#include "chprintf.h"
#include "tft_display.h"
#include "math_misc.h"

void shellStart(void);

#endif
