/*
 * chassis.c
 *
 *  Created on: 10 Jan, 2018
 *      Author: ASUS
 */
#include "ch.h"
#include "hal.h"

#include "dbus.h"
#include "mpu6500.h"
#include "chassis.h"
#include "math_misc.h"

static volatile chassisStruct chassis;
pi_controller_t motor_vel_controllers[CHASSIS_MOTOR_NUM];
pid_controller_t heading_controller;
lpfilterStruct lp_speed[CHASSIS_MOTOR_NUM];
static RC_Ctl_t* pRC;
static PIMUStruct pIMU;

//RC input of chassis controller
static float   strafe_rc = 0.0f, drive_rc = 0.0f, heading_rc = 0.0f;
static int8_t  rc_reversed       = 1; //-1: reverse, 0: kill, 1: no reverse
static float   rc_speed_limit_sp = 1.0f;
static float   rc_speed_limit    = 1.0f; //value from 0 ~ 1
static float   rc_accl_limit     = 100.0f;  //value from 0-100
static float   heading_sp;

static float wheel_odeometry[3] = {0.0f, 0.0f, 0.0f}; //strafe, drive, heading

#define chassis_canUpdate()   \
  (can_motorSetCurrent(CHASSIS_CAN, CHASSIS_CAN_EID, \
    0, 0, 0, 0))

chassisStruct* chassis_get(void)
{
  return &chassis;
}

uint32_t chassis_getError(void)
{
  uint32_t errorFlag = chassis.errorFlag;
  chassis.errorFlag = 0;
  return errorFlag;
}

float* chassis_getWheelOdeometry(void)
{
  if(true)
    return wheel_odeometry;
}

void chassis_setSpeedLimit(const float speed)
{
  if(speed >= 0.0f && speed <= 1.0f)
    rc_speed_limit_sp = speed;
}

void chassis_setAcclLimit(const float accl)
{
  if(accl > 0.0f && accl <= 100.0f)
    rc_accl_limit = accl;
}

static void chassis_kill(void)
{
  LEDY_ON();
  if(chassis.state & CHASSIS_RUNNING)
  {
    chassis.state = CHASSIS_ERROR;
    can_motorSetCurrent(CHASSIS_CAN, CHASSIS_CAN_EID, 0, 0, 0, 0);
  }
}

// Set dead-zone to 2% range to provide smoother control
#define THRESHOLD (RC_CH_VALUE_MAX - RC_CH_VALUE_MIN)/100
static void chassis_inputCmd(void)
{
  int16_t RX_X2 = (pRC->rc.channel0 - RC_CH_VALUE_OFFSET
          #ifdef CHASSIS_USE_KEYBOARD
            + ((pRC->keyboard.key_code & KEY_D) - (pRC->keyboard.key_code & KEY_A))
            * RC_CHASSIS_KEYBOARD_SCALER * (pRC->keyboard.key_code & KEY_SHIFT ? 1.33f : 1.0f)
          #endif
          ) * rc_reversed,

          RX_Y1 = (pRC->rc.channel1 - RC_CH_VALUE_OFFSET
          #ifdef CHASSIS_USE_KEYBOARD
            + ((pRC->keyboard.key_code & KEY_W) - (pRC->keyboard.key_code & KEY_S))
            * RC_CHASSIS_KEYBOARD_SCALER * (pRC->keyboard.key_code & KEY_SHIFT ? 1.33f : 1.0f)
          #endif
          ) * rc_reversed,

          RX_X1 = (pRC->rc.channel2 - RC_CH_VALUE_OFFSET
          #ifdef CHASSIS_USE_KEYBOARD
            +pRC->mouse.x * RC_CHASSIS_MOUSE_SCALER
          #endif
          ) * rc_reversed;

  if(ABS(RX_X2) < THRESHOLD)
    RX_X2 = 0;
  if(ABS(RX_Y1) < THRESHOLD)
    RX_Y1 = 0;
  if(ABS(RX_X1) < THRESHOLD)
    RX_X1 = 0;

  if(RX_X2 > strafe_rc + CHASSIS_XYACCL_LIMIT_RATIO * rc_accl_limit)
    strafe_rc += CHASSIS_XYACCL_LIMIT_RATIO * rc_accl_limit;
  else if(RX_X2 < strafe_rc - CHASSIS_XYACCL_LIMIT_RATIO * rc_accl_limit)
    strafe_rc -= CHASSIS_XYACCL_LIMIT_RATIO * rc_accl_limit;
  else
    strafe_rc = RX_X2 ;

  if(RX_Y1 > drive_rc + rc_accl_limit)
    drive_rc += rc_accl_limit;
  else if(RX_Y1 < drive_rc - rc_accl_limit)
    drive_rc -= rc_accl_limit;
  else
    drive_rc = RX_Y1;

  if(RX_X1 > heading_rc + rc_accl_limit)
    heading_rc += rc_accl_limit;
  else if(RX_X1 < heading_rc - rc_accl_limit)
    heading_rc -= rc_accl_limit;
  else
    heading_rc = RX_X1;

  if(rc_speed_limit < rc_speed_limit_sp)
    rc_speed_limit += 0.005f;
  else if(rc_speed_limit > rc_speed_limit_sp)
    rc_speed_limit -= 0.005f;

  strafe_rc = boundOutput(strafe_rc, 660 * rc_speed_limit);
  drive_rc = boundOutput(drive_rc, 660 * rc_speed_limit);
  heading_rc = boundOutput(heading_rc, 660 * rc_speed_limit);
}

#define   CHASSIS_ANGLE_PSC 7.6699e-4 //2*M_PI/0x1FFF
#define   CHASSIS_SPEED_PSC 1.0f/((float)CHASSIS_GEAR_RATIO)
#define   CHASSIS_CONNECTION_ERROR_COUNT 20U
static void chassis_encoderUpdate(void)
{
  uint8_t i;
  for (i = 0; i < CHASSIS_MOTOR_NUM; i++)
  {
    if(chassis._encoders[i].updated)
    {
      //Check validiaty of can connection
      chassis._encoders[i].updated = false;

      //float pos_input = chassis._encoders[i].raw_angle*CHASSIS_ANGLE_PSC;
      float speed_input = chassis._encoders[i].raw_speed*CHASSIS_SPEED_PSC;
      chassis._motors[i]._speed = lpfilter_apply(&lp_speed[i], speed_input);
      chassis._motors[i]._wait_count = 1;
    }
    else
    {
      chassis._motors[i]._wait_count++;
      if(chassis._motors[i]._wait_count > CHASSIS_CONNECTION_ERROR_COUNT)
      {
        chassis_kill();

        chassis.errorFlag |= CHASSIS0_NOT_CONNECTED << i;
        chassis._motors[i]._wait_count = 1;
      }
    }
  }
  #ifdef CHASSIS_USE_POS_MOTOR
  #endif
}

#define OUTPUT_MAX  30000
static int16_t chassis_controlSpeed(const motorStruct* motor, pi_controller_t* const controller)
{
  float error = motor->speed_sp - motor->_speed;
  controller->error_int += error * controller->ki;
  controller->error_int = boundOutput(controller->error_int, controller->error_int_max);
  float output = error*controller->kp + controller->error_int;
  return (int16_t)(boundOutput(output,OUTPUT_MAX));
}

static void drive_kinematics(const float strafe_vel, const float drive_vel, const float heading_vel)
{
  chassis._motors[FRONT_RIGHT].speed_sp =
    strafe_vel - drive_vel + heading_vel;   // CAN ID: 0x201
  chassis._motors[BACK_RIGHT].speed_sp =
    -strafe_vel - drive_vel + heading_vel;       // CAN ID: 0x202
  chassis._motors[FRONT_LEFT].speed_sp =
    strafe_vel + drive_vel + heading_vel;       // CAN ID: 0x203
  chassis._motors[BACK_LEFT].speed_sp =
    -strafe_vel + drive_vel + heading_vel;     // CAN ID: 0x204

  uint8_t i;
  int16_t output[4];

  for (i = 0; i < CHASSIS_MOTOR_NUM; i++)
    output[i] = chassis_controlSpeed(&chassis._motors[i], &motor_vel_controllers[i]);

  can_motorSetCurrent(CHASSIS_CAN, CHASSIS_CAN_EID,
      	output[FRONT_RIGHT], output[FRONT_LEFT], output[BACK_LEFT], output[BACK_RIGHT]); //FL,FR,BR,BL
}

static THD_WORKING_AREA(chassis_control_wa, 2048);
static THD_FUNCTION(chassis_control, p)
{
  (void)p;
  chRegSetThreadName("chassis controller");

  //reserved for manual override of auto-driving
  uint16_t strafe_count = 0, drive_count = 0, heading_count = 0;
  uint16_t strafe_th = 65535, drive_th = 65535, heading_th = 65535;

  chThdSleepSeconds(2);

  uint32_t tick = chVTGetSystemTimeX();
  while(1)
  {
    tick += US2ST(CHASSIS_UPDATE_PERIOD_US);
    if(tick > chVTGetSystemTimeX())
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();
    }

    chassis_encoderUpdate();

    if(chassis.state & CHASSIS_RUNNING)
    {
      float strafe_vel, drive_vel, heading_vel;
      chassis_inputCmd();

      //These are remote control commands, mapped to match min and max CURRENT
      float strafe_input  = mapInput(strafe_rc, -660, 660, -VEL_MAX, VEL_MAX),
            drive_input   = mapInput(drive_rc, -660, 660, -VEL_MAX, VEL_MAX),
            heading_input = mapInput(heading_rc, -660, 660, -VEL_MAX, VEL_MAX);

      strafe_vel = strafe_input;

      drive_vel = drive_input;

      heading_vel = heading_input;

      drive_kinematics(strafe_vel, drive_vel, heading_vel);
    }
  }
}

static const FRvelName = "FR_vel";
static const FLvelName = "FL_vel";
static const BLvelName = "BL_vel";
static const BRvelName = "BR_vel";
static const HeadingName = "Heading";

#define MOTOR_VEL_INT_MAX 12000U
void chassis_init(void)
{
  memset(&chassis,0,sizeof(chassisStruct));
  pRC = RC_get();
  pIMU = imu_get();

  uint8_t i;
  params_set(&motor_vel_controllers[FRONT_LEFT], 9,2,FLvelName,subname_PI,PARAM_PUBLIC);
  params_set(&motor_vel_controllers[FRONT_RIGHT], 10,2,FRvelName,subname_PI,PARAM_PUBLIC);
  params_set(&motor_vel_controllers[BACK_LEFT], 11,2,BLvelName,subname_PI,PARAM_PUBLIC);
  params_set(&motor_vel_controllers[BACK_RIGHT], 12,2,BRvelName,subname_PI,PARAM_PUBLIC);

  for (i = 0; i < 4; i++)
  {
    chassis._motors[i].speed_sp = 0.0f;
    lpfilter_init(lp_speed + i, CHASSIS_UPDATE_FREQ, 20);
    motor_vel_controllers[i].error_int = 0.0f;
    motor_vel_controllers[i].error_int_max = MOTOR_VEL_INT_MAX;
  }
  heading_controller.error_int_max = 1.8f;
  chassis.heading_cmd = CHASSIS_DISABLE_AUTO;

  chassis._encoders = can_getChassisMotor();
  chThdCreateStatic(chassis_control_wa, sizeof(chassis_control_wa),
                          NORMALPRIO, chassis_control, NULL);

  chassis.state = CHASSIS_RUNNING;
}
