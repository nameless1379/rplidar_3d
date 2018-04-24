#include "ch.h"
#include "hal.h"
#include "stepper.h"
#include <math.h>

#define PWM_PERIOD 12000000U
#define MIN_SPEED       0.8f  //Prevent 16bit timer from overflow
#define MAX_SPEED     8*M_PI

static const PWMConfig stepper_pwmcfg = {
  PWM_PERIOD,
  100,
  NULL,     /* Callback disabled.            */
  {         /* PWM channel configuration:    */
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},  /* CH1 */
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},  /* CH2 */
    {PWM_OUTPUT_DISABLED, NULL},     /* CH3 */
    {PWM_OUTPUT_DISABLED, NULL}      /* CH4 */
  },
  0, /* CR2 register value. */
#if STM32_PWM_USE_ADVANCED
  0, /* BDTR register value. Not used for TIM4. */
#endif
  0  /* DIER register value. */
};

/*
 * GPT3 configuration.
 */
static const GPTConfig stepper_counter_cfg = {
  84000000,
  NULL,   /* Timer callback.*/
  0,
  0
};

static stepperStruct stepper;
#define STEPPER_INITIAL_VALUE 0x7fffffff

float stepper_get_velocity(void)
{
  return stepper.velocity;
}

#define STEPPER_STEP2RAD    2*M_PI/STEPPER_DIV
int32_t stepper_get_steps(void)
{
  return STEPPER_COUNTER.tim->CNT - STEPPER_INITIAL_VALUE;
}

float stepper_get_angle(void)
{
  int16_t step = (STEPPER_COUNTER.tim->CNT - STEPPER_INITIAL_VALUE) % STEPPER_DIV;
  return (float)(step * STEPPER_STEP2RAD);
}

/**
 *  @brief      initialize stepper motor
 *
 *  @api
 */
void stepper_init(const stepper_direction_t dir)
{
    gptStart(&STEPPER_COUNTER, &stepper_counter_cfg);
    palSetPad(STEPPER_EN_GPIO,STEPPER_EN_PIN);
    pwmStart(&STEPPER_TIM, &stepper_pwmcfg);
    /* initialize the step counter*/

    STEPPER_TIM.tim->CR1 &= (~STM32_TIM_CR1_CEN);
    STEPPER_COUNTER.tim->CR1 &= (~STM32_TIM_CR1_CEN);

    STEPPER_COUNTER.tim->PSC = 0;
    STEPPER_COUNTER.tim->ARR = 0xffffffff;
    STEPPER_COUNTER.tim->CNT = STEPPER_INITIAL_VALUE;

    STEPPER_TIM.tim->CCR[STEPPER_DIR_CH] = (dir == STEPPER_CW ? 0 : stepper.ARR);
    STEPPER_COUNTER.tim->CR1 |= (dir == STEPPER_CW ? STM32_TIM_CR1_DIR :0);

    STEPPER_TIM.tim->CR2 |= STM32_TIM_CR2_MMS(4);
    STEPPER_COUNTER.tim->SMCR |= STM32_TIM_SMCR_SMS(7);
    STEPPER_COUNTER.tim->SMCR |= STM32_TIM_SMCR_TS(1); //Configured to be TIM8's slave

    STEPPER_COUNTER.tim->CR1 |= STM32_TIM_CR1_CEN;
    chThdSleepMilliseconds(1);
    STEPPER_TIM.tim->CR1 |= STM32_TIM_CR1_CEN;
}

/**
 *  @brief      set the velocity of stepper motors
 *  @param[in]  dir   direction of rotation STEPPER_CW or STEPPER_CCW
 *
 *  @api
 */
void stepper_setdir(const stepper_direction_t dir)
{
  stepper.dir = dir;
  if(stepper.state != STEPPER_STATE_NOT_INIT)
  {
    STEPPER_TIM.tim->CCR[STEPPER_DIR_CH] = (dir == STEPPER_CW ? 0 : stepper.ARR);
    STEPPER_COUNTER.tim->CR1 |= (dir == STEPPER_CW ? STM32_TIM_CR1_DIR :0);
  }
}

void stepper_changedir(void)
{
  stepper_setdir(stepper.dir == 0 ? 1 : 0);
}

#define  STEPPER_PSC  ((float)PWM_PERIOD * 2 * M_PI)/(float)(STEPPER_DIV)
/**
 *  @brief      set the velocity of stepper motors
 *  @param[in]  velocity   Amplitude of angular velocity of stepper motor in rad/s.
 *
 *  @api
 */
void stepper_setvelocity(const float vel)
{
  if(vel > MAX_SPEED || vel < MIN_SPEED)
    return;

  stepper.velocity = vel;
  if(vel == 0.0f)
  {
    stepper_stop();
    return;
  }
  else if(vel < 0.0f)
    stepper_setdir(STEPPER_CW);
  else
    stepper_setdir(STEPPER_CCW);

  stepper.ARR = (uint16_t)(STEPPER_PSC / vel);

  STEPPER_TIM.tim->ARR = stepper.ARR;
  STEPPER_TIM.tim->CCR[STEPPER_PWM_CH] = stepper.ARR/2;
}
