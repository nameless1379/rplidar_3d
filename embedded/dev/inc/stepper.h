#ifndef _STEPPER_H_
#define _STEPPER_H_

#define STEPPER_DIV       1600 //divisions per revolution of stepper motor

#define STEPPER_TIM      PWMD3
#define STEPPER_PWM_CH      0U
#define STEPPER_DIR_CH      1U

#define STEPPER_COUNTER  GPTD2

#define STEPPER_EN_GPIO  GPIOC
#define STEPPER_EN_PIN      3U

#define stepper_stop() (STEPPER_TIM.tim->CCR[STEPPER_PWM_CH] = 0)

typedef enum
{
  STEPPER_STATE_NOT_INIT = 0,
  STEPPER_STATE_RUNNING,
  STEPPER_STATE_STOPP
};

typedef enum
 {
   STEPPER_CCW = 0,
   STEPPER_CW = 1
 } stepper_direction_t;

typedef struct{
  uint8_t state;

  stepper_direction_t dir;
  float velocity;
  uint16_t ARR;

} stepperStruct;

float stepper_get_velocity(void);
int32_t stepper_get_steps(void);
float stepper_get_angle(void);

void stepper_init(const stepper_direction_t dir);
void stepper_setdir(const stepper_direction_t dir);
void stepper_setvelocity(const float vel);

#endif
