#ifndef __MOTOR_H
#define __MOTOR_H

#include "zf_common_headfile.h"

#define MOTOR_PWM_FREQ  17000

#define MOTOR1_DIR      (IO_P50)
#define MOTOR1_PWM      (PWMD_CH2_P51)

#define MOTOR2_DIR      (IO_P52)
#define MOTOR2_PWM      (PWMD_CH4_P53)

#define LEFT_MIN_START_PWM      5
#define RIGHT_MIN_START_PWM     6

void motor_init(void);
void motor1_set(int8 duty);
void motor2_set(int8 duty);
void motor_set(int8 left_duty, int8 right_duty);
void motor_stop(void);

#endif
