#include "motor.h"

static int8 clamp_duty(int8 duty)
{
    if(duty > 100)  duty = 100;
    if(duty < -100) duty = -100;
    return duty;
}

void motor_init(void)
{
    gpio_init(MOTOR1_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR1_PWM, MOTOR_PWM_FREQ, 0);

    gpio_init(MOTOR2_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR2_PWM, MOTOR_PWM_FREQ, 0);
}

void motor1_set(int8 duty)
{
    duty = clamp_duty(duty);
    if(duty >= 0)
    {
        gpio_set_level(MOTOR1_DIR, GPIO_LOW);
        pwm_set_duty(MOTOR1_PWM, (uint32)duty * (PWM_DUTY_MAX / 100));
    }
    else
    {
        gpio_set_level(MOTOR1_DIR, GPIO_HIGH);
        pwm_set_duty(MOTOR1_PWM, (uint32)(-duty) * (PWM_DUTY_MAX / 100));
    }
}

void motor2_set(int8 duty)
{
    duty = clamp_duty(duty);
    if(duty >= 0)
    {
        gpio_set_level(MOTOR2_DIR, GPIO_LOW);
        pwm_set_duty(MOTOR2_PWM, (uint32)duty * (PWM_DUTY_MAX / 100));
    }
    else
    {
        gpio_set_level(MOTOR2_DIR, GPIO_HIGH);
        pwm_set_duty(MOTOR2_PWM, (uint32)(-duty) * (PWM_DUTY_MAX / 100));
    }
}

void motor_set(int8 left_duty, int8 right_duty)
{
    motor1_set(left_duty);
    motor2_set(right_duty);
}

void motor_stop(void)
{
    motor1_set(0);
    motor2_set(0);
}
