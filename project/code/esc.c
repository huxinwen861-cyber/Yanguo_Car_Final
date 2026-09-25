#include "esc.h"

//----------------------------------------------------------------------------------------------------------------------
// Clamp duty: never go below ESC_IDLE_DUTY during normal operation
//----------------------------------------------------------------------------------------------------------------------
static uint16 clamp_duty(uint16 duty)
{
    if(duty < ESC_IDLE_DUTY)
    {
        duty = ESC_IDLE_DUTY;
    }
    return duty;
}

//----------------------------------------------------------------------------------------------------------------------
// ESC initialization sequence:
// 1. Init both channels at duty=0
// 2. Hold duty=0 for 200ms (ESC power-on blanking)
// 3. Set both to ESC_IDLE_DUTY (500)
// 4. Hold 5s for ESC arming/self-test
// 5. Both channels remain at 500 after init
//----------------------------------------------------------------------------------------------------------------------
void esc_init(void)
{
    pwm_init(ESC_LEFT_PIN,  ESC_PWM_FREQ_HZ, 0);
    pwm_init(ESC_RIGHT_PIN, ESC_PWM_FREQ_HZ, 0);

    system_delay_ms(200);

    pwm_set_duty(ESC_LEFT_PIN,  ESC_IDLE_DUTY);
    pwm_set_duty(ESC_RIGHT_PIN, ESC_IDLE_DUTY);

    system_delay_ms(5000);
}

//----------------------------------------------------------------------------------------------------------------------
// Stop: both channels to IDLE_DUTY
//----------------------------------------------------------------------------------------------------------------------
void esc_stop_all(void)
{
    pwm_set_duty(ESC_LEFT_PIN,  ESC_IDLE_DUTY);
    pwm_set_duty(ESC_RIGHT_PIN, ESC_IDLE_DUTY);
}

//----------------------------------------------------------------------------------------------------------------------
// Single-channel stop
//----------------------------------------------------------------------------------------------------------------------
void esc_left_stop(void)
{
    pwm_set_duty(ESC_LEFT_PIN, ESC_IDLE_DUTY);
}

void esc_right_stop(void)
{
    pwm_set_duty(ESC_RIGHT_PIN, ESC_IDLE_DUTY);
}

//----------------------------------------------------------------------------------------------------------------------
// Safe start: set channel to ESC_SAFE_START_DUTY (550)
//----------------------------------------------------------------------------------------------------------------------
void esc_left_safe_start(void)
{
    pwm_set_duty(ESC_LEFT_PIN, ESC_SAFE_START_DUTY);
}

void esc_right_safe_start(void)
{
    pwm_set_duty(ESC_RIGHT_PIN, ESC_SAFE_START_DUTY);
}

void esc_both_safe_start(void)
{
    pwm_set_duty(ESC_LEFT_PIN,  ESC_SAFE_START_DUTY);
    pwm_set_duty(ESC_RIGHT_PIN, ESC_SAFE_START_DUTY);
}

//----------------------------------------------------------------------------------------------------------------------
// Low-level duty control with safety clamping
//----------------------------------------------------------------------------------------------------------------------
void esc_left_set_duty(uint16 duty)
{
    duty = clamp_duty(duty);
    pwm_set_duty(ESC_LEFT_PIN, duty);
}

void esc_right_set_duty(uint16 duty)
{
    duty = clamp_duty(duty);
    pwm_set_duty(ESC_RIGHT_PIN, duty);
}
