/*********************************************************************************************************************
* STC32G144K Opensourec Library
********************************************************************************************************************/
#include "zf_common_headfile.h"

//----------------------------------------------------------------------------------------------------------------------
// Rear steering servo isolated three-position test
//   PWME_CH1P_PA0, 50Hz, cycle: 82 -> 85 -> 88 -> 85 (each 2s)
//   No camera / UART8 / USB CDC / motor / encoder / ESC / speed_control / PID.
//----------------------------------------------------------------------------------------------------------------------
#define SERVO_PWM_PIN      (PWME_CH1P_PA0)
#define SERVO_FREQ         (50)

#define SERVO_DUTY(x)      ((float)PWM_DUTY_MAX / (1000.0 / (float)SERVO_FREQ) * (0.5 + (float)(x) / 90.0))

void main(void)
{
    clock_init(SYSTEM_CLOCK_96M);

    pwm_init(SERVO_PWM_PIN, SERVO_FREQ, 0);

    while(1)
    {
        pwm_set_duty(SERVO_PWM_PIN, (uint32)SERVO_DUTY(82.0f));
        system_delay_ms(2000);

        pwm_set_duty(SERVO_PWM_PIN, (uint32)SERVO_DUTY(85.0f));
        system_delay_ms(2000);

        pwm_set_duty(SERVO_PWM_PIN, (uint32)SERVO_DUTY(88.0f));
        system_delay_ms(2000);

        pwm_set_duty(SERVO_PWM_PIN, (uint32)SERVO_DUTY(85.0f));
        system_delay_ms(2000);
    }
}
