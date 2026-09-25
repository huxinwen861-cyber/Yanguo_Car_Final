#ifndef __ESC_H
#define __ESC_H

#include "zf_common_headfile.h"

// ESC PWM frequency
#define ESC_PWM_FREQ_HZ             50

// Duty cycle constants (based on实测 50Hz @ 96MHz, PWM_DUTY_MAX=10000)
// duty=500 → 1.0ms → idle / stop
// duty=540 → 1.08ms → 实测最低稳定起转
// duty=550 → 1.1ms → 推荐安全起转值
#define ESC_IDLE_DUTY               500
#define ESC_MIN_STABLE_START_DUTY   540
#define ESC_SAFE_START_DUTY         550

// Physical channel mapping
// ESC_LEFT  = PA5 / PWMF_CH3_PA5
// ESC_RIGHT = PA1 / PWMF_CH1_PA1
#define ESC_LEFT_PIN    (PWMF_CH3_PA5)
#define ESC_RIGHT_PIN   (PWMF_CH1_PA1)

// Initialization: both channels duty=0 → 200ms → duty=500 → 5s arming wait
void esc_init(void);

// Stop: set both channels to IDLE_DUTY (500)
void esc_stop_all(void);

// Single-channel stop
void esc_left_stop(void);
void esc_right_stop(void);

// Safe start: set to ESC_SAFE_START_DUTY (550)
void esc_left_safe_start(void);
void esc_right_safe_start(void);
void esc_both_safe_start(void);

// Low-level duty control with safety clamping
// duty < ESC_IDLE_DUTY is clamped to ESC_IDLE_DUTY
void esc_left_set_duty(uint16 duty);
void esc_right_set_duty(uint16 duty);

#endif
