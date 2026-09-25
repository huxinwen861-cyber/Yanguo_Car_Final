#ifndef __SPEED_CONTROL_H
#define __SPEED_CONTROL_H

#include "zf_common_headfile.h"

// Existing interface (placeholder, not tuned yet)
void speed_control_init(void);
void speed_control_update(void);
void speed_control_set_target(int16 left_target, int16 right_target);
void speed_control_stop(void);

// ESC output interface layer
// All ESC output must go through these, never call esc_* directly from main/controller
void speed_control_set_left_output(uint16 duty);
void speed_control_set_right_output(uint16 duty);
void speed_control_set_output(uint16 left_duty, uint16 right_duty);

// Safe start wrappers
void speed_control_safe_start_left(void);
void speed_control_safe_start_right(void);
void speed_control_safe_start_both(void);

#endif
