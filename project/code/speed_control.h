#ifndef __SPEED_CONTROL_H
#define __SPEED_CONTROL_H

#include "zf_common_headfile.h"

//----------------------------------------------------------------------------------------------------------------------
// Speed closed-loop control period
//----------------------------------------------------------------------------------------------------------------------
#define SPEED_CONTROL_PERIOD_S  0.1f

//----------------------------------------------------------------------------------------------------------------------
// Output limits
//----------------------------------------------------------------------------------------------------------------------
#define SPEED_CONTROL_OUTPUT_MIN       (  0)
#define SPEED_CONTROL_OUTPUT_MAX       ( 40)

//----------------------------------------------------------------------------------------------------------------------
// Integral output limit
//----------------------------------------------------------------------------------------------------------------------
#define SPEED_CONTROL_I_OUTPUT_MAX     ( 5.0f)

//----------------------------------------------------------------------------------------------------------------------
// Speed control state
//----------------------------------------------------------------------------------------------------------------------
typedef enum {
    SPEED_CONTROL_DISABLED = 0,
    SPEED_CONTROL_ENABLED,
    SPEED_CONTROL_EMERGENCY_STOP
} speed_control_state_t;

//----------------------------------------------------------------------------------------------------------------------
// Core interfaces
//----------------------------------------------------------------------------------------------------------------------
void    speed_control_init(void);
void    speed_control_process(void);
void    speed_control_enable(void);
void    speed_control_stop(void);

//----------------------------------------------------------------------------------------------------------------------
// Target
//----------------------------------------------------------------------------------------------------------------------
void    speed_control_set_target(float left_mps, float right_mps);
void    speed_control_set_left_target(float target_mps);
void    speed_control_set_right_target(float target_mps);
void    speed_control_stop_target(void);

//----------------------------------------------------------------------------------------------------------------------
// Getters
//----------------------------------------------------------------------------------------------------------------------
float   speed_control_get_left_target(void);
float   speed_control_get_right_target(void);
float   speed_control_get_left_actual(void);
float   speed_control_get_right_actual(void);
float   speed_control_get_left_error(void);
float   speed_control_get_right_error(void);
int8    speed_control_get_left_output(void);
int8    speed_control_get_right_output(void);
int8    speed_control_get_left_base(void);
int8    speed_control_get_right_base(void);
float   speed_control_get_left_i_correction(void);
float   speed_control_get_left_integral_error(void);
float   speed_control_get_right_i_correction(void);
float   speed_control_get_right_integral_error(void);

//----------------------------------------------------------------------------------------------------------------------
// State
//----------------------------------------------------------------------------------------------------------------------
speed_control_state_t speed_control_get_state(void);

//----------------------------------------------------------------------------------------------------------------------
// Wheel motor output interface layer
//----------------------------------------------------------------------------------------------------------------------
void    speed_control_set_left_output(int8 output);
void    speed_control_set_right_output(int8 output);
void    speed_control_set_output(int8 left_output, int8 right_output);

#endif
