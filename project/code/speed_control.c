#include "zf_common_headfile.h"
#include "speed_control.h"
#include "encoder.h"
#include "motor.h"

//----------------------------------------------------------------------------------------------------------------------
// Speed closed-loop foundation state
//----------------------------------------------------------------------------------------------------------------------
static float left_target_mps  = 0.0f;
static float right_target_mps = 0.0f;

static float left_actual_mps  = 0.0f;
static float right_actual_mps = 0.0f;

static float left_error_mps   = 0.0f;
static float right_error_mps  = 0.0f;

static uint32 last_processed_sequence = 0;

//----------------------------------------------------------------------------------------------------------------------
// Controller state
//----------------------------------------------------------------------------------------------------------------------
static float speed_kp = 4.0f;

static int8  left_base_command     = 0;
static int8  right_base_command    = 0;
static int8  left_control_output   = 0;
static int8  right_control_output  = 0;

//----------------------------------------------------------------------------------------------------------------------
// Integral state
//----------------------------------------------------------------------------------------------------------------------
static float speed_ki              = 0.15f;
static float left_integral_error   = 0.0f;
static float left_i_correction     = 0.0f;
static float right_integral_error  = 0.0f;
static float right_i_correction    = 0.0f;

//----------------------------------------------------------------------------------------------------------------------
// Startup grace period (base-only, no P/I)
//----------------------------------------------------------------------------------------------------------------------
static uint8 pi_startup_samples_remaining = 0;

//----------------------------------------------------------------------------------------------------------------------
// Speed control state
//----------------------------------------------------------------------------------------------------------------------
static speed_control_state_t sc_state = SPEED_CONTROL_DISABLED;

//----------------------------------------------------------------------------------------------------------------------
// Target -> base command (feedforward from calibration)
//----------------------------------------------------------------------------------------------------------------------
static int8 speed_control_target_to_base(float target_mps)
{
    if(target_mps <= 0.0f)  return 0;
    if(target_mps <= 4.0f)  return 24;
    if(target_mps <= 5.0f)  return 30;
    if(target_mps <= 6.0f)  return 35;
    if(target_mps <= 8.0f)  return 40;
    return 50;
}

//----------------------------------------------------------------------------------------------------------------------
// Reset integral state (both wheels)
//----------------------------------------------------------------------------------------------------------------------
static void speed_control_reset_integral(void)
{
    left_integral_error  = 0.0f;
    left_i_correction    = 0.0f;
    right_integral_error = 0.0f;
    right_i_correction   = 0.0f;
}

//----------------------------------------------------------------------------------------------------------------------
// Initialize: all targets, errors, PI state to zero, motors stopped
//----------------------------------------------------------------------------------------------------------------------
void speed_control_init(void)
{
    left_target_mps  = 0.0f;
    right_target_mps = 0.0f;
    left_actual_mps  = 0.0f;
    right_actual_mps = 0.0f;
    left_error_mps   = 0.0f;
    right_error_mps  = 0.0f;
    left_base_command    = 0;
    right_base_command   = 0;
    left_control_output  = 0;
    right_control_output = 0;
    last_processed_sequence = speed_measure_get_sequence();

    speed_control_reset_integral();
    pi_startup_samples_remaining = 0;

    sc_state = SPEED_CONTROL_DISABLED;
    motor_stop();
}

//----------------------------------------------------------------------------------------------------------------------
// Single-wheel PI computation helper
//----------------------------------------------------------------------------------------------------------------------
static void speed_control_compute_single_pi(
    float target, float error,
    float *integral_error, float *i_correction,
    int8 prev_output, int8 *control_output,
    int8 base_cmd)
{
    float p_val, theoretical;
    float integral_limit;
    int16 cmd;
    int8  saturated;

    p_val = (float)base_cmd + speed_kp * error;

    if(speed_ki <= 0.0f || target <= 0.0f)
    {
        *integral_error = 0.0f;
        *i_correction   = 0.0f;
    }
    else
    {
        saturated = 0;
        if(prev_output >= SPEED_CONTROL_OUTPUT_MAX && error > 0.0f)
        {
            saturated = 1;
        }
        if(prev_output <= 0 && error < 0.0f)
        {
            saturated = 1;
        }

        if(!saturated)
        {
            *integral_error += error * SPEED_CONTROL_PERIOD_S;
        }

        integral_limit = SPEED_CONTROL_I_OUTPUT_MAX / speed_ki;
        if(*integral_error >  integral_limit) *integral_error =  integral_limit;
        if(*integral_error < -integral_limit) *integral_error = -integral_limit;

        *i_correction = speed_ki * (*integral_error);
        if(*i_correction >  SPEED_CONTROL_I_OUTPUT_MAX) *i_correction =  SPEED_CONTROL_I_OUTPUT_MAX;
        if(*i_correction < -SPEED_CONTROL_I_OUTPUT_MAX) *i_correction = -SPEED_CONTROL_I_OUTPUT_MAX;
    }

    theoretical = p_val + (*i_correction);

    cmd = (int16)(theoretical + 0.5f);
    if(cmd < 0)                          cmd = 0;
    if(cmd > SPEED_CONTROL_OUTPUT_MAX)   cmd = SPEED_CONTROL_OUTPUT_MAX;

    *control_output = (int8)cmd;
}

//----------------------------------------------------------------------------------------------------------------------
// Speed control process: read actual speed, compute error, compute PI output
// Call after speed_measure_process() in main loop.
//----------------------------------------------------------------------------------------------------------------------
void speed_control_process(void)
{
    uint32 current_sequence;

    if(sc_state == SPEED_CONTROL_DISABLED)
    {
        motor1_set(0);
        motor2_set(0);
        return;
    }

    if(sc_state == SPEED_CONTROL_EMERGENCY_STOP)
    {
        motor1_set(0);
        motor2_set(0);
        return;
    }

    current_sequence = speed_measure_get_sequence();
    if(current_sequence == last_processed_sequence)
    {
        return;
    }

    left_actual_mps  = (float)left_speed_mps;
    right_actual_mps = (float)right_speed_mps;

    left_error_mps  = left_target_mps  - left_actual_mps;
    right_error_mps = right_target_mps - right_actual_mps;

    left_base_command  = speed_control_target_to_base(left_target_mps);
    right_base_command = speed_control_target_to_base(right_target_mps);

    /* Startup grace: both wheels base-only for 5 samples */
    if(pi_startup_samples_remaining > 0)
    {
        left_control_output  = left_base_command;
        right_control_output = right_base_command;
        left_integral_error  = 0.0f;
        left_i_correction    = 0.0f;
        right_integral_error = 0.0f;
        right_i_correction   = 0.0f;
        motor1_set(left_control_output);
        motor2_set(right_control_output);
        pi_startup_samples_remaining--;
        last_processed_sequence = current_sequence;
        return;
    }

    /* Left wheel PI */
    speed_control_compute_single_pi(
        left_target_mps, left_error_mps,
        &left_integral_error, &left_i_correction,
        left_control_output, &left_control_output,
        left_base_command);

    /* Right wheel PI */
    speed_control_compute_single_pi(
        right_target_mps, right_error_mps,
        &right_integral_error, &right_i_correction,
        right_control_output, &right_control_output,
        right_base_command);

    motor1_set(left_control_output);
    motor2_set(right_control_output);

    last_processed_sequence = current_sequence;
}

//----------------------------------------------------------------------------------------------------------------------
// Set target speed (m/s) — both channels
//----------------------------------------------------------------------------------------------------------------------
void speed_control_set_target(float left_mps, float right_mps)
{
    if(left_mps < 0.0f)  left_mps  = 0.0f;
    if(right_mps < 0.0f) right_mps = 0.0f;

    left_target_mps  = left_mps;
    right_target_mps = right_mps;
}

//----------------------------------------------------------------------------------------------------------------------
// Set target speed (m/s) — single channel
//----------------------------------------------------------------------------------------------------------------------
void speed_control_set_left_target(float target_mps)
{
    if(target_mps < 0.0f) target_mps = 0.0f;
    left_target_mps = target_mps;
}

void speed_control_set_right_target(float target_mps)
{
    if(target_mps < 0.0f) target_mps = 0.0f;
    right_target_mps = target_mps;
}

//----------------------------------------------------------------------------------------------------------------------
// Clear all targets to zero
//----------------------------------------------------------------------------------------------------------------------
void speed_control_stop_target(void)
{
    left_target_mps  = 0.0f;
    right_target_mps = 0.0f;
}

//----------------------------------------------------------------------------------------------------------------------
// Getter functions
//----------------------------------------------------------------------------------------------------------------------
float speed_control_get_left_target(void)  { return left_target_mps;  }
float speed_control_get_right_target(void) { return right_target_mps; }
float speed_control_get_left_actual(void)  { return left_actual_mps;  }
float speed_control_get_right_actual(void) { return right_actual_mps; }
float speed_control_get_left_error(void)   { return left_error_mps;   }
float speed_control_get_right_error(void)  { return right_error_mps;  }

//----------------------------------------------------------------------------------------------------------------------
// Control output getters (read-only)
//----------------------------------------------------------------------------------------------------------------------
int8 speed_control_get_left_output(void)  { return left_control_output;  }
int8 speed_control_get_right_output(void) { return right_control_output; }

//----------------------------------------------------------------------------------------------------------------------
// Base command getters (read-only)
//----------------------------------------------------------------------------------------------------------------------
int8 speed_control_get_left_base(void)  { return left_base_command;  }
int8 speed_control_get_right_base(void) { return right_base_command; }

//----------------------------------------------------------------------------------------------------------------------
// Integral getters
//----------------------------------------------------------------------------------------------------------------------
float speed_control_get_left_i_correction(void)   { return left_i_correction;   }
float speed_control_get_left_integral_error(void)  { return left_integral_error;  }
float speed_control_get_right_i_correction(void)  { return right_i_correction;  }
float speed_control_get_right_integral_error(void) { return right_integral_error; }

//----------------------------------------------------------------------------------------------------------------------
// Enable speed control (dual-wheel independent PI)
//----------------------------------------------------------------------------------------------------------------------
void speed_control_enable(void)
{
    if(sc_state == SPEED_CONTROL_EMERGENCY_STOP)
    {
        return;
    }
    speed_control_reset_integral();
    last_processed_sequence = speed_measure_get_sequence();
    pi_startup_samples_remaining = 5;
    sc_state = SPEED_CONTROL_ENABLED;
}

//----------------------------------------------------------------------------------------------------------------------
// Stop: clear all state, stop wheel motors
//----------------------------------------------------------------------------------------------------------------------
void speed_control_stop(void)
{
    left_target_mps  = 0.0f;
    right_target_mps = 0.0f;
    left_actual_mps  = 0.0f;
    right_actual_mps = 0.0f;
    left_error_mps   = 0.0f;
    right_error_mps  = 0.0f;
    left_base_command    = 0;
    right_base_command   = 0;
    left_control_output  = 0;
    right_control_output = 0;
    speed_control_reset_integral();
    pi_startup_samples_remaining = 0;
    sc_state = SPEED_CONTROL_DISABLED;
    motor_stop();
}

//----------------------------------------------------------------------------------------------------------------------
// Wheel motor output interface layer
//----------------------------------------------------------------------------------------------------------------------
void speed_control_set_left_output(int8 output)
{
    motor1_set(output);
}

void speed_control_set_right_output(int8 output)
{
    motor2_set(output);
}

void speed_control_set_output(int8 left_output, int8 right_output)
{
    motor_set(left_output, right_output);
}

//----------------------------------------------------------------------------------------------------------------------
// State
//----------------------------------------------------------------------------------------------------------------------
speed_control_state_t speed_control_get_state(void)
{
    return sc_state;
}
