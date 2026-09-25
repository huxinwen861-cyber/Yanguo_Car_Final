#include "zf_common_headfile.h"
#include "encoder.h"

// Hardware: Right wheel = PWMA (P60/P62), Left wheel = PWMC (P40/P42)
#define ENCODER_QUAD_LEFT      (PWMC_ENCODER)
#define ENCODER_QUAD_LEFT_CHA  (PWMC_ENCODER_CH1P_P40)
#define ENCODER_QUAD_LEFT_CHB  (PWMC_ENCODER_CH2P_P42)

#define ENCODER_QUAD_RIGHT     (PWMA_ENCODER)
#define ENCODER_QUAD_RIGHT_CHA (PWMA_ENCODER_CH1P_P60)
#define ENCODER_QUAD_RIGHT_CHB (PWMA_ENCODER_CH2P_P62)

// ISR -> main shared variables
static volatile int16  sampled_left_count   = 0;
static volatile int16  sampled_right_count  = 0;
static volatile uint8  new_speed_sample_flag = 0;

// 50ms accumulator -> 100ms publish
static volatile int32  left_acc             = 0;
static volatile int32  right_acc            = 0;
static volatile uint8  sample_half          = 0;

// Public state
volatile int16  left_count          = 0;
volatile int16  right_count         = 0;
volatile float  left_speed_mps      = 0.0f;
volatile float  right_speed_mps     = 0.0f;
volatile uint32 sample_overrun_count = 0;

// Speed measurement sequence counter (incremented after each successful m/s update)
static volatile uint32 speed_measure_sequence = 0;

//----------------------------------------------------------------------------------------------------------------------
// TIM4_PIT 50ms ISR: read encoders, accumulate for 100ms publish
//----------------------------------------------------------------------------------------------------------------------
static void pit4_isr(void)
{
    int16 left_raw, right_raw;

    left_raw  =  encoder_get_count(ENCODER_QUAD_LEFT);
    right_raw = -encoder_get_count(ENCODER_QUAD_RIGHT);

    encoder_clear_count(ENCODER_QUAD_LEFT);
    encoder_clear_count(ENCODER_QUAD_RIGHT);

    left_acc  += (int32)left_raw;
    right_acc += (int32)right_raw;

    sample_half++;
    if(sample_half >= 2)
    {
        sample_half = 0;

        if(new_speed_sample_flag)
        {
            sample_overrun_count++;
        }

        sampled_left_count  = (int16)left_acc;
        sampled_right_count = (int16)right_acc;
        new_speed_sample_flag = 1;

        left_acc  = 0;
        right_acc = 0;
    }
}

//----------------------------------------------------------------------------------------------------------------------
// Initialize encoders in quadrature mode
//----------------------------------------------------------------------------------------------------------------------
void encoder_init(void)
{
    encoder_quad_init(ENCODER_QUAD_LEFT,  ENCODER_QUAD_LEFT_CHA,  ENCODER_QUAD_LEFT_CHB);
    encoder_quad_init(ENCODER_QUAD_RIGHT, ENCODER_QUAD_RIGHT_CHA, ENCODER_QUAD_RIGHT_CHB);
}

//----------------------------------------------------------------------------------------------------------------------
// Get left wheel encoder count (forward = positive)
//----------------------------------------------------------------------------------------------------------------------
int16 encoder_get_left(void)
{
    return left_count;
}

//----------------------------------------------------------------------------------------------------------------------
// Get right wheel encoder count (forward = positive)
//----------------------------------------------------------------------------------------------------------------------
int16 encoder_get_right(void)
{
    return right_count;
}

//----------------------------------------------------------------------------------------------------------------------
// Initialize speed measurement: start TIM4_PIT at 50ms
//----------------------------------------------------------------------------------------------------------------------
void speed_measure_init(void)
{
    left_acc  = 0;
    right_acc = 0;
    sample_half = 0;
    pit_ms_init(TIM4_PIT, 50, pit4_isr);
}

//----------------------------------------------------------------------------------------------------------------------
// Read-only getter: speed measurement sequence counter
//----------------------------------------------------------------------------------------------------------------------
uint32 speed_measure_get_sequence(void)
{
    return speed_measure_sequence;
}

//----------------------------------------------------------------------------------------------------------------------
// Non-blocking speed update: call from main loop
// Reads ISR sample, calculates m/s, updates public state
//----------------------------------------------------------------------------------------------------------------------
void speed_measure_process(void)
{
    int16 local_left, local_right;

    if(new_speed_sample_flag)
    {
        EA = 0;
        local_left  = sampled_left_count;
        local_right = sampled_right_count;
        new_speed_sample_flag = 0;
        EA = 1;

        left_count  = local_left;
        right_count = local_right;

        left_speed_mps  = (float)left_count  / LEFT_COUNT_PER_METER  / SPEED_SAMPLE_PERIOD_S;
        right_speed_mps = (float)right_count / RIGHT_COUNT_PER_METER / SPEED_SAMPLE_PERIOD_S;

        speed_measure_sequence++;
    }
}
