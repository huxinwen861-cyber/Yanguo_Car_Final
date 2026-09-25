#ifndef __ENCODER_H
#define __ENCODER_H

#include "zf_common_headfile.h"

#define LEFT_COUNT_PER_METER    11287.5f
#define RIGHT_COUNT_PER_METER   11437.2f

#define SPEED_SAMPLE_PERIOD_S   0.1f

void    encoder_init(void);
int16   encoder_get_left(void);
int16   encoder_get_right(void);

void    speed_measure_init(void);
void    speed_measure_process(void);
uint32  speed_measure_get_sequence(void);

extern volatile int16   left_count;
extern volatile int16   right_count;
extern volatile float   left_speed_mps;
extern volatile float   right_speed_mps;
extern volatile uint32  sample_overrun_count;

#endif
