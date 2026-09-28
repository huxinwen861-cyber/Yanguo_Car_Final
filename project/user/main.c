/*********************************************************************************************************************
* STC32G144K Opensourec Library
*
* Camera Lane V1 - thin entry point (frozen module: camera_lane.c / camera_lane.h)
*   The verified Camera Lane V1 pipeline lives entirely in camera_lane.c,
*   extracted from main_backup_before_servo_direction_test.c.
*   No servo control is used here.
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "camera_lane.h"

void main(void)
{
    uint8 init_ret;

    clock_init(SYSTEM_CLOCK_96M);
    debug_init();

    printf("CAMERA LANE V1 START\r\n");

    /* Official E09_01 demo: retry while mt9v03x_init() returns non-zero (failure) */
    init_ret = mt9v03x_init();
    while(init_ret)
    {
        printf("CAMERA INIT FAILED = %u\r\n", (unsigned int)init_ret);
        system_delay_ms(100);
        init_ret = mt9v03x_init();
    }

    printf("CAMERA INIT OK\r\n");

    camera_lane_init();

    while(1)
    {
        if(mt9v03x_finish_flag)
        {
            mt9v03x_finish_flag = 0;
            camera_lane_process_frame();
        }
    }
}
