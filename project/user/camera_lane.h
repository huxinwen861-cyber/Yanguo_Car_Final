/*********************************************************************************************************************
* Camera Lane V1 - frozen module
*   Faithfully extracted from main_backup_before_servo_direction_test.c
*   (the single authoritative Camera Lane V1 baseline).
*
*   Contains only the verified Camera Lane V1 pipeline:
*     fixed-threshold binarization
*     left / right edge extraction
*     center_line / center_valid
*     LOOKAHEAD_ROW lookahead steering error (steering_error / error_valid)
*     edge / center / lookahead visualization
*     UART8 low-frequency diagnostic error output
*
*   NO servo / steering-actuator control code is present in this module.
*   Do not add servo control here.
********************************************************************************************************************/
#ifndef __CAMERA_LANE_H
#define __CAMERA_LANE_H

#include "zf_common_headfile.h"

//----------------------------------------------------------------------------------------------------------------------
// Lane extraction constants (identical to the frozen baseline values)
//----------------------------------------------------------------------------------------------------------------------
#define IMAGE_THRESHOLD         (128)
#define EDGE_SEARCH_TOP         (50)
#define EDGE_SEARCH_RANGE       (20)
#define IMAGE_CENTER_X          (MT9V03X_W / 2)
#define CENTER_LINE_GRAY        (160)
#define LOOKAHEAD_ROW           (90)
#define LOOKAHEAD_CENTER_GRAY   (210)
#define LOOKAHEAD_TRACK_GRAY    (80)
#define LOOKAHEAD_LINE_GRAY     (180)
#define FIRMWARE_MARK_GRAY      (128)

//----------------------------------------------------------------------------------------------------------------------
// UART8 diagnostic output (wireless), one line every CAMERA_LANE_UART_PERIOD frames
//----------------------------------------------------------------------------------------------------------------------
#define CAMERA_LANE_UART        (UART_8)
#define CAMERA_LANE_UART_PERIOD (30)

//----------------------------------------------------------------------------------------------------------------------
// Frame buffer sent to Seekfree Assistant (raw + overlays)
//----------------------------------------------------------------------------------------------------------------------
extern uint8 far image_copy[MT9V03X_H][MT9V03X_W];

//----------------------------------------------------------------------------------------------------------------------
// Per-row edge / center results
//----------------------------------------------------------------------------------------------------------------------
extern uint16 left_edge[MT9V03X_H];
extern uint16 right_edge[MT9V03X_H];
extern uint8  left_valid[MT9V03X_H];
extern uint8  right_valid[MT9V03X_H];
extern uint16 center_line[MT9V03X_H];
extern uint8  center_valid[MT9V03X_H];

//----------------------------------------------------------------------------------------------------------------------
// Lookahead steering error at LOOKAHEAD_ROW
//----------------------------------------------------------------------------------------------------------------------
extern int16  steering_error;
extern uint8  error_valid;

//----------------------------------------------------------------------------------------------------------------------
// Init: Seekfree Assistant camera config + UART8 (wireless). Call after camera init.
//----------------------------------------------------------------------------------------------------------------------
void camera_lane_init(void);

//----------------------------------------------------------------------------------------------------------------------
// Process one captured frame. Call once while mt9v03x_finish_flag is set.
//----------------------------------------------------------------------------------------------------------------------
void camera_lane_process_frame(void);

#endif
