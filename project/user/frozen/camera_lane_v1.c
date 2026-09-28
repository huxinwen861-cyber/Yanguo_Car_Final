/*********************************************************************************************************************
* Camera Lane V1 - frozen module implementation
*   Faithfully extracted from main_backup_before_servo_direction_test.c
*   (the single authoritative Camera Lane V1 baseline).
*
*   The verified pipeline logic below is copied verbatim from that baseline.
*   NO servo / steering-actuator control code is present. Do not add any.
********************************************************************************************************************/
#include "camera_lane.h"

//----------------------------------------------------------------------------------------------------------------------
// Module state
//----------------------------------------------------------------------------------------------------------------------
uint8 far image_copy[MT9V03X_H][MT9V03X_W];

uint16 left_edge[MT9V03X_H];
uint16 right_edge[MT9V03X_H];
uint8  left_valid[MT9V03X_H];
uint8  right_valid[MT9V03X_H];
uint16 center_line[MT9V03X_H];
uint8  center_valid[MT9V03X_H];

int16  steering_error;
uint8  error_valid;

static uint16 camera_lane_frame_count = 0;

//----------------------------------------------------------------------------------------------------------------------
// Draw a large cross (arm +-5) at row LOOKAHEAD_ROW, column cx, with boundary clamping
//----------------------------------------------------------------------------------------------------------------------
static void draw_lookahead_cross(uint16 cx, uint8 gray)
{
    uint16 k;
    uint16 x0;
    uint16 x1;
    uint16 y0;
    uint16 y1;

    /* horizontal bar */
    if(cx > 5) x0 = cx - 5;
    else       x0 = 0;
    x1 = cx + 5;
    if(x1 > (MT9V03X_W - 1)) x1 = MT9V03X_W - 1;
    for(k = x0; k <= x1; k++)
    {
        image_copy[LOOKAHEAD_ROW][k] = gray;
    }

    /* vertical bar */
    if(LOOKAHEAD_ROW > 5) y0 = LOOKAHEAD_ROW - 5;
    else                  y0 = 0;
    y1 = LOOKAHEAD_ROW + 5;
    if(y1 > (MT9V03X_H - 1)) y1 = MT9V03X_H - 1;
    for(k = y0; k <= y1; k++)
    {
        image_copy[k][cx] = gray;
    }
}

//----------------------------------------------------------------------------------------------------------------------
// Tiny string builder helpers (avoid sprintf on C251)
//----------------------------------------------------------------------------------------------------------------------
static void wu_str_cat(char *buf, uint8 *idx, const char *str)
{
    while(*str != '\0')
    {
        buf[*idx] = *str;
        (*idx)++;
        str++;
    }
}

static void wu_udec_cat(char *buf, uint8 *idx, uint16 val)
{
    char tmp[5];
    uint8 n = 0;

    if(val == 0)
    {
        buf[*idx] = '0';
        (*idx)++;
        return;
    }
    while((val > 0) && (n < 5))
    {
        tmp[n] = (char)('0' + (uint8)(val % 10));
        n++;
        val /= 10;
    }
    while(n > 0)
    {
        n--;
        buf[*idx] = tmp[n];
        (*idx)++;
    }
}

//----------------------------------------------------------------------------------------------------------------------
// Init: Seekfree Assistant camera information + UART8 (wireless)
//----------------------------------------------------------------------------------------------------------------------
void camera_lane_init(void)
{
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy[0], MT9V03X_W, MT9V03X_H);

    /* Wireless UART: UART_8, 115200, MCU TX=P9.3, RX=P9.2 (per WIRELESS_UART_* in zf_device_wireless_uart.h) */
    uart_init(CAMERA_LANE_UART, 115200, UART8_TX_P93, UART8_RX_P92);
}

//----------------------------------------------------------------------------------------------------------------------
// Process one captured frame (verified Camera Lane V1 pipeline)
//----------------------------------------------------------------------------------------------------------------------
void camera_lane_process_frame(void)
{
    uint16 row;
    uint16 col;
    uint16 x;
    uint16 lo;
    uint16 hi;
    uint16 prev_left;
    uint16 prev_right;
    char   wireless_buf[64];
    uint8  wireless_idx;

    /* 1. copy raw frame (mt9v03x_image is never modified) */
    memcpy(image_copy[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);

    /* 2. in-place fixed-threshold binarization */
    for(row = 0; row < MT9V03X_H; row++)
    {
        for(col = 0; col < MT9V03X_W; col++)
        {
            if(image_copy[row][col] > IMAGE_THRESHOLD)
            {
                image_copy[row][col] = 255;
            }
            else
            {
                image_copy[row][col] = 0;
            }
        }
    }

    /* 3. edge search: bottom-up, rows MT9V03X_H-1 .. EDGE_SEARCH_TOP */
    prev_left  = IMAGE_CENTER_X;
    prev_right = IMAGE_CENTER_X;

    row = MT9V03X_H;
    while(row > EDGE_SEARCH_TOP)
    {
        row--;

        left_valid[row]  = 0;
        right_valid[row] = 0;

        /* ---- LEFT edge: from center outward, find white -> black ---- */
        if(row == (MT9V03X_H - 1))
        {
            lo = 0;
            hi = IMAGE_CENTER_X;
        }
        else
        {
            if(prev_left > EDGE_SEARCH_RANGE) lo = prev_left - EDGE_SEARCH_RANGE;
            else                              lo = 0;
            hi = prev_left + EDGE_SEARCH_RANGE;
            if(hi > (MT9V03X_W - 1)) hi = MT9V03X_W - 1;
        }

        for(x = hi; x > lo; x--)
        {
            if((image_copy[row][x] == 255) && (image_copy[row][x - 1] == 0))
            {
                left_edge[row]  = x;
                left_valid[row] = 1;
                break;
            }
        }
        if(left_valid[row])
        {
            prev_left = left_edge[row];
        }

        /* ---- RIGHT edge: from center outward, find white -> black ---- */
        if(row == (MT9V03X_H - 1))
        {
            lo = IMAGE_CENTER_X;
            hi = MT9V03X_W - 1;
        }
        else
        {
            if(prev_right > EDGE_SEARCH_RANGE) lo = prev_right - EDGE_SEARCH_RANGE;
            else                               lo = 0;
            hi = prev_right + EDGE_SEARCH_RANGE;
            if(hi > (MT9V03X_W - 1)) hi = MT9V03X_W - 1;
        }

        for(x = lo; x < hi; x++)
        {
            if((image_copy[row][x] == 255) && (image_copy[row][x + 1] == 0))
            {
                right_edge[row]  = x;
                right_valid[row] = 1;
                break;
            }
        }
        if(right_valid[row])
        {
            prev_right = right_edge[row];
        }
    }

    /* 4. compute center line (only when both edges valid) */
    for(row = EDGE_SEARCH_TOP; row < MT9V03X_H; row++)
    {
        if(left_valid[row] && right_valid[row])
        {
            center_line[row]  = (left_edge[row] + right_edge[row]) / 2;
            center_valid[row] = 1;
        }
        else
        {
            center_valid[row] = 0;
        }
    }

    /* 4b. lookahead steering error (cast each operand to int16 before subtract) */
    error_valid = 0;
    if(center_valid[LOOKAHEAD_ROW])
    {
        steering_error = (int16)center_line[LOOKAHEAD_ROW] - (int16)IMAGE_CENTER_X;
        error_valid    = 1;
    }

    /* 5. draw detected edges as gray (128) into image_copy */
    for(row = EDGE_SEARCH_TOP; row < MT9V03X_H; row++)
    {
        if(left_valid[row])
        {
            x = left_edge[row];
            image_copy[row][x] = 128;
            if(x > 0)               image_copy[row][x - 1] = 128;
            if(x < (MT9V03X_W - 1)) image_copy[row][x + 1] = 128;
        }
        if(right_valid[row])
        {
            x = right_edge[row];
            image_copy[row][x] = 128;
            if(x > 0)               image_copy[row][x - 1] = 128;
            if(x < (MT9V03X_W - 1)) image_copy[row][x + 1] = 128;
        }
        if(center_valid[row])
        {
            x = center_line[row];
            image_copy[row][x] = CENTER_LINE_GRAY;
            if(x > 0)               image_copy[row][x - 1] = CENTER_LINE_GRAY;
            if(x > 1)               image_copy[row][x - 2] = CENTER_LINE_GRAY;
            if(x < (MT9V03X_W - 1)) image_copy[row][x + 1] = CENTER_LINE_GRAY;
            if(x < (MT9V03X_W - 2)) image_copy[row][x + 2] = CENTER_LINE_GRAY;
        }
    }

    /* 6. lookahead visualization */
    /* 6a. dashed horizontal reference line at LOOKAHEAD_ROW (gray 180) */
    for(x = 0; x < MT9V03X_W; x += 4)
    {
        image_copy[LOOKAHEAD_ROW][x] = LOOKAHEAD_LINE_GRAY;
    }

    /* 6b. fixed image-center large cross (gray 210) */
    draw_lookahead_cross(IMAGE_CENTER_X, LOOKAHEAD_CENTER_GRAY);

    /* 6c. lookahead track large cross (gray 80), only when error valid */
    if(error_valid)
    {
        draw_lookahead_cross(center_line[LOOKAHEAD_ROW], LOOKAHEAD_TRACK_GRAY);
    }

    /* 7. low-frequency diagnostic output on UART8 (wireless), not USB CDC */
    camera_lane_frame_count++;
    if((camera_lane_frame_count % CAMERA_LANE_UART_PERIOD) == 0)
    {
        if(error_valid)
        {
            wireless_idx = 0;
            wu_str_cat(wireless_buf, &wireless_idx, "LOOKAHEAD row=90 center=");
            wu_udec_cat(wireless_buf, &wireless_idx, center_line[LOOKAHEAD_ROW]);
            if(steering_error < 0)
            {
                wu_str_cat(wireless_buf, &wireless_idx, " error=-");
                wu_udec_cat(wireless_buf, &wireless_idx, (uint16)(-(int)steering_error));
            }
            else
            {
                wu_str_cat(wireless_buf, &wireless_idx, " error=");
                wu_udec_cat(wireless_buf, &wireless_idx, (uint16)steering_error);
            }
            wu_str_cat(wireless_buf, &wireless_idx, " valid=1\r\n");
            wireless_buf[wireless_idx] = '\0';
            uart_write_string(CAMERA_LANE_UART, wireless_buf);
        }
        else
        {
            uart_write_string(CAMERA_LANE_UART, "LOOKAHEAD row=90 INVALID valid=0\r\n");
        }
    }

    /* 8. firmware version mark: unconditional 20x20 gray block (rows 5..24, cols 5..24) */
    for(row = 5; row <= 24; row++)
    {
        for(col = 5; col <= 24; col++)
        {
            image_copy[row][col] = FIRMWARE_MARK_GRAY;
        }
    }

    /* 9. send to Seekfree Assistant */
    seekfree_assistant_camera_send();
}
