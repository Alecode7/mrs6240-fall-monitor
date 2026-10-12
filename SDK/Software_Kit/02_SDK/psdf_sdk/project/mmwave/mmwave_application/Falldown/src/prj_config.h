/**
 **************************************************************************************************
 * @brief   project config define.
 * @attention
 *
 * Copyright (C) 2025 POSSUMIC TECHNOLOGY CO., LTD. All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the
 *       distribution.
 *    3. Neither the name of POSSUMIC TECHNOLOGY CO., LTD. nor the names of
 *       its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************************************
 */


#ifndef _PRJ_CONFIG_H
#define _PRJ_CONFIG_H

#define CONFIG_RADAR_FRAMEWORK_1D_FRAME_EN						0  // 关闭 1D 数据上传
#define CONFIG_RADAR_FRAMEWORK_2D_FRAME_EN						1
/* calculate and motion and presence point cloud */
#define CONFIG_RADAR_FRAMEWORK_REPORT_2D_DATACUBE               0  // 不上传 1D/2D 大体积中间数据；
#define CONFIG_RADAR_FRAMEWORK_INTERVAL0_RFFT_EN                0   
#define CONFIG_MMW_MOTION_POINT_CLOUD						    1 
#define CONFIG_MMW_PRESENCE_POINT_CLOUD					  	    1  // 只计算并上报 motion point cloud 和 presence point cloud；
/* when use point cloud only, auto gain is recommanded to be enable */
#define CONFIG_MMW_FFT_AUTOGAIN_EN								1  // 开启 FFT 自动增益；

#define CONFIG_MMW_DRIVER_TEMP_MGMT                             0
#define CONFIG_MMW_LOOP_TASK_MONITOR                            0
#define CONFIG_MMW_WATCHDOG                                     0

#define CONFIG_RADAR_FRAMEWORK_FT								1  // 开启 产测FT/Factory Test 相关功能。

/* Match radar_analysis control path: UART shell accepts "mmwc ..." commands. */
#define CONFIG_SHELL                                            1
#define CONFIG_SHELL_UART_NUM                                   0
#define CONFIG_MMW_SHELL                                        1
#define CONFIG_SHELL_CMD_VERSION                                1
#define CONFIG_SHELL_CMD_HEAP                                   1
#define CONFIG_SHELL_CMD_MEM                                    1
#define CONFIG_SHELL_CMD_CLK                                    1
#define CONFIG_SHELL_CMD_KERNEL                                 1
/* Required by subsys/mmw/mmw_cmd/src/mmw_report.c when mmwc report commands are enabled. */
#define CONFIG_HIF_APP_DATA_POOL                                1

/* For more details, please refer to 'radar_framework_default_config.h' */

/*
 * Individual configurations for frame config and application configure for each project
 * */

/* RS6240 Support 57G~66G, RS6130 Support 58G~64G */
#if CONFIG_BOARD_MRS6130_P1806 || CONFIG_BOARD_MRS6130_P1812
#define RADAR_FRAMEWORK_SAMPLE_START_FREQ_MHZ					(58000)
#define RADAR_FRAMEWORK_SAMPLE_MIMO_MODE						(MMW_MIMO_1T3R)
#elif CONFIG_BOARD_MRS6240_P2512_CPUF || CONFIG_BOARD_MRS6240_P2512_CPUS || CONFIG_BOARD_MRS6241_P2828_M62_CPUF || CONFIG_BOARD_MRS6241_P2840_M81_CPUF || CONFIG_BOARD_MRS7241_P2840_CPUF || CONFIG_BOARD_MRS7241_P2828_CPUF
#define RADAR_FRAMEWORK_SAMPLE_START_FREQ_MHZ					(57000)
#define RADAR_FRAMEWORK_SAMPLE_MIMO_MODE						(MMW_MIMO_2T4R)
#else
	#error "Please Slect Proper SOC!"
#endif

/*
 * The frame type used can be set as 0/1.
 * When configured as 0,it indicates the use of 1D frame mode,only one interval is sent for each frame.
 * When configured as 1,it indicates the use of 2D frame mode,based on the doppler points,multiple intervals are sent in each frame.
 *  帧类型
 */
#define RADAR_FRAMEWORK_SAMPLE_FRAME_TYPE						(1)
/* Range fft length = 1 << RADAR_FRAMEWORK_SAMPLE_RFFT_LEN_LOG2 */
#define RADAR_FRAMEWORK_SAMPLE_RFFT_LEN_LOG2					(8) // FFT 点数 1 << 8 = 256
#define RADAR_FRAMEWORK_SAMPLE_RANGE_RESOLUTION_MM				(20) // 改 距离分辨率为2 cm

/*
 * Doppler fft length = 1<<RADAR_FRAMEWORK_SAMPLE_DFFT_LEN_LOG2,
 * Doppler measurement range is [-Doppler fft length / 2 * RADAR_FRAMEWORK_SAMPLE_VEL_RESOLUTION_MM, Doppler fft length / 2 * RADAR_FRAMEWORK_SAMPLE_VEL_RESOLUTION_MM)
 */
#define RADAR_FRAMEWORK_SAMPLE_DFFT_LEN_LOG2					(4)  // 1 << 4 = 16	左移 -8  +8
#define RADAR_FRAMEWORK_SAMPLE_VEL_RESOLUTION_MM				(70)  // 速度分辨率 改 7 cm

#if CONFIG_BOARD_MRS6130_P1812 || CONFIG_BOARD_MRS6240_P2512_CPUF || CONFIG_BOARD_MRS6241_P2828_M62_CPUF || CONFIG_BOARD_MRS6241_P2840_M81_CPUF
#define RADAR_FRAMEWORK_SAMPLE_FRAME_PERIOD_MS					(59)
#else
	#error "Falldown Sample only support MRS6130_P1812, MRS6240_P2512 and MRS6240_P2828_M62!"
#endif
// 最大点数额外定义，如果不在此处定义，则需要在radar_framework_default_config.h 122行中定义
#define CONFIG_RADAR_FRAMEWORK_MAX_RFFT_LEN						(512)

/* Acc number in one interval is 1<<RADAR_FRAMEWORK_SAMPLE_ACC_NUM_LOG2 */
#define RADAR_FRAMEWORK_SAMPLE_ACC_NUM_LOG2						(0)

/* Clutter halt method is keep udate*/
#define RADAR_FRAMEWORK_SAMPLE_CLUTTER_HALT_METHOD				(CLUTTER_KEEP_UPDATE)

/* Falldown project default param define */
#define CONFIG_MMW_POINT_CLOUD_BB_CLUTTER_RM 			(MMW_CLUTTER_REMOVAL_NONE)

#define	CONFIG_MMW_MOTION_CFAR_TH_LIN					(31.6228f) /* default 15dB, 15dB = 10log10(31.6228) */

#define CONFIG_MMW_STATIC_RM_EN							(MMW_DISABLE) /* Disable static removal, use static point to interp */

#define CONFIG_MMW_LOW_SPEED_FILTER_EN					(MMW_ENABLE)

/* Falldown installation and fixed-capacity configuration. */
#define FALL_MOUNT_HEIGHT_CM                    310
#define FALL_MOUNT_PITCH_DEG                    90
#define FALL_MOUNT_HEIGHT_MIN_CM                100
#define FALL_MOUNT_HEIGHT_MAX_CM                500
#define FALL_MOUNT_CONFIG_KVF_ID                0x6F01
#define FALL_MOUNT_CONFIG_MAGIC                 0x46414C4CU

#define FALL_MAX_POINTS                         192
#define FALL_MAX_CLUSTERS                       32
#define FALL_MIN_SNR_DB                         15

#define FALL_ROI_X_MIN_CM                       (-250)
#define FALL_ROI_X_MAX_CM                       250
#define FALL_ROI_Y_MIN_CM                       (-250)
#define FALL_ROI_Y_MAX_CM                       250
#define FALL_ROI_Z_MIN_CM                       0
#define FALL_ROI_Z_MAX_CM                       220

/* Falldown clustering and single-target tracking configuration. */
#define FALL_CLUSTER_RADIUS_CM                  45
#define FALL_MIN_CLUSTER_POINTS                 3
#define FALL_ACQUIRE_MIN_POINTS                 12
#define FALL_MAX_CLUSTER_WIDTH_CM               50
#define FALL_MAX_CLUSTER_DEPTH_CM               50
#define FALL_TRACK_GATE_CM                      50
#define FALL_LATERAL_TRACK_GATE_CM              85
#define FALL_LATERAL_TRACK_HEIGHT_CM            110
#define FALL_HOLD_MS                            3000
#define FALL_SWITCH_CONFIRM_FRAMES              5
#define FALL_STATIC_BODY_MIN_POINTS             20

/* Falldown event configuration, aligned with the Python replay baseline. */
#define FALL_DROP_WINDOW_MS                     3500
#define FALL_CONFIRM_WINDOW_MS                  7000
#define FALL_MIN_DROP_CM                        80
#define FALL_MOTION_VELOCITY_TRIGGER_CM_S       45
#define FALL_MOTION_POINT_THRESHOLD             20
#define FALL_CANDIDATE_LOW_HEIGHT_CM            55
#define FALL_TERMINAL_HEIGHT_CM                 40
#define FALL_SUSPECT_LOW_DURATION_MS            700
#define FALL_SUSPECT_MIN_FRESH_SAMPLES          3
#define FALL_SUSPECT_LOW_RATIO_PERCENT          92
#define FALL_CONFIRM_LOW_DURATION_MS            1200
#define FALL_CONFIRM_MIN_FRESH_SAMPLES          6
#define FALL_CONFIRM_LOW_RATIO_PERCENT          70
#define FALL_CONFIRM_MIN_HORIZONTAL_DISPLACEMENT_CM 30
#define FALL_FRESH_MIN_POINT_COUNT              3
#define FALL_STILL_VELOCITY_CM_S                18
#define FALL_STILL_MOTION_POINTS_MAX            1
#define FALL_REARM_HEIGHT_CM                     85
#define FALL_REARM_DURATION_MS                   1000
#define FALL_REARM_TRACK_GATE_CM                 80
#define FALL_REARM_ORIGIN_GATE_CM                100
#define FALL_CANDIDATE_HOLD_TOLERANCE_MS         800

#define FALL_HISTORY_MAX_SAMPLES                192
#define FALL_EVENT_BATCH_MAX                    2
#define FALL_STATUS_PERIOD_FRAMES               10

#if (FALL_MOUNT_PITCH_DEG < 0) || (FALL_MOUNT_PITCH_DEG > 90)
#error "FALL_MOUNT_PITCH_DEG must be within 0..90 degrees"
#endif

#if (FALL_ROI_X_MIN_CM >= FALL_ROI_X_MAX_CM) || \
    (FALL_ROI_Y_MIN_CM >= FALL_ROI_Y_MAX_CM) || \
    (FALL_ROI_Z_MIN_CM >= FALL_ROI_Z_MAX_CM)
#error "Falldown ROI minimums must be less than maximums"
#endif

#if (FALL_CLUSTER_RADIUS_CM <= 0) || \
    (FALL_MAX_CLUSTER_WIDTH_CM <= 0) || \
    (FALL_MAX_CLUSTER_DEPTH_CM <= 0) || \
    (FALL_TRACK_GATE_CM <= 0) || \
    (FALL_LATERAL_TRACK_GATE_CM < FALL_TRACK_GATE_CM) || \
    (FALL_LATERAL_TRACK_HEIGHT_CM <= 0)
#error "Falldown clustering geometry must be positive"
#endif

#if (FALL_TERMINAL_HEIGHT_CM > FALL_CANDIDATE_LOW_HEIGHT_CM)
#error "Terminal height must not exceed candidate-low height"
#endif

#if (FALL_REARM_HEIGHT_CM <= FALL_TERMINAL_HEIGHT_CM)
#error "Rearm height must exceed terminal height"
#endif

#if (FALL_REARM_TRACK_GATE_CM <= 0) || \
    (FALL_REARM_ORIGIN_GATE_CM < FALL_REARM_TRACK_GATE_CM)
#error "Falldown rearm gates must be positive and ordered"
#endif

#if (FALL_SUSPECT_LOW_RATIO_PERCENT > 100) || \
    (FALL_CONFIRM_LOW_RATIO_PERCENT > 100)
#error "Falldown ratios must be percentages"
#endif

#if (FALL_CONFIRM_MIN_HORIZONTAL_DISPLACEMENT_CM < 0)
#error "Confirmed-fall horizontal displacement must not be negative"
#endif

#if (FALL_HISTORY_MAX_SAMPLES < 8) || (FALL_EVENT_BATCH_MAX < 1)
#error "Falldown fixed capacities are too small"
#endif

/* Point cloud is only enabled in RADAR_FRAM	`EWORK_2DFFT_MODE */
#define CONFIG_RADAR_FRAMEWORK_FFT_MODE					(RADAR_FRAMEWORK_2DFFT_MODE)

/* HIF configuration defined in file radar_framework_default_config.h */

#include "radar_framework_default_config.h"

#endif /* _PRJ_CONFIG_H */

/*
 **************************************************************************************************
 * (C) COPYRIGHT POSSUMIC TECHNOLOGY
 * END OF FILE
 */
