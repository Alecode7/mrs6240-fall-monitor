/**
 ******************************************************************************
 * @file    main.c
 * @brief   main define.
 * @verbatim    null
 ******************************************************************************
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
 ******************************************************************************
 */


/* Includes.
 * ----------------------------------------------------------------------------
 */
#define LOG_MODULE                      "MAIN"
#define LOG_LEVEL                       1

/* Please include "common.h" intead of directly inlcude "prj_config.h" */
#include "common.h"
#include "mmw_ctrl.h"
#include "log.h"
#include "radar_framework.h"


/* Private typedef.
 * ----------------------------------------------------------------------------
 */
/* Private defines.
 * ----------------------------------------------------------------------------
 */
/* Private macros.
 * ----------------------------------------------------------------------------
 */
/* Private variables.
 * ----------------------------------------------------------------------------
 */
RadarConfig_t g_radar_framework_global_config = 
{
    .radar_frame_config = 
    {
        .mimo_mode = RADAR_FRAMEWORK_SAMPLE_MIMO_MODE,
        .start_freq_mhz = RADAR_FRAMEWORK_SAMPLE_START_FREQ_MHZ,
        .frame_type = RADAR_FRAMEWORK_SAMPLE_FRAME_TYPE,
        .range_fft_len_log2 = RADAR_FRAMEWORK_SAMPLE_RFFT_LEN_LOG2,
        .range_resolution_mm = RADAR_FRAMEWORK_SAMPLE_RANGE_RESOLUTION_MM,
        .dopper_fft_len_log2 = RADAR_FRAMEWORK_SAMPLE_DFFT_LEN_LOG2,
        .vel_resolution_mm = RADAR_FRAMEWORK_SAMPLE_VEL_RESOLUTION_MM,
		.frame_period_ms = RADAR_FRAMEWORK_SAMPLE_FRAME_PERIOD_MS,
		.acc_num_log2 = RADAR_FRAMEWORK_SAMPLE_ACC_NUM_LOG2,
		.fft_mode = CONFIG_RADAR_FRAMEWORK_FFT_MODE
    },
	.radar_app_config = {
		.clutter_halt_method = RADAR_FRAMEWORK_SAMPLE_CLUTTER_HALT_METHOD		
	}
};

extern int app_init_cb(void);
extern int app_deinit_cb(void);
extern int hw_init_cb(void);
extern int hw_deinit_cb(void);
extern int mmw_frame_cb(uint8_t argc, void* arg[]);

mmw_application_ops_t s_application_ops = {
	.mmw_entry_proc_cb = 0,
	.mmw_post_proc_cb  = mmw_frame_cb,
	.mmw_app_init_cb = app_init_cb,
	.mmw_app_deinit_cb = app_deinit_cb,
	.mmw_hw_init_cb = hw_init_cb,
	.mmw_hw_deinit_cb = hw_deinit_cb,
	.mmw_hif_config_handler = 0,
	.mmw_hif_report_handler = 0
};

/* Private function prototypes.
 * ----------------------------------------------------------------------------
 */
/* Exported functions.
 * ----------------------------------------------------------------------------
 */

 int main(void)
 {
	int ret = 0;
    LOG_PRINT("WELCOME to PSIC SDK MMW SAMPLE!\n");
    LOG_PRINT("-------------------------------------------\n");

	ret = radar_framework_sys_init();
	if(0 == ret) {
		OSI_MSleep(1000);			/* Wait for mmw ctrl startup process */
		/* Register essential operations into framework */
		if (radar_framework_register_callbacks(&g_radar_framework_global_config, &s_application_ops)) {
			LOG_ERR("radar_framework_register_callbacks......FAILED!\n");
		} else {
			LOG_PRINT("radar_framework_register_callbacks......SUCCESS!\n");
		}
		if (radar_framework_firmware_config()){
			LOG_ERR("radar_framework_firmware_config......FAILED!\n");
		} else {
			LOG_PRINT("radar_framework_firmware_config......SUCCESS!\n");
			/* Start the configured radar automatically for standalone deployment. */
			ret = mmw_ctrl_start();
			if (ret) {
				LOG_ERR("Radar auto-start failed: %d\n", ret);
			} else {
				LOG_PRINT("Radar auto-start success. Use mmwc stop/start for manual control.\n");
			}
		}
	} else{
		if (ret != MMW_ERR_CODE_RUNNING) {
			LOG_ERR("Sys init failed!\n");
		} else {
			LOG_PRINT("Factory mode firmware config bypassed!\n");			
		}
	}

    return 0;
 }


/*
 ******************************************************************************
 * (C) COPYRIGHT POSSUMIC TECHNOLOGY
 * END OF FILE
 */
