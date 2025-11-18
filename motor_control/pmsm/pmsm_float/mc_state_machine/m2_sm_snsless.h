/*
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _M2_SM_REF_SOL_H_
#define _M2_SM_REF_SOL_H_

#include "sm_common.h"
#include "m2_pmsm_appconfig.h"
#include "state_machine.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define M2_SPEED_CONV_SCALE 628.06F /* Speed conversion scale */

#define M2_MCAT_SENSORLESS_CTRL 0U /* Sensorless control flag */
#define M2_MCAT_ENC_CTRL 0U        /* Position quadrature encoder control flag */

/*******************************************************************************
 * Variables
 ******************************************************************************/
extern bool_t g_bM2SwitchAppOnOff;
extern mcdef_pmsm_t g_sM2Drive;
extern sm_app_ctrl_t g_sM2Ctrl;
extern run_substate_t g_eM2StateRun;

extern volatile float g_fltM2voltageScale;
extern volatile float g_fltM2DCBvoltageScale;
extern volatile float g_fltM2currentScale;
extern volatile float g_fltM2speedScale;
extern volatile float g_fltM2speedAngularScale;
extern volatile float g_fltM2speedMechanicalScale;

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * API
 ******************************************************************************/


#ifdef __cplusplus
}
#endif

#endif /* _M2_SM_REF_SOL_H_ */
