/*
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _FM_TSA_PMSM_H_
#define _FM_TSA_PMSM_H_

#include "freemaster.h"
#include "freemaster_tsa.h"
#include "mc_periph_init.h"

#ifdef PMSM_SNSLESS
#include "m1_sm_snsless.h"
#ifdef DUAL_MOTOR
#include "m2_sm_snsless.h"
#endif
#else
#include "m1_sm_snsless.h"
#endif


/* global control variables */
#ifdef PMSM_SNSLESS
extern bool_t bDemoModePosition;
#endif

extern bool_t bDemoModeSpeed;

/* global used misc variables */
extern uint32_t g_ui32NumberOfCycles;
extern uint32_t g_ui32MaxNumberOfCycles;

/* Application and board ID  */
extern app_ver_t g_sAppIdFM;

/* Extern variables only for dual motor application */
#ifdef DUAL_MOTOR
extern bool_t bM2DemoModeSpeed;
extern uint32_t g_ui32M2NumberOfCycles;
extern uint32_t g_ui32M2MaxNumberOfCycles;
#ifdef PMSM_SNSLESS
extern bool_t bM2DemoModePosition;
#endif
#endif

#endif
