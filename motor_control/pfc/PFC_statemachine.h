
/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef PFC_STATEMACHINE_H_
#define PFC_STATEMACHINE_H_

#include "state_machine.h"
#include "PFC_def.h"
#include "fsl_gpio.h"
#include "gmclib.h"
#include "pin_mux.h"
/******************************************************************************
* Macros 
******************************************************************************/
//mark
#if 1
#define PFC_DISABLE_PWM_OUTPUT() {FLEXPWM0->OUTEN &= ~PWM_OUTEN_PWMA_EN(0x8);  FLEXPWM0->OUTEN &= ~PWM_OUTEN_PWMB_EN(0x8);}
#define PFC_ENABLE_PWM_OUTPUT()  {FLEXPWM0->OUTEN |= PWM_OUTEN_PWMA_EN(0x8); FLEXPWM0->OUTEN |= PWM_OUTEN_PWMB_EN(0x8);}
#else
#define PFC_DISABLE_PWM_OUTPUT() {FLEXPWM0->OUTEN &= ~PWM_OUTEN_PWMA_EN(0x8);  }
#define PFC_ENABLE_PWM_OUTPUT()  {FLEXPWM0->OUTEN |= PWM_OUTEN_PWMA_EN(0x8);}
#endif



#define PFC_OVERVOLTAGE_FAULT()  0
#define PFC_OVERCURRENT_FAULT()  (FLEXPWM0->FSTS & PWM_FSTS_FFLAG_MASK)
#define PFC_CLEAR_OVERCURRENT_FAULT();
#define PFC_CLEAR_OVERVOLTAGE_FAULT();
#define RELAY_ON();         {GPIO_PinWrite(BOARD_INITGPIOPINS_RLY_IN_GPIO, BOARD_INITGPIOPINS_RLY_IN_PIN, 1);}
#define RELAY_OFF();        {GPIO_PinWrite(BOARD_INITGPIOPINS_RLY_IN_GPIO, BOARD_INITGPIOPINS_RLY_IN_PIN, 0);}
#define BRAKE_ON();
#define BRAKE_OFF();
/******************************************************************************
* Types
******************************************************************************/
typedef enum {
    SOFTSTART          = 0,
    NORMAL             = 1,
    LIGHTLOAD          = 2
} PFC_RUN_SUBSTATE_T;         /* Run sub-states */

extern sm_app_ctrl_t  gsPFC_Ctrl;
extern PFCDEF_DRIVE_T  gsPFC_Drive;
extern const pfc_app_state_fcn mPFC_STATE_RUN_TABLE[3];
extern volatile float g_fltPFCCurrentScale, g_fltPFCVoltageScale;

typedef struct
{
	uint32_t     *pui32PwmFrac2Val2;
	uint32_t     *pui32PwmFrac3Val3;
	uint32_t     *pui32PwmFrac4Val4;
	uint32_t     *pui32PwmFrac5Val5;
} PFCDRV_PWMVAL;

/*! @brief SubState machine control structure */
typedef struct _sm_app_sub_ctrl
{
    pfc_app_state_fcn const *pSubState; /* State functions */
    PFC_RUN_SUBSTATE_T  eStateRunSub;
} sm_app_sub_ctrl_t;

extern PFCDRV_PWMVAL        gsPFC_PwmVal;
extern sm_app_sub_ctrl_t    gsPFC_SubCtrl;
extern void PFC_FaultDetection(void);
extern void PFC_PWM_UPDATE(float_t Duty1,float_t Duty2);
extern void PFC_Phase_detect(PFCDEF_DRIVE_T *ptr);
extern void PFC_UInPeak_detect(PFCDEF_DRIVE_T *ptr);
#endif /* PFC_STATEMACHINE_H_ */
