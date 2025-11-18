/******************************************************************************
*
* Freescale Semiconductor Inc.
* (c) Copyright 2008 Freescale Semiconductor, Inc.
* ALL RIGHTS RESERVED.
* SPDX-License-Identifier: BSD-3-Clause
*
***************************************************************************//*!
*
* @file      state_machine.h
*
* @author    r63172
* 
* @version   1.0.1.0
* 
* @date      Jul-26-2012
* 
* @brief     State machine
*
*******************************************************************************
*
* State machine.
*
******************************************************************************/

#ifndef _STATE_MACHINE_PFC_H_
#define _STATE_MACHINE_PFC_H_
/******************************************************************************
* Includes
******************************************************************************/

/******************************************************************************
* Constants
******************************************************************************/
#ifndef true
#define true  ((bool)1)
#endif

#ifndef false
#define false ((bool)0)
#endif

/* Application state identification enum */
typedef enum {
    FAULT_PFC           = 0,
    INIT_PFC            = 1,
    STOP_PFC           	= 2,
	RUN_PFC				= 3
} SM_APP_STATE_PFC_T;         

typedef unsigned short SM_APP_CTRL_PFC;
typedef unsigned long SM_APP_FAULT_PFC;

typedef void (*PFCN_VOID_VOID_PFC)(void); /* pointer to function */

/* User state machine functions structure */
typedef struct
{
	PFCN_VOID_VOID_PFC	Fault;
	PFCN_VOID_VOID_PFC	Init;
	PFCN_VOID_VOID_PFC	Stop;
	PFCN_VOID_VOID_PFC	Run;
} SM_APP_STATE_FCN_PFC_T;

/* User state-transition functions structure*/
typedef struct
{
	PFCN_VOID_VOID_PFC	FaultInit;
	PFCN_VOID_VOID_PFC	InitFault;
	PFCN_VOID_VOID_PFC	InitStop;
	PFCN_VOID_VOID_PFC	StopFault;
	PFCN_VOID_VOID_PFC	StopRun;
	PFCN_VOID_VOID_PFC	RunFault;
	PFCN_VOID_VOID_PFC	RunStop;
} SM_APP_TRANS_FCN_PFC_T;

/* State machine control structure */
typedef struct
{
    SM_APP_STATE_FCN_PFC_T const*	psState;			/* State functions */
    SM_APP_TRANS_FCN_PFC_T const* 	psTrans; 			/* Transition functions */
    SM_APP_CTRL_PFC                 uiCtrl;				/* Control flags */
    SM_APP_STATE_PFC_T              eState;				/* State */
} SM_APP_CTRL_PFC_T;

/* pointer to function with a pointer to state machine control structure */
typedef void (*PFCN_VOID_PSM_PFC)(SM_APP_CTRL_PFC_T *sAppCtrl); 


/* State machine control command flags */
#define SM_CTRL_PFC_NONE		0x0
#define SM_CTRL_PFC_FAULT		0x1
#define SM_CTRL_PFC_FAULT_CLEAR	0x2
#define SM_CTRL_PFC_INIT_DONE	0x4
#define SM_CTRL_PFC_STOP		0x8
#define SM_CTRL_PFC_START		0x10
#define SM_CTRL_PFC_STOP_ACK	0x20
#define SM_CTRL_PFC_RUN_ACK		0x40

/* State machine function table (in pmem) */
extern const PFCN_VOID_PSM_PFC gSM_STATE_TABLE[4];

/* State machine function */
static inline void SM_StateMachine(SM_APP_CTRL_PFC_T *sAppCtrl)
{
	gSM_STATE_TABLE[sAppCtrl->eState](sAppCtrl);
}

#endif //_STATE_MACHINE_H_
