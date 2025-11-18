/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef PFC_STRUCTURE_H_
#define PFC_STRUCTURE_H_

#include "gflib_FP.h"
#include "gdflib_FP.h"
#include "mlib.h"

/******************************************************************************
* Types
******************************************************************************/


typedef struct
{
    GFLIB_CTRL_PI_P_AW_T_FLT        sIPiParams;     	/* Current PI controller parameters */
    GDFLIB_FILTER_MA_T_FLT          sIOffsetFilter;     /* Current offset filter */
    frac16_t              		    f16Duty;			/* Applied duty cycle */
    float_t              		    fltDuty;			/* Applied duty cycle */
    float_t              		    fltDutyIout;			/* Applied duty cycle */
    frac16_t                        f16IFdbck;
    float_t                        fltIFdbck; 	       	/* Real current on the MOS */
    float_t                        fltICorrect;        /* Correct current(actual average current) on the MOS */
    float_t						fltIRef;  		    /* Required current reference */
    float_t						fltIError;			/* Current error */
    float_t						fltCompCoeff;       /* Current compensation for DCM */
    bool_t                          bStopIntegFlag;     /* PI controller integration stop flag */ 
} PFCSTRUC_CURRENT_CTRL_T;

typedef struct
{			
	GFLIB_CTRL_PI_P_AW_T_FLT        sUDcBusPiParams;	/* Dc bus voltage PI controller parameters */
	GFLIB_RAMP_T_FLT				sUDcBusRampParams; 	/* Dc bus voltage ramp parameters */
	GDFLIB_FILTER_IIR1_T_FLT 		sUDcBusFilter;		/* Dc bus voltage filter */
	float_t						fltUDcBusReq;		/* Required DC bus voltage (ramp output) */
	float_t						fltUDcBusCmd;		/* DC bus voltage command (entered by user or master layer) */
	frac16_t                    f16UDcBus;          /* Raw Sampled DC bus voltage */
    float_t						fltUDcBus;			
    
	float_t						fltUDcBusFilt;		/* Filtered DC bus voltage */
	float_t						fltUDcBusError;		/* DC bus voltage error */
	float_t                      fltUDcBusErrLim;    /* DC bus voltage error limit */
	float_t						fltVoltDiff_UDc_UIn;/* Voltage difference between DC bus voltage and input AC voltage */

	float_t						fltUDcBusCtrlOut;   /* DC bus voltage PI controller output */
    frac16_t                    f16UDcBusCtrlOut;
	float_t 						fltIRef;  		    /* Required current reference */
	float_t                        fltUDcBusRelayOn;   /* DC bus voltage threshold to turn on the relay */
	float_t                        fltUDcBusRelayOff;  /* DC bus voltage threshold to turn off the relay */
	float_t                        fltUDcBusBurstOn;   /* Start PFC operation when output voltage is less than this value in light-load mode */
	float_t                        fltUDcBusBurstOff;  /* Stop PFC operation when output voltage is larger than this value in light-load mode */
	bool_t                          bStopIntegFlag;     /* PI controller integration stop flag */ 
} PFCSTRUC_VOLTAGE_CTRL_T;

typedef struct
{				
    float_t                         fltSine;                    /* Generated sine wave based on input voltage phase */ 
	frac16_t						f16Phase; 					/* Electrical phase of the voltage */ 
	frac16_t                        f16Phase_refine;
	uint16_t                        u16FallOkCnt;               /* A counter that increases when input voltage is below threshold when the voltage is falling */
	uint16_t                        u16RiseOkCnt;               /* A counter that increases when input voltage is above threshold when the voltage is rising */
	uint16_t						u16CntThrsd2Zero;		    /* A counter that records the time when input voltage falls from threshold to zero */
	uint16_t                        u16TimeCnt;			        /* A counter to record the elapsed time within an input voltage period */
	uint16_t						u16Period;					/* Input voltage period duration, which is derived from u16TimeCnt */
	uint16_t						u16PeriodFilt;				/* Filtered period duration */
	uint16_t                        u16Step; 
	frac16_t					    f16Freq;					/* Reciprocal of filtered period,Q1.15 format,which represents the value 1 divided by filtered period */
	GDFLIB_FILTER_MA_T_A32          sPeriodFilter;              /* A filter for the period */
	uint16_t                        u16PosCnt;
	uint16_t                        u16NegCnt;
	uint8_t                         u8Polar;
	uint32_t                        u16PLLSum;
	frac16_t                        f16Err;
	uint16_t                        u16Cnt;
	float_t						fltUInThreshold;			/* Rectified voltage threshold for phase detect, which is a value near zero */
	bool_t							bFallThrsdDetEn;			/* A control switch to enable searching for threshold when input voltage is falling */
	bool_t							bRiseThrsdDetEn;			/* A control switch to enable searching for threshold when input voltage is rising */
} PFCSTRUC_PHASE_DETECT_T;

typedef struct
{
	GDFLIB_FILTER_IIR1_T_FLT        sFilter;     			/* Input voltage filter */
	float_t                         fltUIn;					/* Input voltage */
        frac16_t                        f16UIn;					/* Input voltage */

        frac16_t                        f16Imos1;					/* imos1 */
        float_t                         fltImos1;					/* imos */
        frac16_t                        f16Offset1;					/* imos1 offset */        
 
        frac16_t                        f16Imos2;					/* imos2 */
        float_t                         fltImos2;					/* imos */
        frac16_t                        f16Offset2;					/* imos2 offset */   

        float_t                         fltUdcb;					/* bus voltage */
        frac16_t                        f16Udcb;					/* bus voltage */  
	float_t                         fltUInFilt;             /* Filtered input voltage */
	float_t                         fltUInFiltRaw;
	float_t                         fltLastUInFilt;           /* Filtered input voltage of last step */
	float_t                         fltUInMaxTemp;          /* Temporary max value of input during peak value detection */
	float_t                         fltUInMax;              /* Final detected input voltage max value */

	float_t                         fltUInMaxSquare;        /* Square of max input voltage */
	float_t                         fltUInMaxSquInv;        /* Reciprocal of the square of max input voltage */
	float_t                         fltUInMaxInv;           /* Reciprocal of max input voltage */
	
	uint16_t                        u16UpCnt;
	uint16_t                        u16DownCnt;
	uint16_t                        u16VoltagePeakCnt;		/* A counter recording how many times peak voltage has been detected.
	                                                           It stops increasing after it reaches certain value. */
	bool_t                          bSeveralPeaksDetectedFlag;        /* This flag will set after peak voltage has been detected for certain times */  
	bool_t                          bPeakDetectEn;      	/* A control switch that decides whether to detect the peak voltage or not */
	bool_t                          bPeakFlag;              /* A flag indicating that a peak value has been detected */
} PFCSTRUC_INPUT_VOLTAGE_T;

typedef struct
{
	uint16_t             RelayFlag:1;
	uint16_t             BrakeFlag:1;
	uint16_t             PWM_enable:1;
	uint16_t             DCMFlag:1;
	uint16_t             Reserved:11;
} PFCSTRUC_FLAG_T;

typedef union
{
    uint16_t R;
    struct
    {
    	uint16_t UDcBusOver            	: 1;   /* DC bus over voltage */
    	uint16_t UDcBusUnder            : 1;   /* DC bus under voltage */
    	uint16_t UInOver       		: 1;   /* AC input over voltage */
    	uint16_t UInUnder      		: 1;   /* AC input under voltage */
    	uint16_t IPh1Over       	: 1;   /* Phase 1 PFC over current */
    	uint16_t UInFreqOver            : 1;   /* AC input over frequency */
    	uint16_t UInFreqUnder           : 1;   /* AC input under frequency */
        uint16_t HW_UDcBusOver		: 1;   /* DC bus over voltage - HW protection */
        uint16_t HW_IOver		: 1;   /* Phase 1or2 over current - HW protection */
    } B;
} PFCSTRUC_FAULT_STATUS_T;    /* Application fault status user type*/

typedef struct
{
	float_t					fltUDcBusOver;		/* DC bus over voltage level */
	float_t					fltUDcBusUnder;		/* DC bus under voltage level */
	float_t					fltUInOver;			/* Input over voltage level */
	float_t					fltUInUnder;		/* Input under voltage level */
	frac16_t					f16FreqUInOver;		/* Input voltage over frequency  level */
	frac16_t					f16FreqUInUnder;	/* Input voltage under frequency level */
	float_t                  fltIInOver;         /* Input over current level */
} PFCDEF_FAULT_THRESHOLDS_T;

typedef struct
{	
	PFCDEF_FAULT_THRESHOLDS_T	sFaultThresholds;
	PFCSTRUC_CURRENT_CTRL_T		sICtrlPh1;
	PFCSTRUC_CURRENT_CTRL_T		sICtrlPh2;
	PFCSTRUC_VOLTAGE_CTRL_T		sUCtrl;
	PFCSTRUC_PHASE_DETECT_T		sPhaseDetector;
	PFCSTRUC_INPUT_VOLTAGE_T    sUInPeakDetection;
	PFCSTRUC_FLAG_T             sFlag;

	float_t                     fltDutyCCM;
	float_t                     fltDutyDCM;
	float_t                     fltDutyComp;
	PFCSTRUC_FAULT_STATUS_T 	FaultId; 	    	   /* Fault identification */ 
	PFCSTRUC_FAULT_STATUS_T 	FaultIdPending;    	   /* Fault identification pending*/ 
	uint16_t 					u16CounterTimeBase;   /* A counter to obtain a time base, such as 1ms */   

	uint16_t 					ui16CounterState;      /* A counter to time the duration of each necessary state */

} PFCDEF_DRIVE_T;


#endif /* PFC_STRUCTURE_H_ */
