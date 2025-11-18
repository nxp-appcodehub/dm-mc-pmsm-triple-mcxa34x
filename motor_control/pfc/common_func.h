
/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef COMMON_FUNC_H_
#define COMMON_FUNC_H_

#include "PFC_statemachine.h"
#include "Peripherals.h"
#include "mlib_FP.h"
#include "gdflib.h"
#include "fsl_ctimer.h"

extern uint16_t sinetable[1024];



#define  PLL_K    3
#define  MAX_SIN_TABLE_SIZE    1024
/* PFC_Phase_detect is called in current control loop of PFC when compilation condition "PLL" is not zero*/
//mark, need to calculation the float/frac unit 
inline static void PFC_Phase_detect2(PFCDEF_DRIVE_T *ptr)
{	
    ptr->sPhaseDetector.u16TimeCnt++;
    
    // Voltage zero-crossing detection when input AC voltage is indeed falling down
    if(ptr->sUInPeakDetection.fltUInFiltRaw > 0)
    {
        if(ptr->sPhaseDetector.u8Polar == 0 && ptr->sPhaseDetector.u16NegCnt > 100 )
        {
            ptr->sPhaseDetector.u16PosCnt = 0; 
            ptr->sPhaseDetector.u16NegCnt=0;
            ptr->sPhaseDetector.u8Polar = 1;
            ptr->sPhaseDetector.u16PLLSum = 0;
            ptr->sPhaseDetector.f16Err = MAX_SIN_TABLE_SIZE - ptr->sPhaseDetector.u16Cnt;
            ptr->sPhaseDetector.u16Step += ptr->sPhaseDetector.f16Err * PLL_K;
            GFLIB_Limit_F16(ptr->sPhaseDetector.u16Step, 8389, 13005);               
        }
        ptr->sPhaseDetector.u16PosCnt ++;           
    }
    else if(ptr->sUInPeakDetection.fltUInFiltRaw < 0)  
    {
        if(ptr->sPhaseDetector.u8Polar == 1 && ptr->sPhaseDetector.u16PosCnt > 100 )
        {
            ptr->sPhaseDetector.u16PosCnt = 0; 
            ptr->sPhaseDetector.u16NegCnt=0;
            ptr->sPhaseDetector.u8Polar = 0;
            ptr->sPhaseDetector.u16PLLSum = 0;
            ptr->sPhaseDetector.f16Err = MAX_SIN_TABLE_SIZE - ptr->sPhaseDetector.u16Cnt;
            ptr->sPhaseDetector.u16Step += ptr->sPhaseDetector.f16Err * PLL_K;
            GFLIB_Limit_F16(ptr->sPhaseDetector.u16Step, 8389, 13005);                   
        }
        ptr->sPhaseDetector.u16NegCnt ++;          
    }
    ptr->sPhaseDetector.u16PLLSum += ptr->sPhaseDetector.u16Step;
    ptr->sPhaseDetector.u16Cnt = (ptr->sPhaseDetector.u16PLLSum >> 12);
    if(ptr->sPhaseDetector.u16Cnt >= MAX_SIN_TABLE_SIZE)
        ptr->sPhaseDetector.f16Phase =  (ptr->sPhaseDetector.u16Cnt - MAX_SIN_TABLE_SIZE);
    else
       ptr->sPhaseDetector.f16Phase = ( ptr->sPhaseDetector.u16Cnt);
//    ptr->sPhaseDetector.fltSine = GFLIB_Sin_FLTa((acc32_t)ptr->sPhaseDetector.f16Phase);
      ptr->sPhaseDetector.fltSine = MLIB_Conv_FLTs((*(sinetable + ptr->sPhaseDetector.f16Phase))<<3);

}


/* PFC_Phase_detect is called in current control loop of PFC when compilation condition "PLL" is not zero*/
//SOGI
inline static void PFC_Phase_detect3(PFCDEF_DRIVE_T *ptr)
{	
	ptr->sPhaseDetector.u16TimeCnt++;
	
	// Voltage zero-crossing detection when input AC voltage is indeed falling down

	
	ptr->sPhaseDetector.fltSine = GFLIB_Sin_FLTa((acc32_t)ptr->sPhaseDetector.f16Phase);

}
#endif /* COMMON_FUNC_H_ */
