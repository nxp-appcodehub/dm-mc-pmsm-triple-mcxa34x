/*
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* This is a generic configuration file of the motor control driver. You need to edit the file.
 * Remove this warning statement after this file is edited manually or
 * re-generate this file using MC_PMSM Config Tool component.
 */

#ifndef _MC_PERIPH_INIT_H_
#define _MC_PERIPH_INIT_H_

#include "fsl_device_registers.h"
#include "mcdrv_eflexpwm_mcxa346.h"
#include "mcdrv_adc_mcxa346.h"
#include "m1_pmsm_appconfig.h"
#include "m2_pmsm_appconfig.h"
#include "PFC_def.h"

/* macro used for TSA table */
#define PMSM_SNSLESS

#define USE_INTERNAL_OPAMPS     (true)

/* Macro used for dual motor application support */
#define DUAL_MOTOR

/******************************************************************************
 * Timing - common for motor 1 and motor 2
 ******************************************************************************/
#define MCU_CLOCK_FREQ          (180000000U)                    /* MCU core clock 150 MHz */

/******************************************************************************
 * Timing for motor 1
 ******************************************************************************/
#define M1_PWM_FREQ             (8000U)                        /* PWM frequency in Hz */
#define M1_PWM_MODULO           (MCU_CLOCK_FREQ / M1_PWM_FREQ)  /* PWM modulo = FTM_input_clock / M1_PWM_FREQ */
#define M1_PWM_DEADTIME         (1000)                           /* Output PWM deadtime value in nanoseconds */
#define M1_FOC_FREQ_VS_PWM_FREQ (1U)                            /* PWM vs. Fast control loop ratio */
#define M1_SPEED_LOOP_FREQ       (1000U)                         /* Slow control loop frequency in Hz */

/******************************************************************************
 * Timing for motor 2
 ******************************************************************************/
#define M2_PWM_FREQ (8000)         /* PWM frequency - 8kHz */
#define M2_FOC_FREQ_VS_PWM_FREQ (1) /* FOC calculation is called every n-th PWM reload */
#define M2_SPEED_LOOP_FREQ (1000)   /* Speed loop frequency */
#define M2_PWM_DEADTIME (500)       /* Output PWM deadtime value in nanoseconds */
   
#define M3_PWM_FREQ (8000)         /* PWM frequency - 8kHz */
#define M3_FOC_FREQ_VS_PWM_FREQ (1) /* FOC calculation is called every n-th PWM reload */
#define M3_SPEED_LOOP_FREQ (1000)   /* Speed loop frequency */
#define M3_PWM_DEADTIME (500)       /* Output PWM deadtime value in nanoseconds */

#define M2_FAST_LOOP_TS ((float_t)1.0 / (float_t)(M2_PWM_FREQ / M2_FOC_FREQ_VS_PWM_FREQ))
#define M2_SLOW_LOOP_TS ((float_t)1.0 / (float_t)(M2_SLOW_LOOP_FREQ))
#define M2_TIME_ONESEC_COUNT (M2_PWM_FREQ / M2_FOC_FREQ_VS_PWM_FREQ)
#define M2_FAST_LOOP_FREQ       (M2_PWM_FREQ / M2_FOC_FREQ_VS_PWM_FREQ)
#define M2_SLOW_LOOP_FREQ       (1000U)



/* PFC frequency in Hz*/
/* PFC PWM modulo = Input_clock / PFC_PWM_FREQ */
#define PFC_PWM_MODULO           (MCU_CLOCK_FREQ / PFC_PWM_FREQ / 1000)  
 /******************************************************************************
  * Output control
  ******************************************************************************/
/* DC bus braking resistor control */
#define M1_BRAKE_SET()
#define M2_BRAKE_SET()
#define M3_BRAKE_SET()
#define M1_BRAKE_CLEAR()
#define M2_BRAKE_CLEAR()
#define M3_BRAKE_CLEAR()   
/* DC bus braking threshold hysteresis */
#define M1_U_DCB_HYSTERESIS (0.05F)
#define M2_U_DCB_HYSTERESIS (0.05F)
#define M3_U_DCB_HYSTERESIS (0.05F)
/******************************************************************************
 * ADC measurement definition
 ******************************************************************************/

/******************************************************************************
 * MC driver macro definition and check - do not change this part
 ******************************************************************************/
/******************************************************************************
 * Define motor 1 ADC control functions
 ******************************************************************************/
#define M1_MCDRV_ADC_GET(par)	(MCDRV_CurrAndVoltDcBusGet(par))
#define M1_MCDRV_CURR_3PH_CHAN_ASSIGN(par)  
#define M1_MCDRV_CURR_3PH_CALIB_INIT(par) (MCDRV_Curr3Ph2ShCalibInit(par))
#define M1_MCDRV_CURR_3PH_CALIB(par) (MCDRV_Curr3Ph2ShCalib(par))
#define M1_MCDRV_CURR_3PH_CALIB_SET(par) (MCDRV_Curr3Ph2ShCalibSet(par))

/******************************************************************************
 * Define motor 2 ADC control functions
 ******************************************************************************/
#define M2_MCDRV_ADC_GET(par)	(MCDRV_CurrAndVoltDcBusGet(par))
#define M2_MCDRV_CURR_3PH_CHAN_ASSIGN(par) 
#define M2_MCDRV_CURR_3PH_CALIB_INIT(par) (MCDRV_Curr3Ph2ShCalibInit(par))
#define M2_MCDRV_CURR_3PH_CALIB(par) (MCDRV_Curr3Ph2ShCalib(par))
#define M2_MCDRV_CURR_3PH_CALIB_SET(par) (MCDRV_Curr3Ph2ShCalibSet(par))

#define M3_MCDRV_ADC_GET(par)	(MCDRV_CurrAndVoltDcBusGet_M3(par))
#define M3_MCDRV_CURR_3PH_CALIB_INIT(par) (MCDRV_Curr3Ph2ShCalibInit(par))
   
/* Over-current Fault detection number */
#define M2_FAULT_NUM            (0U)
#define M3_FAULT_NUM            (2U)
/******************************************************************************
 * Define PFC ADC control functions
 ******************************************************************************/
#define PFC_MCDRV_ADC_GET(par)	(PFC_CurrAndVoltDcBusGet(par))
#define M2_MCDRV_CURR_3PH_CALIB_INIT(par) (MCDRV_Curr3Ph2ShCalibInit(par))
#define M2_MCDRV_CURR_3PH_CALIB(par) (MCDRV_Curr3Ph2ShCalib(par))
#define M2_MCDRV_CURR_3PH_CALIB_SET(par) (MCDRV_Curr3Ph2ShCalibSet(par))
              
/******************************************************************************
 * Define motor 1 3-ph PWM control functions
 ******************************************************************************/
#define M1_MCDRV_PWM3PH_SET(par) (MCDRV_eFlexPwm3PhSet(par))
#define M1_MCDRV_PWM3PH_EN(par) (MCDRV_eFlexPwm3PhOutEn(par))
#define M1_MCDRV_PWM3PH_DIS(par) (MCDRV_eFlexPwm3PhOutDis(par))
#define M1_MCDRV_PWM3PH_FLT_GET(par) (MCDRV_eFlexPwm3PhFltGet(par))
#define M1_MCDRV_P2M3PH_FLT_TRY_CLR(par) (MCDRV_eFlexPwm3PhFltTryClr(par))

/******************************************************************************
 * Define motor 2 3-ph PWM control functions
 ******************************************************************************/
#define M2_MCDRV_PWM3PH_SET(par) (MCDRV_eFlexPwm3PhSet(par))
#define M2_MCDRV_PWM3PH_EN(par) (MCDRV_eFlexPwm3PhOutEn(par))
#define M2_MCDRV_PWM3PH_DIS(par) (MCDRV_eFlexPwm3PhOutDis(par))
#define M2_MCDRV_PWMBOTTOM_EN(par) (MCDRV_eFlexPwm3PhBottomOutEn(par))
#define M2_MCDRV_PWM3PH_FLT_GET(par) (MCDRV_eFlexPwm3PhFltGet(par))
#define M2_MCDRV_P2M3PH_FLT_TRY_CLR(par) (MCDRV_eFlexPwm3PhFltTryClr(par))
   
#define M3_MCDRV_PWM3PH_SET(par) (MCDRV_eFlexPwm3PhSet_M3(par))   
#define M3_MCDRV_PWM3PH_DIS(par) (MCDRV_eFlexPwm3PhOutDis_M3(par))   
#define M3_MCDRV_PWM3PH_EN(par) (MCDRV_eFlexPwm3PhOutEn_M3(par))
#define M3_MCDRV_PWM3PH_FLT_GET(par) (MCDRV_eFlexPwm3PhFltGet(par))

/******************************************************************************
 * Define PFC PWM control functions
 ******************************************************************************/

/******************************************************************************
 * Define FreeMASTER functions
 ******************************************************************************/
/* FreeMASTER_Recorder_0's buffer size definition */
#define FREEMASTER_REC_0_SIZE 4096*4
/******************************************************************************
 * Global typedefs
 ******************************************************************************/
/* Structure used during clocks and modulo calculations */
typedef struct _clock_setup
{
    /* Common variables for motor 1 and motor 2 */
    uint32_t ui32FastPeripheralClock;
    uint32_t ui32CpuFrequency;
    uint32_t ui32BusClock;
    uint32_t ui32SysPllClock;

    /* Variables for motor 1 */
    uint16_t ui16M1SpeedLoopFreq;
    uint16_t ui16M1SpeedLoopModulo;
    uint16_t ui16M1PwmFreq;
    uint16_t ui16M1PwmModulo;
    uint16_t ui16M1PwmDeadTime;

    /* Variables for motor 2 */
    uint16_t ui16M2SpeedLoopModulo;
    uint16_t ui16M2PwmFreq;
    uint16_t ui16M2PwmModulo;
    uint16_t ui16M2PwmDeadTime;
    
    /* Variables for motor 3 */
    uint16_t ui16M3SpeedLoopModulo;
    uint16_t ui16M3PwmFreq;
    uint16_t ui16M3PwmModulo;
    uint16_t ui16M3PwmDeadTime;
} clock_setup_t;

/******************************************************************************
 * Global variable definitions
 ******************************************************************************/
extern mcdrv_eflexpwm_t g_sM1Pwm3ph;
extern mcdrv_eflexpwm_t g_sM2Pwm3ph;
extern mcdrv_eflexpwm_t g_sM3Pwm3ph;
extern mcdrv_adc_t g_sM1AdcSensor;
extern mcdrv_adc_t g_sM2AdcSensor;
extern mcdrv_adc_t g_sM3AdcSensor;
extern mcdrv_adc_t g_sPFCAdcSensor;
extern clock_setup_t g_sClockSetup;
extern void InitInputmux(void);
extern void FMSTR_Recorder_Create(void);
extern void Ctimer_callback(uint32_t flags);
/*******************************************************************************
 * API
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif
void MCDRV_Init(void);

#ifdef __cplusplus
}
#endif
#endif /* _MC_PERIPH_INIT_H_  */
