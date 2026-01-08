/*
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "mc_periph_init.h"
#include "peripherals.h"
#include "fsl_common.h"
#include "pin_mux.h"
#include "fsl_gpio.h"
#include "fsl_ctimer.h"
#include "fsl_edma.h"
#include "fsl_opamp.h"
#include "fsl_inputmux.h"
#include "fsl_aoi.h"
#include "fsl_lpcmp.h"
#include "fsl_spc.h"
#include "fsl_lpuart.h"
#include "freemaster.h"
#include "freemaster_serial_lpuart.h"
#include "fsl_lptmr.h"
/*******************************************************************************
 * Defines
 ******************************************************************************/

/*******************************************************************************
 * Typedef
 ******************************************************************************/


/*${prototype:end}*/
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* Motor 1 */
static void InitADC0(void);
static void InitPWM0(void);
static void InitOpAmps(void);

/* Motor 2 */
static void InitADC1(void);
static void InitPWM1(void);
static void InitLpCmp0(void);

/* Common for motor 1 and motor 2 */
static void InitSlowLoop(void);
static void InitClock(void);
static void InitUART0(void);
static void InitAOI0(void);
static void InitAOI1(void);
static void InitCTIMER4(void);
/*******************************************************************************
 * Variables
 ******************************************************************************/
/* configuration structure for 3-phase PWM mc driver */
mcdrv_eflexpwm_t g_sM1Pwm3ph;
mcdrv_eflexpwm_t g_sM2Pwm3ph;
mcdrv_eflexpwm_t g_sM3Pwm3ph;

/* structure for current and voltage measurement*/
mcdrv_adc_t g_sM1AdcSensor;
mcdrv_adc_t g_sM2AdcSensor;
mcdrv_adc_t g_sM3AdcSensor;
mcdrv_adc_t g_sPFCAdcSensor;

/* Clock setup structure */
clock_setup_t g_sClockSetup;
/*******************************************************************************
 * Local functions
 ******************************************************************************/
/*!
 * @brief   void InitPWM0(void)
 *           - Initialization of the eFlexPWM0 peripheral for motor M1
 *           - 3-phase center-aligned PWM
 *
 * @param   void
 *
 * @return  none
 */
static void InitPWM0(void)//PWM0_SM0-2 for compressor sampling and control，SM3 for PFC sampling and control interrupt
{
  PWM_Type *PWMBase = (PWM_Type *)FLEXPWM0;

  /* eFlexPWM0 init*/
  SYSCON->PWM0SUBCTL = (SYSCON_PWM0SUBCTL_CLK0_EN_MASK |
                        SYSCON_PWM0SUBCTL_CLK1_EN_MASK |
                        SYSCON_PWM0SUBCTL_CLK2_EN_MASK |
                        SYSCON_PWM0SUBCTL_CLK3_EN_MASK); //Enable Sub-module0 clock
  
  CLOCK_EnableClock(kCLOCK_GateFLEXPWM0);
  CLOCK_EnableClock(kCLOCK_GatePWM0SM0);
  CLOCK_EnableClock(kCLOCK_GatePWM0SM1);
  CLOCK_EnableClock(kCLOCK_GatePWM0SM2);
  CLOCK_EnableClock(kCLOCK_GatePWM0SM3);

  /* value register initial values, duty cycle 50% */
  PWMBase->SM[0].INIT = PWM_INIT_INIT((uint16_t)(-(M1_PWM_MODULO / 2)));
  PWMBase->SM[1].INIT = PWM_INIT_INIT((uint16_t)(-(M1_PWM_MODULO / 2)));
  PWMBase->SM[2].INIT = PWM_INIT_INIT((uint16_t)(-(M1_PWM_MODULO / 2)));
  PWMBase->SM[3].INIT = PWM_INIT_INIT((uint16_t)(-(PFC_PWM_MODULO / 2)));  

  PWMBase->SM[0].VAL1 = PWM_VAL1_VAL1((uint16_t)((M1_PWM_MODULO / 2) - 1));
  PWMBase->SM[1].VAL1 = PWM_VAL1_VAL1((uint16_t)((M1_PWM_MODULO / 2) - 1));
  PWMBase->SM[2].VAL1 = PWM_VAL1_VAL1((uint16_t)((M1_PWM_MODULO / 2) - 1));
  PWMBase->SM[3].VAL1 = PWM_VAL1_VAL1((uint16_t)((PFC_PWM_MODULO / 2)) - 1);  

  PWMBase->SM[0].VAL2 = PWM_VAL2_VAL2((uint16_t)(-(M1_PWM_MODULO / 4)));
  PWMBase->SM[1].VAL2 = PWM_VAL2_VAL2((uint16_t)(-(M1_PWM_MODULO / 4)));
  PWMBase->SM[2].VAL2 = PWM_VAL2_VAL2((uint16_t)(-(M1_PWM_MODULO / 4)));

    
  PWMBase->SM[0].VAL3 = PWM_VAL3_VAL3((uint16_t)((M1_PWM_MODULO / 4) - 1));
  PWMBase->SM[1].VAL3 = PWM_VAL3_VAL3((uint16_t)((M1_PWM_MODULO / 4) - 1));
  PWMBase->SM[2].VAL3 = PWM_VAL3_VAL3((uint16_t)((M1_PWM_MODULO / 4) - 1));

  PWMBase->SM[3].VAL2 = PWM_VAL2_VAL2((uint16_t)(-(PFC_PWM_MODULO / 25)));    
  PWMBase->SM[3].VAL3 = PWM_VAL3_VAL3((uint16_t)((PFC_PWM_MODULO / 25) - 1));  
  PWMBase->SM[3].VAL4 = PWM_VAL4_VAL4((uint16_t)( PFC_PWM_MODULO / 2 - PFC_PWM_MODULO / 25));  
  PWMBase->SM[3].VAL5 = PWM_VAL5_VAL5((uint16_t)(-PFC_PWM_MODULO / 2 + PFC_PWM_MODULO / 25 -1));    

  /* PWM0 module 0 TRIG0 on VAL4 enabled for ADC0-M1 */
  PWMBase->SM[0].VAL4 = PWM_VAL4_VAL4((uint16_t)(-(M1_PWM_MODULO / 2)) + 60);
  PWMBase->SM[0].TCTRL |= PWM_TCTRL_OUT_TRIG_EN(1 << 4);
 
  /* PWM0 module 1 TRIG0 on VAL4 enabled for ADC1-M2 */
  /* PWM0 module 1 TRIG1 on VAL5 enabled for SYNC PWM1-SM0/SM1/SM2 */
  PWMBase->SM[1].VAL4 = PWM_VAL4_VAL4((uint16_t)(-(M1_PWM_MODULO / 6)) + 60);
  PWMBase->SM[1].VAL5 = PWM_VAL5_VAL5((uint16_t)(-(M1_PWM_MODULO / 6)));  
  PWMBase->SM[1].TCTRL |= PWM_TCTRL_OUT_TRIG_EN(1 << 4|1 << 5);
  
  /* PWM0 module 2 TRIG0 on VAL4 enabled for ADC1-M3 */
  /* PWM0 module 2 TRIG1 on VAL5 enabled for SYNC PWM1-SM3 */
  PWMBase->SM[2].VAL4 = PWM_VAL4_VAL4((uint16_t)(M1_PWM_MODULO / 6) + 60);
  PWMBase->SM[2].VAL5 = PWM_VAL5_VAL5((uint16_t)(M1_PWM_MODULO / 6));  
  PWMBase->SM[2].TCTRL |= PWM_TCTRL_OUT_TRIG_EN(1 << 4|1 << 5);  
  
  PWMBase->SM[3].TCTRL |= PWM_TCTRL_OUT_TRIG_EN(1 << 1);  
  
  /* set deadtime (number of Fast Peripheral Clocks)
     DTCNT0,1 = T_dead * f_fpc = 1.5us * 72MHz = 108 */
  /* DTCNTx = 95 if the clock is 95977472 Hz and deadtime = 1 us */
  PWMBase->SM[0].DTCNT0 = ((M1_PWM_DEADTIME * (MCU_CLOCK_FREQ / 1000000U)) / 1000U);
  PWMBase->SM[1].DTCNT0 = ((M1_PWM_DEADTIME * (MCU_CLOCK_FREQ / 1000000U)) / 1000U);
  PWMBase->SM[2].DTCNT0 = ((M1_PWM_DEADTIME * (MCU_CLOCK_FREQ / 1000000U)) / 1000U);
  PWMBase->SM[3].DTCNT0 = 0;

  PWMBase->SM[0].DTCNT1 = ((M1_PWM_DEADTIME * (MCU_CLOCK_FREQ / 1000000U)) / 1000U);
  PWMBase->SM[1].DTCNT1 = ((M1_PWM_DEADTIME * (MCU_CLOCK_FREQ / 1000000U)) / 1000U);
  PWMBase->SM[2].DTCNT1 = ((M1_PWM_DEADTIME * (MCU_CLOCK_FREQ / 1000000U)) / 1000U);
  PWMBase->SM[3].DTCNT1 = 0;

   /* Full cycle reload */
  PWMBase->SM[0].CTRL |= PWM_CTRL_FULL_MASK;
  PWMBase->SM[1].CTRL |= PWM_CTRL_FULL_MASK;
  PWMBase->SM[2].CTRL |= PWM_CTRL_FULL_MASK;
  PWMBase->SM[3].CTRL |= PWM_CTRL_FULL_MASK;
  
  //master_sync_init
  PWMBase->SM[1].CTRL2 = (PWMBase->SM[1].CTRL2 & (~PWM_CTRL2_INIT_SEL_MASK)) | PWM_CTRL2_INIT_SEL(0x2);
  PWMBase->SM[2].CTRL2 = (PWMBase->SM[2].CTRL2 & (~PWM_CTRL2_INIT_SEL_MASK)) | PWM_CTRL2_INIT_SEL(0x2);    
  //local_sync_init
  PWMBase->SM[3].CTRL2 = (PWMBase->SM[3].CTRL2 & (~PWM_CTRL2_INIT_SEL_MASK)) | PWM_CTRL2_INIT_SEL(0x0) | PWM_CTRL2_INDEP_MASK;
  PWMBase->SM[3].CTRL2 = (PWMBase->SM[3].CTRL2 & (~PWM_CTRL2_FORCE_SEL_MASK)) | PWM_CTRL2_FORCE_SEL(5U) | PWM_CTRL2_FRCEN_MASK;
  
  PWMBase->SM[3].OCTRL |= PWM_OCTRL_POLB_MASK|PWM_OCTRL_PWMBFS(1U);
  
  /* Fault trigger settings */
#if 1
  PWMBase->SM[0].DISMAP[0] = 0xF333U;
  PWMBase->SM[1].DISMAP[0] = 0xF333U;
  PWMBase->SM[2].DISMAP[0] = 0xF333U;
  PWMBase->SM[3].DISMAP[0] = 0xF111U;
#else
  PWMBase->SM[0].DISMAP[0] = 0xF000U;
  PWMBase->SM[1].DISMAP[0] = 0xF000U;
  PWMBase->SM[2].DISMAP[0] = 0xF000U;
  PWMBase->SM[3].DISMAP[0] = 0xF000U;
#endif

  /* PWMs are re-enabled at PWM full cycle */
  PWMBase->FSTS = (PWMBase->FSTS & ~PWM_FSTS_FFULL_MASK) | PWM_FSTS_FFULL(0x3);

  /* PWM fault filter - 3 Fast periph. clocks sample rate, 5 agreeing
     samples to activate */
  PWMBase->FFILT = (PWMBase->FFILT & ~PWM_FFILT_FILT_PER_MASK) | PWM_FFILT_FILT_PER(2);

  /* All interrupts disabled, safe manual fault clearing, inversed logic (trigger level = high) */
  PWMBase->FCTRL &= ~(PWM_FCTRL_FLVL_MASK | PWM_FCTRL_FAUTO_MASK | PWM_FCTRL_FSAFE_MASK | PWM_FCTRL_FIE_MASK); /* clear FCTRL register prior further settings */
  PWMBase->FCTRL |= PWM_FCTRL_FIE(0U); /* FAULT 0 - Interrupt disable */
  PWMBase->FCTRL |= PWM_FCTRL_FLVL(0xEU);
  PWMBase->FCTRL |= PWM_FCTRL_FAUTO(0U);
  PWMBase->FCTRL |= PWM_FCTRL_FSAFE(0xFU);

  /* Clear all fault flags */
  PWMBase->FSTS = (PWMBase->FSTS & ~PWM_FSTS_FFLAG_MASK) | PWM_FSTS_FFLAG(0xF);

  /* Start PWMs (set load OK flags and run - we need to trigger the ADC) */
  PWMBase->MCTRL = (PWMBase->MCTRL & ~PWM_MCTRL_CLDOK_MASK) | PWM_MCTRL_CLDOK(0xF);
  PWMBase->MCTRL = (PWMBase->MCTRL & ~PWM_MCTRL_LDOK_MASK) | PWM_MCTRL_LDOK(0xF);
//  PWMBase->MCTRL = (PWMBase->MCTRL & ~PWM_MCTRL_RUN_MASK) | PWM_MCTRL_RUN(0xF);

  /* eFlexPWM base address */
  g_sM1Pwm3ph.pui32PwmBaseAddress = (PWM_Type *)PWMBase;
  
#if 0
    PWMBase->OUTEN |= PWM_OUTEN_PWMA_EN(0xF);
    PWMBase->OUTEN |= PWM_OUTEN_PWMB_EN(0xF);
#else
    PWMBase->OUTEN &= (~PWM_OUTEN_PWMA_EN(0xF));
    PWMBase->OUTEN &= (~PWM_OUTEN_PWMB_EN(0xF));
#endif
  
    NVIC_SetPriority(FLEXPWM0_SUBMODULE3_IRQn, 1U);//for inputmux signal enable
    NVIC_EnableIRQ(FLEXPWM0_SUBMODULE3_IRQn);
}
/*!
 * @brief   void InitADC0(void)
 *           - Initialization of the ADC0 peripheral
 *           - Initialization of the A/D converter for current and voltage sensing
 *
 * @param   void
 *
 * @return  none
 */
static void InitADC0(void)//for compressor sampling
{
  lpadc_config_t lpadcConfig;
  
  /* Init the lpadcConfig */
  LPADC_GetDefaultConfig(&lpadcConfig);
  lpadcConfig.enableAnalogPreliminary = true;
  lpadcConfig.powerLevelMode = kLPADC_PowerLevelAlt4;
  lpadcConfig.referenceVoltageSource = kLPADC_ReferenceVoltageAlt3;
  lpadcConfig.conversionAverageMode = kLPADC_ConversionAverage128;

  /* Init ADC */
  lpadc_conv_trigger_config_t lpadcTriggerConfig;
  lpadc_conv_command_config_t lpadcCommandConfig;

  LPADC_Init(ADC0, &lpadcConfig);

  LPADC_DoOffsetCalibration(ADC0);
  LPADC_DoAutoCalibration(ADC0);
  
  LPADC_GetDefaultConvCommandConfig(&lpadcCommandConfig);
  lpadcCommandConfig.sampleChannelMode = kLPADC_SampleChannelSingleEndSideA;
  lpadcCommandConfig.conversionResolutionMode = kLPADC_ConversionResolutionStandard;
  lpadcCommandConfig.hardwareAverageMode = kLPADC_HardwareAverageCount1;
  lpadcCommandConfig.sampleTimeMode = kLPADC_SampleTimeADCK3;
  
  LPADC_GetDefaultConvTriggerConfig(&lpadcTriggerConfig);
  lpadcTriggerConfig.enableHardwareTrigger = true; 
  
  //////////////////////////////////////////////////////////////////////////////
  /* Set conversion CMD1 configuration. */
  lpadcCommandConfig.channelNumber = 10U;                                      /* Set ADC channel M1_IPHA */
  lpadcCommandConfig.chainedNextCommandNumber = 2U;                           
  LPADC_SetConvCommandConfig( ADC0, 1U, &lpadcCommandConfig );                

  /* Set conversion CMD2 configuration. */         
  lpadcCommandConfig.channelNumber = 11U;                                      /* Set ADC channel M1_IPHB */
  lpadcCommandConfig.chainedNextCommandNumber = 3U;
  LPADC_SetConvCommandConfig( ADC0, 2U, &lpadcCommandConfig );                

  /* Set conversion CMD3 configuration. */            
  lpadcCommandConfig.channelNumber = 12U;                                      /* Set ADC channel M1_IPHC */
  lpadcCommandConfig.chainedNextCommandNumber = 0U;                           
  LPADC_SetConvCommandConfig( ADC0, 3U, &lpadcCommandConfig );
  
  /* Init triggers (use trigger 0). */
  lpadcTriggerConfig.targetCommandId = 1U;
  LPADC_SetConvTriggerConfig(ADC0, 0U, &lpadcTriggerConfig); 
  
  //////////////////////////////////////////////////////////////////////////////
  /* Set conversion CMD4 configuration. */           
  lpadcCommandConfig.channelNumber = 9U;                                       /* Set ADC channel I_pfc2 - PWM0_A3 */
  lpadcCommandConfig.chainedNextCommandNumber = 5U;                           
  LPADC_SetConvCommandConfig( ADC0, 4U, &lpadcCommandConfig );             
  
  /* Set conversion CMD4 configuration. */           
  lpadcCommandConfig.channelNumber = 21U;                                      /* Set ADC channel HV_VDC */
  lpadcCommandConfig.chainedNextCommandNumber = 0U;                           
  LPADC_SetConvCommandConfig( ADC0, 5U, &lpadcCommandConfig );               

  /* Init triggers (use trigger 1). */  
  lpadcTriggerConfig.targetCommandId = 4U;
  LPADC_SetConvTriggerConfig(ADC0, 1U, &lpadcTriggerConfig);  
  
  //////////////////////////////////////////////////////////////////////////////
  lpadcCommandConfig.channelNumber = 8U;                                      /* Set ADC channel I_pfc1 - PWM0_B3 */
  lpadcCommandConfig.chainedNextCommandNumber = 7U;                           
  LPADC_SetConvCommandConfig( ADC0, 6U, &lpadcCommandConfig );               
  
  lpadcCommandConfig.channelNumber = 19U;                                     /* Set ADC channel HV_VAC */
  lpadcCommandConfig.chainedNextCommandNumber = 0U;                           
  LPADC_SetConvCommandConfig( ADC0, 7U, &lpadcCommandConfig );      
  
  /* Init triggers (use trigger 2). */
  lpadcTriggerConfig.targetCommandId = 6U;
  LPADC_SetConvTriggerConfig(ADC0, 2U, &lpadcTriggerConfig);
  
  //////////////////////////////////////////////////////////////////////////////
  /* Enable TCOMP interrupt. */
  LPADC_EnableInterrupts(ADC0, ADC_IE_TCOMP_IE(0x1U));//Only trigger source 0 trigger the interrupt
  NVIC_SetPriority(ADC0_IRQn, 4U);
  NVIC_EnableIRQ(ADC0_IRQn);
}

/*!
@brief   void InitHsCmp0(void)
          - Initialization of the comparator 0 module for dc-bus over current
            detection to generate eFlexPWM0 fault

@param   void

@return  none
*/
static void InitLpCmp0(void)
{
    SPC_EnableActiveModeAnalogModules(SPC0, (kSPC_controlCmp0 | kSPC_controlCmp0Dac));
    lpcmp_config_t mLpcmpConfigStruct;
    lpcmp_dac_config_t mLpcmpDacConfigStruct;
    lpcmp_filter_config_t mLpcmpFilterConfigStruct;
    /*
     *   k_LpcmpConfigStruct->enableStopMode      = false;
     *   k_LpcmpConfigStruct->enableOutputPin     = false;
     *   k_LpcmpConfigStruct->useUnfilteredOutput = false;
     *   k_LpcmpConfigStruct->enableInvertOutput  = false;
     *   k_LpcmpConfigStruct->hysteresisMode      = kLPCMP_HysteresisLevel0;
     *   k_LpcmpConfigStruct->powerMode           = kLPCMP_LowSpeedPowerMode;
     *   k_LpcmpConfigStruct->functionalSourceClock = kLPCMP_FunctionalClockSource0;
     */
    
    LPCMP_GetDefaultConfig(&mLpcmpConfigStruct);
    /* Init the LPCMP module. */
    LPCMP_Init(CMP0, &mLpcmpConfigStruct);
    
//    mLpcmpFilterConfigStruct.enableSample = false;
//    mLpcmpFilterConfigStruct.filterSampleCount = 5;
//    mLpcmpFilterConfigStruct.filterSamplePeriod= 5;
//    LPCMP_SetFilterConfig(CMP0,&mLpcmpFilterConfigStruct);
    /* Configure the internal DAC to output half of reference voltage. */
    mLpcmpDacConfigStruct.enableLowPowerMode = false;
    mLpcmpDacConfigStruct.referenceVoltageSource = kLPCMP_VrefSourceVin1;//use VDD as DAC reference
    mLpcmpDacConfigStruct.DACValue = 128+FRAC8(4.0/M2_I_MAX);  //4A protect
    LPCMP_SetDACConfig(CMP0, &mLpcmpDacConfigStruct);

    /* Configure LPCMP input channels: ch2 and DAC ch7. */
    LPCMP_SetInputChannels(CMP0, 2, 7);
}

/*!
@brief   void InitLpCmp1(void)
          - Initialization of the comparator 0 module for dc-bus over current
            detection to generate eFlexPWM0 fault

@param   void

@return  none
*/
static void InitLpCmp1(void)
{
    SPC_EnableActiveModeAnalogModules(SPC0, (kSPC_controlCmp1 | kSPC_controlCmp1Dac));
    lpcmp_config_t mLpcmpConfigStruct;
    lpcmp_dac_config_t mLpcmpDacConfigStruct;
    lpcmp_filter_config_t mLpcmpFilterConfigStruct;
    /*
     *   k_LpcmpConfigStruct->enableStopMode      = false;
     *   k_LpcmpConfigStruct->enableOutputPin     = false;
     *   k_LpcmpConfigStruct->useUnfilteredOutput = false;
     *   k_LpcmpConfigStruct->enableInvertOutput  = false;
     *   k_LpcmpConfigStruct->hysteresisMode      = kLPCMP_HysteresisLevel0;
     *   k_LpcmpConfigStruct->powerMode           = kLPCMP_LowSpeedPowerMode;
     *   k_LpcmpConfigStruct->functionalSourceClock = kLPCMP_FunctionalClockSource0;
     */
    
    LPCMP_GetDefaultConfig(&mLpcmpConfigStruct);
    /* Init the LPCMP module. */
    LPCMP_Init(CMP1, &mLpcmpConfigStruct);
    
//    mLpcmpFilterConfigStruct.enableSample = false;
//    mLpcmpFilterConfigStruct.filterSampleCount = 5;
//    mLpcmpFilterConfigStruct.filterSamplePeriod= 5;
//    LPCMP_SetFilterConfig(CMP1,&mLpcmpFilterConfigStruct);
    /* Configure the internal DAC to output half of reference voltage. */
    mLpcmpDacConfigStruct.enableLowPowerMode = false;
    mLpcmpDacConfigStruct.referenceVoltageSource = kLPCMP_VrefSourceVin1;//use VDD as DAC reference
    mLpcmpDacConfigStruct.DACValue = 128+FRAC8(7.0/M2_I_MAX);  //7A protect
    LPCMP_SetDACConfig(CMP1, &mLpcmpDacConfigStruct);

    /* Configure LPCMP input channels: ch2 and DAC ch7. */
    LPCMP_SetInputChannels(CMP1, 0, 7);
}


/*!
 * @brief   void InitOpAmps(void)
 *           - Initialization of the Operational Amplifier 0-2 peripheral
 *
 * @param   void
 *
 * @return  none
 */
static void InitOpAmps(void)
{
    SPC_EnableActiveModeAnalogModules(SPC0, (kSPC_controlOpamp0 | kSPC_controlOpamp1));
    opamp_config_t opampConfig;

    /* OPAMP0 peripheral is released from reset */
    RESET_ReleasePeripheralReset(kOPAMP0_RST_SHIFT_RSTn);
    OPAMP_GetDefaultConfig(&opampConfig);
    opampConfig.compCap     = kOPAMP_FitGain2x;
    opampConfig.biasCurrent = kOPAMP_NoChange;
    OPAMP_Init(OPAMP0, &opampConfig);
    OPAMP_Enable(OPAMP0, true);


    /* OPAMP1 peripheral is released from reset */
    RESET_ReleasePeripheralReset(kOPAMP1_RST_SHIFT_RSTn);
    OPAMP_GetDefaultConfig(&opampConfig);
    opampConfig.compCap     = kOPAMP_FitGain2x;
    opampConfig.biasCurrent = kOPAMP_NoChange;
    OPAMP_Init(OPAMP1, &opampConfig);
    OPAMP_Enable(OPAMP1, true);


}

  /* Channel CH0 definitions */
/* DMA0 eDMA source request. */
#define DMA0_CH0_DMA_REQUEST kDma0RequestMuxCtimer0M0
/* Selected eDMA channel number. */
#define DMA0_CH0_DMA_CHANNEL 0

  /* Channel CH1 definitions */
/* DMA0 eDMA source request. */
#define DMA0_CH1_DMA_REQUEST kDma0RequestMuxCtimer0M1
/* Selected eDMA channel number. */
#define DMA0_CH1_DMA_CHANNEL 1

  /* Channel CH2 definitions */
/* DMA0 eDMA source request. */
#define DMA0_CH2_DMA_REQUEST kDma0RequestMuxCtimer1M0
/* Selected eDMA channel number. */
#define DMA0_CH2_DMA_CHANNEL 2

  /* Channel CH3 definitions */
/* DMA0 eDMA source request. */
#define DMA0_CH3_DMA_REQUEST kDma0RequestMuxCtimer1M1
/* Selected eDMA channel number. */
#define DMA0_CH3_DMA_CHANNEL 3

/* DMA0 eDMA source request. */
#define DMA0_CH4_DMA_REQUEST kDma0RequestMuxCtimer2M0
/* Selected eDMA channel number. */
#define DMA0_CH4_DMA_CHANNEL 4

  /* Channel CH5 definitions */
/* DMA0 eDMA source request. */
#define DMA0_CH5_DMA_REQUEST kDma0RequestMuxCtimer2M1
/* Selected eDMA channel number. */
#define DMA0_CH5_DMA_CHANNEL 5

  /* Channel CH6 definitions */
/* DMA0 eDMA source request. */
#define DMA0_CH6_DMA_REQUEST kDma0RequestMuxCtimer3M0
/* Selected eDMA channel number. */
#define DMA0_CH6_DMA_CHANNEL 6

  /* Channel CH7 definitions */
/* DMA0 eDMA source request. */
#define DMA0_CH7_DMA_REQUEST kDma0RequestMuxCtimer3M1
/* Selected eDMA channel number. */
#define DMA0_CH7_DMA_CHANNEL 7

uint32_t srcAddr01 = 1U << BOARD_SMARTDMA0PINS_MC3_BB_GPIO_PIN;
uint32_t srcAddr23 = 1U << BOARD_SMARTDMA0PINS_MC3_BT_GPIO_PIN;
uint32_t srcAddr45 = 1U << BOARD_SMARTDMA0PINS_MC3_CB_GPIO_PIN;
uint32_t srcAddr67 = 1U << BOARD_SMARTDMA0PINS_MC3_CT_GPIO_PIN;

/* Source address extern definition */
AT_NONCACHEABLE_SECTION_ALIGN_INIT(extern uint32_t srcAddr01, 4);
/* Destination address extern definition */
#define PORT3_PCOR_ADDR 0x40105048
/* Source address extern definition */
AT_NONCACHEABLE_SECTION_ALIGN_INIT(extern uint32_t srcAddr23,4);
/* Destination address extern definition */
#define PORT3_PSOR_ADDR 0x40105044
/* Source address extern definition */
AT_NONCACHEABLE_SECTION_ALIGN_INIT(extern uint32_t srcAddr45, 4);
/* Source address extern definition */
AT_NONCACHEABLE_SECTION_ALIGN_INIT(extern uint32_t srcAddr67, 4);
/***********************************************************************************************************************
 * DMA0 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'DMA0'
- type: 'edma4'
- mode: 'general'
- custom_name_enabled: 'false'
- type_id: 'edma4_2.9.0'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'DMA0'
- config_sets:
  - fsl_edma:
    - common_settings:
      - vars: []
      - enableHaltOnError: 'false'
      - enableDebugMode: 'false'
      - enableRoundRobinArbitration: 'fixedPriority'
      - enableGlobalChannelLink: 'false'
      - enableMasterIdReplication: 'true'
    - dma_table:
      - 0: []
      - 1: []
      - 2: []
      - 3: []
    - edma_channels:
      - 0:
        - apiMode: 'nontrans'
        - edma_channel:
          - channel_prefix_id: 'CH0'
          - uid: '1722510117507'
          - eDMAn: '0'
          - eDMA_source: 'kDma0RequestMuxCtimer0M0'
          - init_channel_priority: 'true'
          - edma_channel_Preemption:
            - enableChannelPreemption: 'false'
            - enablePreemptAbility: 'false'
            - channelPriority: '0'
          - masterIdReplicationEnable: 'enabled'
          - securityLevel: 'kEDMA_ChannelSecurityLevelSecure'
          - protectionLevel: 'kEDMA_ChannelProtectionLevelUser'
        - resetChannel: 'true'
        - enableChannelRequest: 'false'
        - enableAsyncRequest: 'false'
        - nontransEnable: 'true'
        - nontrans_config:
          - uid: '1722822694864'
          - tcdID: 'CH0_transfer'
          - ssize: 'kEDMA_TransferSize4Bytes'
          - saddr_expr: '&srcAddr01'
          - saddr_def: 'AT_NONCACHEABLE_SECTION_ALIGN_INIT(extern uint32_t srcAddr01, 4);'
          - soff: '0'
          - soff_def: ''
          - smod: 'kEDMA_ModuloDisable'
          - dsize: 'kEDMA_TransferSize4Bytes'
          - daddr_expr: 'PORT3_PCOR_ADDR '
          - daddr_def: '#define PORT3_PCOR_ADDR 0x5009A048'
          - doff: '0'
          - doff_def: ''
          - dmod: 'kEDMA_ModuloDisable'
          - nbytes: '4'
          - MLconfig:
            - offsetType: 'disabled'
            - mloff: '0'
          - enableChannelLinkMinor: 'false'
          - linkedChannelMinor: '1722510117507'
          - citer: '1'
          - slast: '0'
          - dlast: '0'
          - enableChannelLinkMajor: 'false'
          - linkedChannelMajor: '1722510117507'
          - interruptSources: ''
          - setTransfer: 'true'
        - enableAutoStop: 'false'
        - init_bandwidth: 'false'
        - bandwidth: 'kEDMA_BandwidthStallNone'
        - no_init_uid: '1722510117516'
        - channel_enabled_interrupts: ''
        - init_interruptsEnable: 'false'
        - interrupt_channel:
          - IRQn: 'EDMA_0_CH0_IRQn'
          - enable_interrrupt: 'enabled'
          - enable_priority: 'false'
          - priority: '0'
          - enable_custom_name: 'false'
      - 1:
        - apiMode: 'nontrans'
        - edma_channel:
          - channel_prefix_id: 'CH1'
          - uid: '1722510119269'
          - eDMAn: '1'
          - eDMA_source: 'kDma0RequestMuxCtimer0M1'
          - init_channel_priority: 'false'
          - edma_channel_Preemption:
            - enableChannelPreemption: 'false'
            - enablePreemptAbility: 'false'
            - channelPriority: '0'
          - masterIdReplicationEnable: 'enabled'
          - securityLevel: 'kEDMA_ChannelSecurityLevelSecure'
          - protectionLevel: 'kEDMA_ChannelProtectionLevelUser'
        - resetChannel: 'true'
        - enableChannelRequest: 'false'
        - enableAsyncRequest: 'false'
        - nontransEnable: 'true'
        - nontrans_config:
          - uid: '1722822694956'
          - tcdID: 'CH1_transfer'
          - ssize: 'kEDMA_TransferSize4Bytes'
          - saddr_expr: '&srcAddr01'
          - saddr_def: 'AT_NONCACHEABLE_SECTION_ALIGN_INIT(extern uint32_t srcAddr01,4);'
          - soff: '0'
          - soff_def: ''
          - smod: 'kEDMA_ModuloDisable'
          - dsize: 'kEDMA_TransferSize4Bytes'
          - daddr_expr: 'PORT3_PSOR_ADDR '
          - daddr_def: '#define PORT3_PSOR_ADDR 0x5009A044'
          - doff: '0'
          - doff_def: ''
          - dmod: 'kEDMA_ModuloDisable'
          - nbytes: '4'
          - MLconfig:
            - offsetType: 'disabled'
            - mloff: '0'
          - enableChannelLinkMinor: 'false'
          - linkedChannelMinor: '1722510117507'
          - citer: '2'
          - slast: '0'
          - dlast: '0'
          - enableChannelLinkMajor: 'false'
          - linkedChannelMajor: '1722510117507'
          - interruptSources: ''
          - setTransfer: 'true'
        - enableAutoStop: 'false'
        - init_bandwidth: 'false'
        - bandwidth: 'kEDMA_BandwidthStallNone'
        - no_init_uid: '1722510119282'
        - channel_enabled_interrupts: ''
        - init_interruptsEnable: 'false'
        - interrupt_channel:
          - IRQn: 'EDMA_0_CH1_IRQn'
          - enable_interrrupt: 'enabled'
          - enable_priority: 'false'
          - priority: '0'
          - enable_custom_name: 'false'
      - 2:
        - apiMode: 'nontrans'
        - edma_channel:
          - channel_prefix_id: 'CH2'
          - uid: '1722510120790'
          - eDMAn: '2'
          - eDMA_source: 'kDma0RequestMuxCtimer1M0'
          - init_channel_priority: 'false'
          - edma_channel_Preemption:
            - enableChannelPreemption: 'false'
            - enablePreemptAbility: 'false'
            - channelPriority: '0'
          - masterIdReplicationEnable: 'enabled'
          - securityLevel: 'kEDMA_ChannelSecurityLevelSecure'
          - protectionLevel: 'kEDMA_ChannelProtectionLevelUser'
        - resetChannel: 'true'
        - enableChannelRequest: 'false'
        - enableAsyncRequest: 'false'
        - nontransEnable: 'true'
        - nontrans_config:
          - uid: '1722822694957'
          - tcdID: 'CH2_transfer'
          - ssize: 'kEDMA_TransferSize4Bytes'
          - saddr_expr: '&srcAddr23'
          - saddr_def: 'AT_NONCACHEABLE_SECTION_ALIGN_INIT(extern uint32_t srcAddr23, 4);'
          - soff: '0'
          - soff_def: ''
          - smod: 'kEDMA_ModuloDisable'
          - dsize: 'kEDMA_TransferSize4Bytes'
          - daddr_expr: 'PORT3_PSOR_ADDR '
          - daddr_def: ''
          - doff: '0'
          - doff_def: ''
          - dmod: 'kEDMA_ModuloDisable'
          - nbytes: '4'
          - MLconfig:
            - offsetType: 'disabled'
            - mloff: '0'
          - enableChannelLinkMinor: 'false'
          - linkedChannelMinor: '1722510117507'
          - citer: '2'
          - slast: '0'
          - dlast: '0'
          - enableChannelLinkMajor: 'false'
          - linkedChannelMajor: '1722510117507'
          - interruptSources: ''
          - setTransfer: 'true'
        - enableAutoStop: 'false'
        - init_bandwidth: 'false'
        - bandwidth: 'kEDMA_BandwidthStallNone'
        - no_init_uid: '1722510120799'
        - channel_enabled_interrupts: ''
        - init_interruptsEnable: 'false'
        - interrupt_channel:
          - IRQn: 'EDMA_0_CH2_IRQn'
          - enable_interrrupt: 'enabled'
          - enable_priority: 'false'
          - priority: '0'
          - enable_custom_name: 'false'
      - 3:
        - apiMode: 'nontrans'
        - edma_channel:
          - channel_prefix_id: 'CH3'
          - uid: '1722510122240'
          - eDMAn: '3'
          - eDMA_source: 'kDma0RequestMuxCtimer1M1'
          - init_channel_priority: 'false'
          - edma_channel_Preemption:
            - enableChannelPreemption: 'false'
            - enablePreemptAbility: 'false'
            - channelPriority: '0'
          - masterIdReplicationEnable: 'enabled'
          - securityLevel: 'kEDMA_ChannelSecurityLevelSecure'
          - protectionLevel: 'kEDMA_ChannelProtectionLevelUser'
        - resetChannel: 'true'
        - enableChannelRequest: 'false'
        - enableAsyncRequest: 'false'
        - nontransEnable: 'true'
        - nontrans_config:
          - uid: '1722822694958'
          - tcdID: 'CH3_transfer'
          - ssize: 'kEDMA_TransferSize4Bytes'
          - saddr_expr: '&srcAddr23'
          - saddr_def: 'AT_NONCACHEABLE_SECTION_ALIGN_INIT(extern uint32_t srcAddr23, 4);'
          - soff: '0'
          - soff_def: ''
          - smod: 'kEDMA_ModuloDisable'
          - dsize: 'kEDMA_TransferSize4Bytes'
          - daddr_expr: 'PORT3_PCOR_ADDR '
          - daddr_def: ''
          - doff: '0'
          - doff_def: ''
          - dmod: 'kEDMA_ModuloDisable'
          - nbytes: '4'
          - MLconfig:
            - offsetType: 'disabled'
            - mloff: '0'
          - enableChannelLinkMinor: 'false'
          - linkedChannelMinor: '1722510117507'
          - citer: '2'
          - slast: '0'
          - dlast: '0'
          - enableChannelLinkMajor: 'false'
          - linkedChannelMajor: '1722510117507'
          - interruptSources: ''
          - setTransfer: 'true'
        - enableAutoStop: 'false'
        - init_bandwidth: 'false'
        - bandwidth: 'kEDMA_BandwidthStallNone'
        - no_init_uid: '1722510122251'
        - channel_enabled_interrupts: ''
        - init_interruptsEnable: 'false'
        - interrupt_channel:
          - IRQn: 'EDMA_0_CH3_IRQn'
          - enable_interrrupt: 'enabled'
          - enable_priority: 'false'
          - priority: '0'
          - enable_custom_name: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
edma_config_t DMA0_config = {
  .enableMasterIdReplication = true,
  .enableGlobalChannelLink = false,
  .enableHaltOnError = false,
  .enableDebugMode = false,
  .enableRoundRobinArbitration = false
};
edma_transfer_config_t DMA0_CH0_TRANSFER_CONFIG = {
  .srcAddr = (uint32_t) &srcAddr01,
  .destAddr = (uint32_t) PORT3_PCOR_ADDR ,
  .srcTransferSize = kEDMA_TransferSize4Bytes,
  .destTransferSize = kEDMA_TransferSize4Bytes,
  .srcOffset = (int16_t) 0,
  .destOffset = (int16_t) 0,
  .minorLoopBytes = 4U,
  .majorLoopCounts = 1U,
  .enabledInterruptMask = 0U,
  .srcAddrModulo = kEDMA_ModuloDisable,
  .srcMajorLoopOffset = 0,
  .dstAddrModulo = kEDMA_ModuloDisable,
  .dstMajorLoopOffset = 0,
  .enableSrcMinorLoopOffset = false,
  .enableDstMinorLoopOffset = false,
  .enableChannelMajorLoopLink = false,
  .enableChannelMinorLoopLink = false,
  .linkTCD = NULL,
};
edma_transfer_config_t DMA0_CH1_TRANSFER_CONFIG = {
  .srcAddr = (uint32_t) &srcAddr01,
  .destAddr = (uint32_t) PORT3_PSOR_ADDR ,
  .srcTransferSize = kEDMA_TransferSize4Bytes,
  .destTransferSize = kEDMA_TransferSize4Bytes,
  .srcOffset = (int16_t) 0,
  .destOffset = (int16_t) 0,
  .minorLoopBytes = 4U,
  .majorLoopCounts = 2U,
  .enabledInterruptMask = 0U,
  .srcAddrModulo = kEDMA_ModuloDisable,
  .srcMajorLoopOffset = 0,
  .dstAddrModulo = kEDMA_ModuloDisable,
  .dstMajorLoopOffset = 0,
  .enableSrcMinorLoopOffset = false,
  .enableDstMinorLoopOffset = false,
  .enableChannelMajorLoopLink = false,
  .enableChannelMinorLoopLink = false,
  .linkTCD = NULL,
};
edma_transfer_config_t DMA0_CH2_TRANSFER_CONFIG = {
  .srcAddr = (uint32_t) &srcAddr23,
  .destAddr = (uint32_t) PORT3_PSOR_ADDR ,
  .srcTransferSize = kEDMA_TransferSize4Bytes,
  .destTransferSize = kEDMA_TransferSize4Bytes,
  .srcOffset = (int16_t) 0,
  .destOffset = (int16_t) 0,
  .minorLoopBytes = 4U,
  .majorLoopCounts = 2U,
  .enabledInterruptMask = 0U,
  .srcAddrModulo = kEDMA_ModuloDisable,
  .srcMajorLoopOffset = 0,
  .dstAddrModulo = kEDMA_ModuloDisable,
  .dstMajorLoopOffset = 0,
  .enableSrcMinorLoopOffset = false,
  .enableDstMinorLoopOffset = false,
  .enableChannelMajorLoopLink = false,
  .enableChannelMinorLoopLink = false,
  .linkTCD = NULL,
};
edma_transfer_config_t DMA0_CH3_TRANSFER_CONFIG = {
  .srcAddr = (uint32_t) &srcAddr23,
  .destAddr = (uint32_t) PORT3_PCOR_ADDR ,
  .srcTransferSize = kEDMA_TransferSize4Bytes,
  .destTransferSize = kEDMA_TransferSize4Bytes,
  .srcOffset = (int16_t) 0,
  .destOffset = (int16_t) 0,
  .minorLoopBytes = 4U,
  .majorLoopCounts = 2U,
  .enabledInterruptMask = 0U,
  .srcAddrModulo = kEDMA_ModuloDisable,
  .srcMajorLoopOffset = 0,
  .dstAddrModulo = kEDMA_ModuloDisable,
  .dstMajorLoopOffset = 0,
  .enableSrcMinorLoopOffset = false,
  .enableDstMinorLoopOffset = false,
  .enableChannelMajorLoopLink = false,
  .enableChannelMinorLoopLink = false,
  .linkTCD = NULL,
};


edma_transfer_config_t DMA0_CH4_TRANSFER_CONFIG = {
  .srcAddr = (uint32_t) &srcAddr45,
  .destAddr = (uint32_t) PORT3_PCOR_ADDR ,
  .srcTransferSize = kEDMA_TransferSize4Bytes,
  .destTransferSize = kEDMA_TransferSize4Bytes,
  .srcOffset = (int16_t) 0,
  .destOffset = (int16_t) 0,
  .minorLoopBytes = 4U,
  .majorLoopCounts = 1U,
  .enabledInterruptMask = 0U,
  .srcAddrModulo = kEDMA_ModuloDisable,
  .srcMajorLoopOffset = 0,
  .dstAddrModulo = kEDMA_ModuloDisable,
  .dstMajorLoopOffset = 0,
  .enableSrcMinorLoopOffset = false,
  .enableDstMinorLoopOffset = false,
  .enableChannelMajorLoopLink = false,
  .enableChannelMinorLoopLink = false,
  .linkTCD = NULL,
};
edma_transfer_config_t DMA0_CH5_TRANSFER_CONFIG = {
  .srcAddr = (uint32_t) &srcAddr45,
  .destAddr = (uint32_t) PORT3_PSOR_ADDR ,
  .srcTransferSize = kEDMA_TransferSize4Bytes,
  .destTransferSize = kEDMA_TransferSize4Bytes,
  .srcOffset = (int16_t) 0,
  .destOffset = (int16_t) 0,
  .minorLoopBytes = 4U,
  .majorLoopCounts = 2U,
  .enabledInterruptMask = 0U,
  .srcAddrModulo = kEDMA_ModuloDisable,
  .srcMajorLoopOffset = 0,
  .dstAddrModulo = kEDMA_ModuloDisable,
  .dstMajorLoopOffset = 0,
  .enableSrcMinorLoopOffset = false,
  .enableDstMinorLoopOffset = false,
  .enableChannelMajorLoopLink = false,
  .enableChannelMinorLoopLink = false,
  .linkTCD = NULL,
};
edma_transfer_config_t DMA0_CH6_TRANSFER_CONFIG = {
  .srcAddr = (uint32_t) &srcAddr67,
  .destAddr = (uint32_t) PORT3_PSOR_ADDR ,
  .srcTransferSize = kEDMA_TransferSize4Bytes,
  .destTransferSize = kEDMA_TransferSize4Bytes,
  .srcOffset = (int16_t) 0,
  .destOffset = (int16_t) 0,
  .minorLoopBytes = 4U,
  .majorLoopCounts = 2U,
  .enabledInterruptMask = 0U,
  .srcAddrModulo = kEDMA_ModuloDisable,
  .srcMajorLoopOffset = 0,
  .dstAddrModulo = kEDMA_ModuloDisable,
  .dstMajorLoopOffset = 0,
  .enableSrcMinorLoopOffset = false,
  .enableDstMinorLoopOffset = false,
  .enableChannelMajorLoopLink = false,
  .enableChannelMinorLoopLink = false,
  .linkTCD = NULL,
};
edma_transfer_config_t DMA0_CH7_TRANSFER_CONFIG = {
  .srcAddr = (uint32_t) &srcAddr67,
  .destAddr = (uint32_t) PORT3_PCOR_ADDR ,
  .srcTransferSize = kEDMA_TransferSize4Bytes,
  .destTransferSize = kEDMA_TransferSize4Bytes,
  .srcOffset = (int16_t) 0,
  .destOffset = (int16_t) 0,
  .minorLoopBytes = 4U,
  .majorLoopCounts = 2U,
  .enabledInterruptMask = 0U,
  .srcAddrModulo = kEDMA_ModuloDisable,
  .srcMajorLoopOffset = 0,
  .dstAddrModulo = kEDMA_ModuloDisable,
  .dstMajorLoopOffset = 0,
  .enableSrcMinorLoopOffset = false,
  .enableDstMinorLoopOffset = false,
  .enableChannelMajorLoopLink = false,
  .enableChannelMinorLoopLink = false,
  .linkTCD = NULL,
};

const edma_channel_Preemption_config_t DMA0_CH0_preemption_config = {
  .enableChannelPreemption = false,
  .enablePreemptAbility = false,
  .channelPriority = 0U
};

static void DMA0_init(void) {

  /* Channel CH0 initialization */
  /* Set the kDma0RequestMuxCtimer0M0 request */
  EDMA_SetChannelMux(DMA0, DMA0_CH0_DMA_CHANNEL, DMA0_CH0_DMA_REQUEST);
  /* Set the DMA 0 channel priority */
  EDMA_SetChannelPreemptionConfig(DMA0, DMA0_CH0_DMA_CHANNEL, &DMA0_CH0_preemption_config);
  /* Set the DMA channel 0 leader ID replication */
  EDMA_EnableChannelMasterIDReplication(DMA0, DMA0_CH0_DMA_CHANNEL, true);
  /* Set the DMA channel 0 security level */
//  EDMA_SetChannelSecurityLevel(DMA0, DMA0_CH0_DMA_CHANNEL, kEDMA_ChannelSecurityLevelSecure);
  /* Set the DMA channel 0 protection level */
  EDMA_SetChannelProtectionLevel(DMA0, DMA0_CH0_DMA_CHANNEL, kEDMA_ChannelProtectionLevelUser);
  /* DMA0 channel 0 reset */
  EDMA_ResetChannel(DMA0, DMA0_CH0_DMA_CHANNEL);
  /* DMA0 transfer initialization */
  EDMA_SetTransferConfig(DMA0, DMA0_CH0_DMA_CHANNEL, &DMA0_CH0_TRANSFER_CONFIG, NULL);

  /* Channel CH1 initialization */
  /* Set the kDma0RequestMuxCtimer0M1 request */
  EDMA_SetChannelMux(DMA0, DMA0_CH1_DMA_CHANNEL, DMA0_CH1_DMA_REQUEST);
  /* Set the DMA channel 1 leader ID replication */
  EDMA_EnableChannelMasterIDReplication(DMA0, DMA0_CH1_DMA_CHANNEL, true);
  /* Set the DMA channel 1 security level */
//  EDMA_SetChannelSecurityLevel(DMA0, DMA0_CH1_DMA_CHANNEL, kEDMA_ChannelSecurityLevelSecure);
  /* Set the DMA channel 1 protection level */
  EDMA_SetChannelProtectionLevel(DMA0, DMA0_CH1_DMA_CHANNEL, kEDMA_ChannelProtectionLevelUser);
  /* DMA0 channel 1 reset */
  EDMA_ResetChannel(DMA0, DMA0_CH1_DMA_CHANNEL);
  /* DMA0 transfer initialization */
  EDMA_SetTransferConfig(DMA0, DMA0_CH1_DMA_CHANNEL, &DMA0_CH1_TRANSFER_CONFIG, NULL);

  /* Channel CH2 initialization */
  /* Set the kDma0RequestMuxCtimer1M0 request */
  EDMA_SetChannelMux(DMA0, DMA0_CH2_DMA_CHANNEL, DMA0_CH2_DMA_REQUEST);
  /* Set the DMA channel 2 leader ID replication */
  EDMA_EnableChannelMasterIDReplication(DMA0, DMA0_CH2_DMA_CHANNEL, true);
  /* Set the DMA channel 2 security level */
//  EDMA_SetChannelSecurityLevel(DMA0, DMA0_CH2_DMA_CHANNEL, kEDMA_ChannelSecurityLevelSecure);
  /* Set the DMA channel 2 protection level */
  EDMA_SetChannelProtectionLevel(DMA0, DMA0_CH2_DMA_CHANNEL, kEDMA_ChannelProtectionLevelUser);
  /* DMA0 channel 2 reset */
  EDMA_ResetChannel(DMA0, DMA0_CH2_DMA_CHANNEL);
  /* DMA0 transfer initialization */
  EDMA_SetTransferConfig(DMA0, DMA0_CH2_DMA_CHANNEL, &DMA0_CH2_TRANSFER_CONFIG, NULL);

  /* Channel CH3 initialization */
  /* Set the kDma0RequestMuxCtimer1M1 request */
  EDMA_SetChannelMux(DMA0, DMA0_CH3_DMA_CHANNEL, DMA0_CH3_DMA_REQUEST);
  /* Set the DMA channel 3 leader ID replication */
  EDMA_EnableChannelMasterIDReplication(DMA0, DMA0_CH3_DMA_CHANNEL, true);
  /* Set the DMA channel 3 security level */
//  EDMA_SetChannelSecurityLevel(DMA0, DMA0_CH3_DMA_CHANNEL, kEDMA_ChannelSecurityLevelSecure);
  /* Set the DMA channel 3 protection level */
  EDMA_SetChannelProtectionLevel(DMA0, DMA0_CH3_DMA_CHANNEL, kEDMA_ChannelProtectionLevelUser);
  /* DMA0 channel 3 reset */
  EDMA_ResetChannel(DMA0, DMA0_CH3_DMA_CHANNEL);
  /* DMA0 transfer initialization */
  EDMA_SetTransferConfig(DMA0, DMA0_CH3_DMA_CHANNEL, &DMA0_CH3_TRANSFER_CONFIG, NULL);
  
  /* Channel CH4 initialization */
  /* Set the kDma0RequestMuxCtimer0M0 request */
  EDMA_SetChannelMux(DMA0, DMA0_CH4_DMA_CHANNEL, DMA0_CH4_DMA_REQUEST);
  /* Set the DMA channel 4 leader ID replication */
  EDMA_EnableChannelMasterIDReplication(DMA0, DMA0_CH4_DMA_CHANNEL, true);
  /* Set the DMA channel 4 security level */
//  EDMA_SetChannelSecurityLevel(DMA0, DMA0_CH4_DMA_CHANNEL, kEDMA_ChannelSecurityLevelSecure);
  /* Set the DMA channel 4 protection level */
  EDMA_SetChannelProtectionLevel(DMA0, DMA0_CH4_DMA_CHANNEL, kEDMA_ChannelProtectionLevelUser);
  /* DMA0 channel4 reset */
  EDMA_ResetChannel(DMA0, DMA0_CH4_DMA_CHANNEL);
  /* DMA0 transfer initialization */
  EDMA_SetTransferConfig(DMA0, DMA0_CH4_DMA_CHANNEL, &DMA0_CH4_TRANSFER_CONFIG, NULL);

  /* Channel CH5 initialization */
  /* Set the kDma0RequestMuxCtimer0M1 request */
  EDMA_SetChannelMux(DMA0, DMA0_CH5_DMA_CHANNEL, DMA0_CH5_DMA_REQUEST);
  /* Set the DMA channel 1 leader ID replication */
  EDMA_EnableChannelMasterIDReplication(DMA0, DMA0_CH5_DMA_CHANNEL, true);
  /* Set the DMA channel 1 security level */
//  EDMA_SetChannelSecurityLevel(DMA0, DMA0_CH5_DMA_CHANNEL, kEDMA_ChannelSecurityLevelSecure);
  /* Set the DMA channel 1 protection level */
  EDMA_SetChannelProtectionLevel(DMA0, DMA0_CH5_DMA_CHANNEL, kEDMA_ChannelProtectionLevelUser);
  /* DMA0 channel 1 reset */
  EDMA_ResetChannel(DMA0, DMA0_CH5_DMA_CHANNEL);
  /* DMA0 transfer initialization */
  EDMA_SetTransferConfig(DMA0, DMA0_CH5_DMA_CHANNEL, &DMA0_CH5_TRANSFER_CONFIG, NULL);

  /* Channel CH6 initialization */
  /* Set the kDma0RequestMuxCtimer1M0 request */
  EDMA_SetChannelMux(DMA0, DMA0_CH6_DMA_CHANNEL, DMA0_CH6_DMA_REQUEST);
  /* Set the DMA channel 2 leader ID replication */
  EDMA_EnableChannelMasterIDReplication(DMA0, DMA0_CH6_DMA_CHANNEL, true);
  /* Set the DMA channel 2 security level */
//  EDMA_SetChannelSecurityLevel(DMA0, DMA0_CH6_DMA_CHANNEL, kEDMA_ChannelSecurityLevelSecure);
  /* Set the DMA channel 2 protection level */
  EDMA_SetChannelProtectionLevel(DMA0, DMA0_CH6_DMA_CHANNEL, kEDMA_ChannelProtectionLevelUser);
  /* DMA0 channel 2 reset */
  EDMA_ResetChannel(DMA0, DMA0_CH6_DMA_CHANNEL);
  /* DMA0 transfer initialization */
  EDMA_SetTransferConfig(DMA0, DMA0_CH6_DMA_CHANNEL, &DMA0_CH6_TRANSFER_CONFIG, NULL);

  /* Channel CH7 initialization */
  /* Set the kDma0RequestMuxCtimer1M1 request */
  EDMA_SetChannelMux(DMA0, DMA0_CH7_DMA_CHANNEL, DMA0_CH7_DMA_REQUEST);
  /* Set the DMA channel 3 leader ID replication */
  EDMA_EnableChannelMasterIDReplication(DMA0, DMA0_CH7_DMA_CHANNEL, true);
  /* Set the DMA channel 3 security level */
//  EDMA_SetChannelSecurityLevel(DMA0, DMA0_CH7_DMA_CHANNEL, kEDMA_ChannelSecurityLevelSecure);
  /* Set the DMA channel 3 protection level */
  EDMA_SetChannelProtectionLevel(DMA0, DMA0_CH7_DMA_CHANNEL, kEDMA_ChannelProtectionLevelUser);
  /* DMA0 channel 3 reset */
  EDMA_ResetChannel(DMA0, DMA0_CH7_DMA_CHANNEL);
  /* DMA0 transfer initialization */
  EDMA_SetTransferConfig(DMA0, DMA0_CH7_DMA_CHANNEL, &DMA0_CH7_TRANSFER_CONFIG, NULL); 
   
  
  /* DMA0 hardware channel 0 request auto stop */
  EDMA_EnableAutoStopRequest(DMA0, DMA0_CH0_DMA_CHANNEL, false);
  /* DMA0 hardware channel 1 request auto stop */
  EDMA_EnableAutoStopRequest(DMA0, DMA0_CH1_DMA_CHANNEL, false);
  /* DMA0 hardware channel 2 request auto stop */
  EDMA_EnableAutoStopRequest(DMA0, DMA0_CH2_DMA_CHANNEL, false);
  /* DMA0 hardware channel 3 request auto stop */
  EDMA_EnableAutoStopRequest(DMA0, DMA0_CH3_DMA_CHANNEL, false);
  
  EDMA_EnableAutoStopRequest(DMA0, DMA0_CH4_DMA_CHANNEL, false);
  /* DMA0 hardware channel 1 request auto stop */
  EDMA_EnableAutoStopRequest(DMA0, DMA0_CH5_DMA_CHANNEL, false);
  /* DMA0 hardware channel 2 request auto stop */
  EDMA_EnableAutoStopRequest(DMA0, DMA0_CH6_DMA_CHANNEL, false);
  /* DMA0 hardware channel 3 request auto stop */
  EDMA_EnableAutoStopRequest(DMA0, DMA0_CH7_DMA_CHANNEL, false);  
  
}

static void DMA_EnableAllChannels(void)
{
    /* Enable DMA channels 0-7 */
    for (uint8_t channel = 0; channel <= 7; channel++)
    {
        EDMA_EnableChannelRequest(DMA0, channel);
    }
}

static void InitCTIMER0(void)
{
    ctimer_config_t config;
    ctimer_match_config_t matchConfig;

    CTIMER_GetDefaultConfig(&config);

    CTIMER_Init(CTIMER0, &config);
//    CTIMER0->TCR |= CTIMER_TCR_ATCEN(1);
    matchConfig.enableCounterReset = false;
    matchConfig.enableCounterStop  = false;
    matchConfig.matchValue         = 5622;//CLOCK_GetCTimerClkFreq(0U) / M3_PWM_FREQ / 2 - 1;
    matchConfig.outControl         = kCTIMER_Output_NoAction;
    matchConfig.outPinInitState    = true;
    matchConfig.enableInterrupt    = false;
    CTIMER_SetupMatch(CTIMER0, kCTIMER_Match_0, &matchConfig);

    matchConfig.enableCounterReset = false;
    matchConfig.enableCounterStop  = false;
    matchConfig.matchValue         = 16957;//CLOCK_GetCTimerClkFreq(0U) / M3_PWM_FREQ / 2 - 1;
    matchConfig.outControl         = kCTIMER_Output_NoAction;
    matchConfig.outPinInitState    = true;
    matchConfig.enableInterrupt    = false;
    CTIMER_SetupMatch(CTIMER0, kCTIMER_Match_1, &matchConfig);
    
    matchConfig.enableCounterReset = true;
    matchConfig.enableCounterStop  = true;
    matchConfig.matchValue         = CLOCK_GetCTimerClkFreq(0U) / M3_PWM_FREQ - 10;
    matchConfig.outControl         = kCTIMER_Output_NoAction;
    matchConfig.outPinInitState    = true;
    matchConfig.enableInterrupt    = false;
    CTIMER_SetupMatch(CTIMER0, kCTIMER_Match_2, &matchConfig);

    CTIMER0->MCR |= (3<<24);
}

static void InitCTIMER1(void)
{
    ctimer_config_t config;
    ctimer_match_config_t matchConfig;

    CTIMER_GetDefaultConfig(&config);

    CTIMER_Init(CTIMER1, &config);
//    CTIMER1->TCR |= CTIMER_TCR_ATCEN(1);
    matchConfig.enableCounterReset = false;
    matchConfig.enableCounterStop  = false;
    matchConfig.matchValue         = 5711;//CLOCK_GetCTimerClkFreq(0U) / M3_PWM_FREQ / 2 - 1;
    matchConfig.outControl         = kCTIMER_Output_NoAction;
    matchConfig.outPinInitState    = true;
    matchConfig.enableInterrupt    = false;
    CTIMER_SetupMatch(CTIMER1, kCTIMER_Match_0, &matchConfig);

    matchConfig.enableCounterReset = false;
    matchConfig.enableCounterStop  = false;
    matchConfig.matchValue         = 16868;//CLOCK_GetCTimerClkFreq(0U) / M3_PWM_FREQ / 2 - 1;
    matchConfig.outControl         = kCTIMER_Output_NoAction;
    matchConfig.outPinInitState    = true;
    matchConfig.enableInterrupt    = false;
    CTIMER_SetupMatch(CTIMER1, kCTIMER_Match_1, &matchConfig);
    
    matchConfig.enableCounterReset = true;
    matchConfig.enableCounterStop  = true;
    matchConfig.matchValue         = CLOCK_GetCTimerClkFreq(1U) / M3_PWM_FREQ - 10;
    matchConfig.outControl         = kCTIMER_Output_NoAction;
    matchConfig.outPinInitState    = true;
    matchConfig.enableInterrupt    = false;
    CTIMER_SetupMatch(CTIMER1, kCTIMER_Match_2, &matchConfig);

    CTIMER1->MCR |= (3<<24);

}

static void InitCTIMER2(void)
{
    ctimer_config_t config;
    ctimer_match_config_t matchConfig;

    CTIMER_GetDefaultConfig(&config);

    CTIMER_Init(CTIMER2, &config);
//    CTIMER2->TCR |= CTIMER_TCR_ATCEN(1);
    matchConfig.enableCounterReset = false;
    matchConfig.enableCounterStop  = false;
    matchConfig.matchValue         = 5622;//CLOCK_GetCTimerClkFreq(0U) / M3_PWM_FREQ / 2 - 1;
    matchConfig.outControl         = kCTIMER_Output_NoAction;
    matchConfig.outPinInitState    = true;
    matchConfig.enableInterrupt    = false;
    CTIMER_SetupMatch(CTIMER2, kCTIMER_Match_0, &matchConfig);

    matchConfig.enableCounterReset = false;
    matchConfig.enableCounterStop  = false;
    matchConfig.matchValue         = 16957;//CLOCK_GetCTimerClkFreq(0U) / M3_PWM_FREQ / 2 - 1;
    matchConfig.outControl         = kCTIMER_Output_NoAction;
    matchConfig.outPinInitState    = true;
    matchConfig.enableInterrupt    = false;
    CTIMER_SetupMatch(CTIMER2, kCTIMER_Match_1, &matchConfig);
    
    matchConfig.enableCounterReset = true;
    matchConfig.enableCounterStop  = true;
    matchConfig.matchValue         = CLOCK_GetCTimerClkFreq(2U) / M3_PWM_FREQ - 10;
    matchConfig.outControl         = kCTIMER_Output_NoAction;
    matchConfig.outPinInitState    = true;
    matchConfig.enableInterrupt    = false;
	
    CTIMER_SetupMatch(CTIMER2, kCTIMER_Match_2, &matchConfig);

    CTIMER2->MCR |= (3<<24);
}


static void InitCTIMER3(void)
{
    ctimer_config_t config;
    ctimer_match_config_t matchConfig;

    CTIMER_GetDefaultConfig(&config);

    CTIMER_Init(CTIMER3, &config);
 //   CTIMER3->TCR |= CTIMER_TCR_ATCEN(1);
    matchConfig.enableCounterReset = false;
    matchConfig.enableCounterStop  = false;
    matchConfig.matchValue         = 5711;//CLOCK_GetCTimerClkFreq(0U) / M3_PWM_FREQ / 2 - 1;
    matchConfig.outControl         = kCTIMER_Output_NoAction;
    matchConfig.outPinInitState    = true;
    matchConfig.enableInterrupt    = false;
    CTIMER_SetupMatch(CTIMER3, kCTIMER_Match_0, &matchConfig);

    matchConfig.enableCounterReset = false;
    matchConfig.enableCounterStop  = false;
    matchConfig.matchValue         = 16868;//CLOCK_GetCTimerClkFreq(0U) / M3_PWM_FREQ / 2 - 1;
    matchConfig.outControl         = kCTIMER_Output_NoAction;
    matchConfig.outPinInitState    = true;
    matchConfig.enableInterrupt    = false;
    CTIMER_SetupMatch(CTIMER3, kCTIMER_Match_1, &matchConfig);
    
    matchConfig.enableCounterReset = true;
    matchConfig.enableCounterStop  = true;
    matchConfig.matchValue         = CLOCK_GetCTimerClkFreq(3U) / M3_PWM_FREQ - 10;
    matchConfig.outControl         = kCTIMER_Output_NoAction;
    matchConfig.outPinInitState    = true;
    matchConfig.enableInterrupt    = false;
      
    CTIMER_SetupMatch(CTIMER3, kCTIMER_Match_2, &matchConfig);

    CTIMER3->MCR |= (3<<24);

}

static void InitPWM1(void)
{
    /* Enable eFlexPWM1 AHB clock */
    CLOCK_EnableClock(kCLOCK_GateFLEXPWM1);
    CLOCK_EnableClock(kCLOCK_GatePWM1SM0);
    CLOCK_EnableClock(kCLOCK_GatePWM1SM1);
    CLOCK_EnableClock(kCLOCK_GatePWM1SM2);
    CLOCK_EnableClock(kCLOCK_GatePWM1SM3);

    RESET_PeripheralReset(kFLEXPWM1_RST_SHIFT_RSTn);

    /* Enable Submodules 0 - 2 clocks */
    SYSCON->PWM1SUBCTL |= SYSCON_PWM1SUBCTL_CLK0_EN_MASK |
                          SYSCON_PWM1SUBCTL_CLK1_EN_MASK |
                          SYSCON_PWM1SUBCTL_CLK2_EN_MASK |
                          SYSCON_PWM1SUBCTL_CLK3_EN_MASK;

    /* PWM base pointer (affects the entire initialization) */
    PWM_Type *PWMBase = (PWM_Type *)FLEXPWM1;

    /* Full cycle reload */
    PWMBase->SM[0].CTRL |= PWM_CTRL_FULL_MASK;
    PWMBase->SM[1].CTRL |= PWM_CTRL_FULL_MASK;
    PWMBase->SM[2].CTRL |= PWM_CTRL_FULL_MASK;
    PWMBase->SM[3].CTRL |= PWM_CTRL_FULL_MASK;    

    /* Value register initial values, duty cycle 50% */
    PWMBase->SM[0].INIT = PWM_INIT_INIT((uint16_t)(-(g_sClockSetup.ui16M2PwmModulo / 2)));
    PWMBase->SM[1].INIT = PWM_INIT_INIT((uint16_t)(-(g_sClockSetup.ui16M2PwmModulo / 2)));
    PWMBase->SM[2].INIT = PWM_INIT_INIT((uint16_t)(-(g_sClockSetup.ui16M2PwmModulo / 2)));
    PWMBase->SM[3].INIT = PWM_INIT_INIT((uint16_t)(-(g_sClockSetup.ui16M3PwmModulo / 2)));

    PWMBase->SM[0].VAL0 = PWM_VAL0_VAL0((uint16_t)(0));
    PWMBase->SM[1].VAL0 = PWM_VAL0_VAL0((uint16_t)(0));
    PWMBase->SM[2].VAL0 = PWM_VAL0_VAL0((uint16_t)(0));
    PWMBase->SM[3].VAL0 = PWM_VAL0_VAL0((uint16_t)(0));

    PWMBase->SM[0].VAL1 = PWM_VAL1_VAL1((uint16_t)((g_sClockSetup.ui16M2PwmModulo / 2) - 1));
    PWMBase->SM[1].VAL1 = PWM_VAL1_VAL1((uint16_t)((g_sClockSetup.ui16M2PwmModulo / 2) - 1));
    PWMBase->SM[2].VAL1 = PWM_VAL1_VAL1((uint16_t)((g_sClockSetup.ui16M2PwmModulo / 2) - 1));
    PWMBase->SM[3].VAL1 = PWM_VAL1_VAL1((uint16_t)((g_sClockSetup.ui16M3PwmModulo / 2) - 1));    

    PWMBase->SM[0].VAL2 = PWM_VAL2_VAL2((uint16_t)(-(g_sClockSetup.ui16M2PwmModulo / 4)));
    PWMBase->SM[1].VAL2 = PWM_VAL2_VAL2((uint16_t)(-(g_sClockSetup.ui16M2PwmModulo / 4)));
    PWMBase->SM[2].VAL2 = PWM_VAL2_VAL2((uint16_t)(-(g_sClockSetup.ui16M2PwmModulo / 4)));
    PWMBase->SM[3].VAL2 = PWM_VAL2_VAL2((uint16_t)(-(g_sClockSetup.ui16M3PwmModulo / 4)));    

    PWMBase->SM[0].VAL3 = PWM_VAL3_VAL3((uint16_t)(g_sClockSetup.ui16M2PwmModulo / 4));
    PWMBase->SM[1].VAL3 = PWM_VAL3_VAL3((uint16_t)(g_sClockSetup.ui16M2PwmModulo / 4));
    PWMBase->SM[2].VAL3 = PWM_VAL3_VAL3((uint16_t)(g_sClockSetup.ui16M2PwmModulo / 4));
    PWMBase->SM[3].VAL3 = PWM_VAL3_VAL3((uint16_t)(g_sClockSetup.ui16M3PwmModulo / 4));    

    PWMBase->SM[0].VAL4 = PWM_VAL4_VAL4((uint16_t)(0));
    PWMBase->SM[1].VAL4 = PWM_VAL4_VAL4((uint16_t)(0));
    PWMBase->SM[2].VAL4 = PWM_VAL4_VAL4((uint16_t)(0));
    PWMBase->SM[3].VAL4 = PWM_VAL4_VAL4((uint16_t)((g_sClockSetup.ui16M3PwmModulo / 2) - 1));    

    PWMBase->SM[0].VAL5 = PWM_VAL5_VAL5((uint16_t)(0));
    PWMBase->SM[1].VAL5 = PWM_VAL5_VAL5((uint16_t)(0));
    PWMBase->SM[2].VAL5 = PWM_VAL5_VAL5((uint16_t)(0));
    PWMBase->SM[3].VAL5 = PWM_VAL5_VAL5((uint16_t)(0));    

    /* PWM0 module 0 trigger on VAL4 enabled for ADC synchronization */
    PWMBase->SM[0].TCTRL |= PWM_TCTRL_OUT_TRIG_EN(1 << 4) | PWM_TCTRL_TRGFRQ(1);
    
    PWMBase->SM[3].TCTRL |= PWM_TCTRL_OUT_TRIG_EN(1 << 4) | PWM_TCTRL_TRGFRQ(1);

    /* Set dead-time register */
    PWMBase->SM[0].DTCNT0 = PWM_DTCNT0_DTCNT0(g_sClockSetup.ui16M2PwmDeadTime);
    PWMBase->SM[1].DTCNT0 = PWM_DTCNT0_DTCNT0(g_sClockSetup.ui16M2PwmDeadTime);
    PWMBase->SM[2].DTCNT0 = PWM_DTCNT0_DTCNT0(g_sClockSetup.ui16M2PwmDeadTime);
    PWMBase->SM[3].DTCNT0 = PWM_DTCNT0_DTCNT0(g_sClockSetup.ui16M3PwmDeadTime);    
    PWMBase->SM[0].DTCNT1 = PWM_DTCNT1_DTCNT1(g_sClockSetup.ui16M2PwmDeadTime);
    PWMBase->SM[1].DTCNT1 = PWM_DTCNT1_DTCNT1(g_sClockSetup.ui16M2PwmDeadTime);
    PWMBase->SM[2].DTCNT1 = PWM_DTCNT1_DTCNT1(g_sClockSetup.ui16M2PwmDeadTime);
    PWMBase->SM[3].DTCNT1 = PWM_DTCNT1_DTCNT1(g_sClockSetup.ui16M3PwmDeadTime);
    
    /* Channels A and B disabled when fault 0 occurs */
    PWMBase->SM[0].DISMAP[0] = ((PWMBase->SM[0].DISMAP[0] & ~PWM_DISMAP_DIS0A_MASK) | PWM_DISMAP_DIS0A(0x1));
    PWMBase->SM[1].DISMAP[0] = ((PWMBase->SM[1].DISMAP[0] & ~PWM_DISMAP_DIS0A_MASK) | PWM_DISMAP_DIS0A(0x1));
    PWMBase->SM[2].DISMAP[0] = ((PWMBase->SM[2].DISMAP[0] & ~PWM_DISMAP_DIS0A_MASK) | PWM_DISMAP_DIS0A(0x1));
    PWMBase->SM[3].DISMAP[0] = ((PWMBase->SM[3].DISMAP[0] & ~PWM_DISMAP_DIS0A_MASK) | PWM_DISMAP_DIS0A(0x4));
    
    PWMBase->SM[0].DISMAP[0] = ((PWMBase->SM[0].DISMAP[0] & ~PWM_DISMAP_DIS0B_MASK) | PWM_DISMAP_DIS0B(0x1));
    PWMBase->SM[1].DISMAP[0] = ((PWMBase->SM[1].DISMAP[0] & ~PWM_DISMAP_DIS0B_MASK) | PWM_DISMAP_DIS0B(0x1));
    PWMBase->SM[2].DISMAP[0] = ((PWMBase->SM[2].DISMAP[0] & ~PWM_DISMAP_DIS0B_MASK) | PWM_DISMAP_DIS0B(0x1));
    PWMBase->SM[3].DISMAP[0] = ((PWMBase->SM[3].DISMAP[0] & ~PWM_DISMAP_DIS0B_MASK) | PWM_DISMAP_DIS0B(0x4));

    /* Modules one and two gets clock from module zero */
    PWMBase->SM[1].CTRL2 = (PWMBase->SM[1].CTRL2 & ~PWM_CTRL2_CLK_SEL_MASK) | PWM_CTRL2_CLK_SEL(0x2);
    PWMBase->SM[2].CTRL2 = (PWMBase->SM[2].CTRL2 & ~PWM_CTRL2_CLK_SEL_MASK) | PWM_CTRL2_CLK_SEL(0x2);
    PWMBase->SM[3].CTRL2 = (PWMBase->SM[3].CTRL2 & ~PWM_CTRL2_CLK_SEL_MASK) | PWM_CTRL2_CLK_SEL(0x2);    

    /* Master reload active for modules one and two */
    PWMBase->SM[1].CTRL2 |= PWM_CTRL2_RELOAD_SEL_MASK;
    PWMBase->SM[2].CTRL2 |= PWM_CTRL2_RELOAD_SEL_MASK;

    /* Master reload is generated every one opportunity */
    PWMBase->SM[0].CTRL = (PWMBase->SM[0].CTRL & ~PWM_CTRL_LDFQ_MASK) | PWM_CTRL_LDFQ(M2_FOC_FREQ_VS_PWM_FREQ - 1);

    /* FORCE active for SM0~2*/
    PWMBase->SM[0].CTRL2 = (PWMBase->SM[0].CTRL2 & ~PWM_CTRL2_FORCE_SEL_MASK) | PWM_CTRL2_FORCE_SEL(6U) | PWM_CTRL2_FRCEN_MASK;
    PWMBase->SM[1].CTRL2 = (PWMBase->SM[1].CTRL2 & ~PWM_CTRL2_FORCE_SEL_MASK) | PWM_CTRL2_FORCE_SEL(6U) | PWM_CTRL2_FRCEN_MASK;
    PWMBase->SM[2].CTRL2 = (PWMBase->SM[2].CTRL2 & ~PWM_CTRL2_FORCE_SEL_MASK) | PWM_CTRL2_FORCE_SEL(6U) | PWM_CTRL2_FRCEN_MASK;
    PWMBase->SM[3].CTRL2 = (PWMBase->SM[3].CTRL2 & ~PWM_CTRL2_INIT_SEL_MASK) | PWM_CTRL2_INIT_SEL(3U);

    /* Fault 0 active in logic level one, automatic clearing */
    PWMBase->FCTRL = (PWMBase->FCTRL & ~PWM_FCTRL_FLVL_MASK) | PWM_FCTRL_FLVL(0x1)| PWM_FCTRL_FLVL(0x4);
    PWMBase->FCTRL = (PWMBase->FCTRL & ~PWM_FCTRL_FAUTO_MASK) | PWM_FCTRL_FAUTO(0x1)| PWM_FCTRL_FAUTO(0x4);

    /* Clear fault flags */
    PWMBase->FSTS = (PWMBase->FSTS & ~PWM_FSTS_FFLAG_MASK) | PWM_FSTS_FFLAG(0xF);

    /* PWMs are re-enabled at PWM full cycle */
    PWMBase->FSTS = (PWMBase->FSTS & ~PWM_FSTS_FFULL_MASK) | PWM_FSTS_FFULL(0x1)| PWM_FSTS_FFULL(0x4);

    /* PWM fault filter - 5 Fast peripheral clocks sample rate, 5 agreeing samples to activate */
    PWMBase->FFILT = (PWMBase->FFILT & ~PWM_FFILT_FILT_PER_MASK) | PWM_FFILT_FILT_PER(5);
    PWMBase->FFILT = (PWMBase->FFILT & ~PWM_FFILT_FILT_CNT_MASK) | PWM_FFILT_FILT_CNT(5);
    
    /* Enable A&B PWM outputs for submodules zero, one and two */
    PWMBase->OUTEN = (PWMBase->OUTEN & ~PWM_OUTEN_PWMA_EN_MASK) | PWM_OUTEN_PWMA_EN(0xF);
    PWMBase->OUTEN = (PWMBase->OUTEN & ~PWM_OUTEN_PWMB_EN_MASK) | PWM_OUTEN_PWMB_EN(0xF);

    /* Start PWMs (set load OK flags and run) */
    PWMBase->MCTRL = (PWMBase->MCTRL & ~PWM_MCTRL_CLDOK_MASK) | PWM_MCTRL_CLDOK(0xF);
    PWMBase->MCTRL = (PWMBase->MCTRL & ~PWM_MCTRL_LDOK_MASK) | PWM_MCTRL_LDOK(0xF);
    PWMBase->MCTRL = (PWMBase->MCTRL & ~PWM_MCTRL_RUN_MASK) | PWM_MCTRL_RUN(0xF);

    /* Initialize MC driver */
    g_sM2Pwm3ph.pui32PwmBaseAddress = (PWM_Type *)PWMBase;
    g_sM3Pwm3ph.pui32PwmBaseAddress = (PWM_Type *)PWMBase;    

    g_sM2Pwm3ph.ui16PhASubNum = 0U; /* PWMA phase A sub-module number */
    g_sM2Pwm3ph.ui16PhBSubNum = 1U; /* PWMA phase B sub-module number */
    g_sM2Pwm3ph.ui16PhCSubNum = 2U; /* PWMA phase C sub-module number */

    g_sM2Pwm3ph.ui16FaultFixNum = M2_FAULT_NUM; /* PWMA fixed-value over-current fault number */
    g_sM2Pwm3ph.ui16FaultAdjNum = M2_FAULT_NUM; /* PWMA adjustable over-current fault number */

    g_sM3Pwm3ph.ui16FaultFixNum = M3_FAULT_NUM; /* PWMA fixed-value over-current fault number */
    g_sM3Pwm3ph.ui16FaultAdjNum = M3_FAULT_NUM; /* PWMA adjustable over-current fault number */
}

/*!
 * @brief   void InitADC1(void)
 *           - Initialization of the ADC1 peripheral
 *           - Initialization of the A/D converter for current and voltage sensing
 *
 * @param   void
 *
 * @return  none
 */
static void InitADC1(void)
{
  lpadc_config_t lpadcConfig;

  /* Init the lpadcConfig */
  LPADC_GetDefaultConfig(&lpadcConfig);
  lpadcConfig.enableAnalogPreliminary = true;
  lpadcConfig.powerLevelMode = kLPADC_PowerLevelAlt4;
  lpadcConfig.referenceVoltageSource = kLPADC_ReferenceVoltageAlt3;
  lpadcConfig.conversionAverageMode = kLPADC_ConversionAverage1;

  /* Init ADC */
  lpadc_conv_trigger_config_t lpadcTriggerConfig;
  lpadc_conv_command_config_t lpadcCommandConfig;

  LPADC_Init(ADC1, &lpadcConfig);

  LPADC_DoOffsetCalibration(ADC1);
  LPADC_DoAutoCalibration(ADC1);
  
  LPADC_GetDefaultConvCommandConfig(&lpadcCommandConfig);
  lpadcCommandConfig.sampleChannelMode = kLPADC_SampleChannelSingleEndSideA;
  lpadcCommandConfig.conversionResolutionMode = kLPADC_ConversionResolutionStandard;
  lpadcCommandConfig.hardwareAverageMode = kLPADC_HardwareAverageCount1;
  lpadcCommandConfig.sampleTimeMode = kLPADC_SampleTimeADCK3;  
  
  /* Init commands */
  /* Set conversion CMD1 configuration. */
  lpadcCommandConfig.channelNumber = 8U;                                /* Set ADC channel M2_IPHA */
  lpadcCommandConfig.chainedNextCommandNumber = 2U;                           
  LPADC_SetConvCommandConfig( ADC1, 1U, &lpadcCommandConfig );                

  /* Set conversion CMD2 configuration. */                     
  lpadcCommandConfig.channelNumber = 9U;                                /* Set ADC channel M2_IPHB */                                      
  lpadcCommandConfig.chainedNextCommandNumber = 3U;
  LPADC_SetConvCommandConfig( ADC1, 2U, &lpadcCommandConfig );                
  
  /* Set conversion CMD3 configuration. */                     
  lpadcCommandConfig.channelNumber = 10U;                                /* Set ADC channel M2_IPHC */                                      
  lpadcCommandConfig.chainedNextCommandNumber = 4U;
  LPADC_SetConvCommandConfig( ADC1, 3U, &lpadcCommandConfig );                
  
  /* Set conversion CMD4 configuration. */
  lpadcCommandConfig.channelNumber = 11U;                                /* Set ADC channel M2_DCB */
  lpadcCommandConfig.chainedNextCommandNumber = 0U;                           
  LPADC_SetConvCommandConfig( ADC1, 4U, &lpadcCommandConfig );                
  
  /* Init triggers (use trigger 0). */
  LPADC_GetDefaultConvTriggerConfig(&lpadcTriggerConfig);
  lpadcTriggerConfig.targetCommandId = 1U;
  lpadcTriggerConfig.enableHardwareTrigger = true; 
  LPADC_SetConvTriggerConfig(ADC1, 0U, &lpadcTriggerConfig);
  
  /* Set conversion CMD5 configuration. */
  lpadcCommandConfig.channelNumber = 13U;                                      /* Set ADC channel M3_IPHA */
  lpadcCommandConfig.chainedNextCommandNumber = 6U;                           
  LPADC_SetConvCommandConfig( ADC1, 5U, &lpadcCommandConfig );
  
  /* Set conversion CMD6 configuration. */
  lpadcCommandConfig.channelNumber = 0U;                                      /* Set ADC channel M3_IPHB */
  lpadcCommandConfig.chainedNextCommandNumber = 7U;                           
  LPADC_SetConvCommandConfig( ADC1, 6U, &lpadcCommandConfig );

  /* Set conversion CMD7 configuration. */
  lpadcCommandConfig.channelNumber = 21U;                                      /* Set ADC channel M3_DCB */
  lpadcCommandConfig.chainedNextCommandNumber = 0U;                           
  LPADC_SetConvCommandConfig( ADC1, 7U, &lpadcCommandConfig );
  
 /* Init triggers (use trigger 1). */
  LPADC_GetDefaultConvTriggerConfig(&lpadcTriggerConfig);
  lpadcTriggerConfig.targetCommandId = 5U;
  lpadcTriggerConfig.enableHardwareTrigger = true; 
  LPADC_SetConvTriggerConfig(ADC1, 1U, &lpadcTriggerConfig);  
  
  /* Enable TCOMP interrupt. */
  LPADC_EnableInterrupts(ADC1, ADC_IE_TCOMP_IE(0x3U));//Only trigger source 0 trigger the interrupt
  NVIC_SetPriority(ADC1_IRQn, 4U);
  NVIC_EnableIRQ(ADC1_IRQn);
  
  g_sM2AdcSensor.pToAdcBase = ADC1;  
  g_sM3AdcSensor.pToAdcBase = ADC1;    
}

/*!
 * @brief   void InitSlowLoop(void)
 *           - Initialization of the CTIMER0 peripheral
 *           - performs slow control loop counter
 *
 * @param   void
 *
 * @return  none
 */
#define LPTMR_SOURCE_CLOCK (12000000U)
static void InitSlowLoop(void)
{    
    lptmr_config_t lptmrConfig;
    /* Configure LPTMR */
    /*
     * lptmrConfig.timerMode = kLPTMR_TimerModeTimeCounter;
     * lptmrConfig.pinSelect = kLPTMR_PinSelectInput_0;
     * lptmrConfig.pinPolarity = kLPTMR_PinPolarityActiveHigh;
     * lptmrConfig.enableFreeRunning = false;
     * lptmrConfig.bypassPrescaler = true;
     * lptmrConfig.prescalerClockSource = kLPTMR_PrescalerClock_1;
     * lptmrConfig.value = kLPTMR_Prescale_Glitch_0;
     */
    /* Note: the input clock source for prescaler clock must be enabled and attached in advance with configuration in SYSCON or SCG */
    LPTMR_GetDefaultConfig(&lptmrConfig);
    lptmrConfig.prescalerClockSource = kLPTMR_PrescalerClock_3;
    /* Initialize the LPTMR */
    LPTMR_Init(LPTMR0, &lptmrConfig);

    /*
     * Set timer period.
     * Note : the parameter "ticks" of LPTMR_SetTimerPeriod should be equal or greater than 1.
     */
    LPTMR_SetTimerPeriod(LPTMR0, USEC_TO_COUNT(1000/PFC_SLOW_LOOP_FREQ, LPTMR_SOURCE_CLOCK));

    /* Enable timer interrupt */
    LPTMR_EnableInterrupts(LPTMR0, kLPTMR_TimerInterruptEnable);

    /* Enable at the NVIC */
    EnableIRQ(LPTMR0_IRQn);

    /* Start counting */
    LPTMR_StartTimer(LPTMR0);
}

static void InitCTIMER4(void)
{
    CLOCK_EnableClock(kCLOCK_GateCTIMER4);
    RESET_PeripheralReset(kCTIMER4_RST_SHIFT_RSTn);
    
    CLOCK_SetClockDiv(kCLOCK_DivCTIMER4, 1U);
    CLOCK_AttachClk(kFRO_HF_to_CTIMER4);
    
    CTIMER4->TCR |= CTIMER_TCR_ATCEN_MASK;
    
    CTIMER4->MR[0] = PFC_PWM_MODULO / 2 + 120;
    CTIMER4->MR[1] = PFC_PWM_MODULO + 120;
    CTIMER4->MR[2] = PFC_PWM_MODULO + 400U;
    CTIMER4->MR[3] = PFC_PWM_MODULO + PFC_PWM_MODULO - 240U;
    
    CTIMER4->MCR |= CTIMER_MCR_MR3S_MASK | CTIMER_MCR_MR2I_MASK;
    
    CTIMER4->PWMC |= CTIMER_PWMC_PWMEN0_MASK | CTIMER_PWMC_PWMEN1_MASK;
    
    NVIC_SetPriority(CTIMER4_IRQn, 4U);
    NVIC_EnableIRQ(CTIMER4_IRQn);

}

/*!
* @brief   void InitClock(void)
*          - Core, bus, flash clock setup
*
* @param   void
*
* @return  none
*/
static void InitClock(void)
{
    uint32_t ui32CyclesNumber = 0U;

    /* Calculate clock dependant variables for PMSM control algorithm */
    g_sClockSetup.ui32FastPeripheralClock = CLOCK_GetFreq(kCLOCK_CoreSysClk);
    g_sClockSetup.ui32CpuFrequency = CLOCK_GetFreq(kCLOCK_CoreSysClk);

    /* Parameters for motor 1 */
    g_sClockSetup.ui16M1PwmFreq   = M1_PWM_FREQ; /* 6.25 kHz */
    g_sClockSetup.ui16M1PwmModulo = (g_sClockSetup.ui32FastPeripheralClock) / g_sClockSetup.ui16M1PwmFreq;
    ui32CyclesNumber = ((M1_PWM_DEADTIME * (g_sClockSetup.ui32FastPeripheralClock / 1000000U)) / 1000U);
    g_sClockSetup.ui16M1PwmDeadTime   = ui32CyclesNumber;
    g_sClockSetup.ui16M1SpeedLoopFreq = M1_SPEED_LOOP_FREQ; /* 1kHz */

    /* Parameters for motor 2 */
    g_sClockSetup.ui16M2PwmFreq   = M2_PWM_FREQ; /* 8 kHz */
    g_sClockSetup.ui16M2PwmModulo = (g_sClockSetup.ui32FastPeripheralClock) / g_sClockSetup.ui16M2PwmFreq;
    ui32CyclesNumber = ((M2_PWM_DEADTIME * (g_sClockSetup.ui32FastPeripheralClock / 1000000U)) / 1000U);
    g_sClockSetup.ui16M2PwmDeadTime   = ui32CyclesNumber;
    
    /* Parameters for motor 3 */
    g_sClockSetup.ui16M3PwmFreq   = M3_PWM_FREQ; /* 8 kHz */
    g_sClockSetup.ui16M3PwmModulo = (g_sClockSetup.ui32FastPeripheralClock) / g_sClockSetup.ui16M2PwmFreq;
    ui32CyclesNumber = ((M3_PWM_DEADTIME * (g_sClockSetup.ui32FastPeripheralClock / 1000000U)) / 1000U);
    g_sClockSetup.ui16M3PwmDeadTime   = ui32CyclesNumber;   
}

static void InitAOI0(void)
{
    aoi_event_config_t aoiEventLogicStruct;
    
    /* Configure the AOI event */
    aoiEventLogicStruct.PT0AC = kAOI_InputSignal;
    aoiEventLogicStruct.PT0BC = kAOI_LogicOne;
    aoiEventLogicStruct.PT0CC = kAOI_LogicOne;
    aoiEventLogicStruct.PT0DC = kAOI_LogicOne;

    aoiEventLogicStruct.PT1AC = kAOI_LogicZero;
    aoiEventLogicStruct.PT1BC = kAOI_LogicZero;
    aoiEventLogicStruct.PT1CC = kAOI_LogicZero;
    aoiEventLogicStruct.PT1DC = kAOI_LogicZero;

    aoiEventLogicStruct.PT2AC = kAOI_LogicZero;
    aoiEventLogicStruct.PT2BC = kAOI_LogicZero;
    aoiEventLogicStruct.PT2CC = kAOI_LogicZero;
    aoiEventLogicStruct.PT2DC = kAOI_LogicZero;

    aoiEventLogicStruct.PT3AC = kAOI_LogicZero;
    aoiEventLogicStruct.PT3BC = kAOI_LogicZero;
    aoiEventLogicStruct.PT3CC = kAOI_LogicZero;
    aoiEventLogicStruct.PT3DC = kAOI_LogicZero;

    /* Init AOI module. */
    AOI_Init(AOI0);
    AOI_SetEventLogicConfig(AOI0, kAOI_Event0, &aoiEventLogicStruct);
    AOI_SetEventLogicConfig(AOI0, kAOI_Event1, &aoiEventLogicStruct);
    AOI_SetEventLogicConfig(AOI0, kAOI_Event2, &aoiEventLogicStruct);    
}

static void InitAOI1(void)
{
    aoi_event_config_t aoiEventLogicStruct;
    
    /* Configure the AOI event */
    aoiEventLogicStruct.PT0AC = kAOI_InputSignal;
    aoiEventLogicStruct.PT0BC = kAOI_LogicOne;
    aoiEventLogicStruct.PT0CC = kAOI_LogicOne;
    aoiEventLogicStruct.PT0DC = kAOI_LogicOne;

    aoiEventLogicStruct.PT1AC = kAOI_LogicOne;
    aoiEventLogicStruct.PT1BC = kAOI_InputSignal;
    aoiEventLogicStruct.PT1CC = kAOI_LogicOne;
    aoiEventLogicStruct.PT1DC = kAOI_LogicOne;

    aoiEventLogicStruct.PT2AC = kAOI_LogicZero;
    aoiEventLogicStruct.PT2BC = kAOI_LogicZero;
    aoiEventLogicStruct.PT2CC = kAOI_LogicZero;
    aoiEventLogicStruct.PT2DC = kAOI_LogicZero;

    aoiEventLogicStruct.PT3AC = kAOI_LogicZero;
    aoiEventLogicStruct.PT3BC = kAOI_LogicZero;
    aoiEventLogicStruct.PT3CC = kAOI_LogicZero;
    aoiEventLogicStruct.PT3DC = kAOI_LogicZero;

    /* Init AOI module. */
    AOI_Init(AOI1);
    AOI_SetEventLogicConfig(AOI1, kAOI_Event0, &aoiEventLogicStruct);
    INPUTMUX_AttachSignal(INPUTMUX0, 1U, kINPUTMUX_Aoi1Out0ToExtTrigger);
}

static void InitUART0(void)
{
    lpuart_config_t config;
    /*
     * config.baudRate_Bps = 38400U;
     * config.parityMode = kLPUART_ParityDisabled;
     * config.stopBitCount = kLPUART_OneStopBit;
     * config.txFifoWatermark = 0;
     * config.rxFifoWatermark = 0;
     * config.enableTx = false;
     * config.enableRx = false;
     */
    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = 115200U;
    config.enableTx     = false;
    config.enableRx     = false;

    LPUART_Init((LPUART_Type *)LPUART3, &config, 12000000U);
    
    /* Register communication module used by FreeMASTER driver. */
    FMSTR_SerialSetBaseAddress((LPUART_Type *)LPUART3);
}

void InitInputmux(void)
{
    /* INPUTMUX0: Peripheral clock is enabled */
    CLOCK_EnableClock(kCLOCK_GateINPUTMUX0);
    /* INPUTMUX0 peripheral is released from reset */
    RESET_ReleasePeripheralReset(kINPUTMUX0_RST_SHIFT_RSTn);
        
    INPUTMUX_AttachSignal(INPUTMUX0, 0U, kINPUTMUX_Pwm0Sm0OutTrig0ToAdc0Trigger);
    
    INPUTMUX_AttachSignal(INPUTMUX0, 1U, kINPUTMUX_Ctimer4M0ToAdc0Trigger);
    INPUTMUX_AttachSignal(INPUTMUX0, 2U, kINPUTMUX_Ctimer4M1ToAdc0Trigger);
   
    INPUTMUX_AttachSignal(INPUTMUX0, 0U, kINPUTMUX_Pwm0Sm1OutTrig0ToAdc1Trigger);    
    INPUTMUX_AttachSignal(INPUTMUX0, 0U, kINPUTMUX_Pwm0Sm1OutTrig1ToFlexPwm1Force);
    
    INPUTMUX_AttachSignal(INPUTMUX0, 1U, kINPUTMUX_Pwm0Sm2OutTrig0ToAdc1Trigger);    
    INPUTMUX_AttachSignal(INPUTMUX0, 0U, kINPUTMUX_Pwm0Sm2OutTrig1ToFlexPwm1Sm3Extsync3);  
    
    INPUTMUX_AttachSignal(INPUTMUX0, 0U, kINPUTMUX_Pwm0Sm3OutTrig1ToAoi0Mux);
    
    
    INPUTMUX_AttachSignal(INPUTMUX0,0,kINPUTMUX_Pwm1Sm3OutTrig0ToTimer0Trigger);
    INPUTMUX_AttachSignal(INPUTMUX0,0,kINPUTMUX_Pwm1Sm3OutTrig0ToTimer1Trigger);
    INPUTMUX_AttachSignal(INPUTMUX0,0,kINPUTMUX_Pwm1Sm3OutTrig0ToTimer2Trigger);
    INPUTMUX_AttachSignal(INPUTMUX0,0,kINPUTMUX_Pwm1Sm3OutTrig0ToTimer3Trigger);
    
    INPUTMUX_AttachSignal(INPUTMUX0, 2U, kINPUTMUX_Cmp1OutToFlexPwm1Fault);
}

/* Recorder0 configuration */
FMSTR_U8 FreeMASTER_RecBuffer0[FREEMASTER_REC_0_SIZE];
FMSTR_REC_BUFF FreeMASTER_Recorder_0 = {
  .name = "Description of recorder 0",
  .addr = (FMSTR_ADDR)FreeMASTER_RecBuffer0,
  .size = (FMSTR_SIZE) sizeof(FreeMASTER_RecBuffer0),
  .basePeriod_ns = 160000UL
};
/*******************************************************************************
 * Public functions
 ******************************************************************************/
/*!
 * @brief   void MCDRV_Init(void)
 *           - Motor control driver main initialization
 *           - Calls initialization functions of peripherals required for motor
 *             control functionality
 *
 * @param   void
 *
 * @return  none
 */
void MCDRV_Init(void)
{
    /* Global initialization */
    (void)memset(DMA0_config.channelConfig, 0, FSL_FEATURE_EDMA_INSTANCE_CHANNELn(DMA0_DMA_BASEADDR) * sizeof(edma_channel_config_t *));
    EDMA_Init(DMA0, &DMA0_config);

    InitClock();                /* Init application clock dependent variables */
    InitSlowLoop();             /* Init slow loop counter */
    
    InitInputmux();

    InitADC1();                 /* Init ADC1 */
    InitOpAmps();               /* Init OPAMPS */
    InitLpCmp0();               /* Init Low Power Comparator 0 */
    InitLpCmp1();               /* Init Low Power Comparator 1 */
    InitPWM0();                 /* 6-channel PWM0 peripheral init */
    InitADC0();                 /* Init ADC0 */
    InitPWM1();                 /* 8-channel PWM1 peripheral init */
    
    InitAOI0();                  /* Init AOI for compressor sampling */
    DMA0_init();
    InitCTIMER0();
    InitCTIMER1(); 
    InitCTIMER2(); 
    InitCTIMER3();
    InitCTIMER4();
    DMA_EnableAllChannels();
    
    INPUTMUX_AttachSignal(INPUTMUX0, 0U, kINPUTMUX_Aoi0Out0ToTimer4Trigger);
    
    InitUART0();                /* Init UART1 for FreeMASTER */
    
    InitAOI1();                  /* Init AOI for PFC sampling */ 
}

void FMSTR_Recorder_Create()
{
    /* FreeMASTER recorder 0 configuration initialization  */
    FMSTR_RecorderCreate(0, &FreeMASTER_Recorder_0);
}
