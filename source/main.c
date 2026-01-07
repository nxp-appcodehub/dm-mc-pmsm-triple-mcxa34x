/*
 * Copyright 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "board.h"
#include "hardware_init.h"
#include "mc_periph_init.h"
#include "freemaster.h"
#include "freemaster_serial_lpuart.h"
#include "m1_sm_snsless.h"
#include "m2_sm_snsless.h"
#include "m3_sm_snsless.h"
#include "PFC_statemachine.h"
#include "fsl_lpuart.h"
#include "fsl_debug_console.h"
#include "fsl_lptmr.h"
#include "fsl_smartdma.h"
#include "fsl_smartdma_mcxa.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Version info */
#define MCRSP_VER "2.0.0" /* motor control package version */

/* Example's feature set in form of bits inside ui16featureSet.
   This feature set is expected to be growing over time.
   ... | FEATURE_S_RAMP | FEATURE_FIELD_WEAKENING | FEATURE_ENC
*/
#define FEATURE_ENC (1)               /* Encoder feature flag */
#define FEATURE_FIELD_WEAKENING (0)   /* Field weakening feature flag */
#define FEATURE_S_RAMP (0)            /* S-ramp feature flag */

#define FEATURE_SET (FEATURE_ENC << (0) | \
                     FEATURE_FIELD_WEAKENING << (1) | \
                     FEATURE_S_RAMP << (2))

/* CPU load measurement SysTick START / STOP macros */
#define SYSTICK_START_COUNT() (SysTick->VAL = SysTick->LOAD)
#define SYSTICK_STOP_COUNT(par1)   \
    uint32_t val  = SysTick->VAL;  \
    uint32_t load = SysTick->LOAD; \
    par1          = load - val

/* CPU load measurement using Systick */
uint32_t g_ui32NumberOfCycles    = 0U;
uint32_t g_ui32MaxNumberOfCycles = 0U;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void BOARD_InitSysTick(void);
void SmartDMA_pwm_fault_callback(void *param);
/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Structure used in FM to get required ID's */
app_ver_t g_sAppIdFM = {
    "MCXA346", /* board id */
    "X-MCXA346-3MC", /* example id */
    MCRSP_VER,      /* sw version */
    FEATURE_SET,    /* example's feature-set */
};

uint32_t        M1_isr_cnt=0;
uint32_t        M2_isr_cnt=0;
uint32_t        M3_isr_cnt=0;
uint32_t        PFC_isr_cnt=0;
uint32_t        Slow_isr_cnt=0;
/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Whether the SW button is pressed */
volatile bool g_ButtonPress = false;
smartdma_pwm_fault_param_t smartdmaParam;                  /*!< SMARTDMA function parameters. */
volatile uint8_t g_samrtdma_stack[32];
volatile uint32_t g_pwm_gpio_register[8] = {
		0x400BF000 + 0x80 + 25*4,/*PWM_AT, P3_27,Pin Control Register*/
		0x400BF000 + 0x80 + 26*4,/*PWM_AB, P3_28,Pin Control Register*/
		0x400BF000 + 0x80 + 21*4,/*PWM_BT, P3_21,Pin Control Register*/
		0x400BF000 + 0x80 + 22*4,/*PWM_BB, P3_22,Pin Control Register*/
		0x400BF000 + 0x80 + 19*4,/*PWM_CT, P3_19,Pin Control Register*/
		0x400BF000 + 0x80 + 20*4,/*PWM_CB, P3_20,Pin Control Register*/
};
/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
void main(void)
{
    /*Accessing ID structure to prevent optimization*/
    g_sAppIdFM.ui16FeatureSet = FEATURE_SET;
    
    SYSCON->NVM_CTRL &= ~SYSCON_NVM_CTRL_DIS_MBECC_ERR_DATA_MASK;
    SYSCON->LPCAC_CTRL |= SYSCON_LPCAC_CTRL_LPCAC_MEM_REQ_MASK;
    SYSCON->LPCAC_CTRL &= ~SYSCON_LPCAC_CTRL_DIS_LPCAC_MASK;
    
    uint32_t ui32PrimaskReg;

    /* Disable all interrupts before peripherals are initialized */
    ui32PrimaskReg = DisableGlobalIRQ();
    
    /* Board pin init */
    BOARD_InitHardware();
         
    SMARTDMA_InitWithoutFirmware();
    SMARTDMA_InstallFirmware(SMARTDMA_PWM_FAULT_MEM_ADDR,s_smartdmaPWMFaultFirmware,
    SMARTDMA_PWM_FAULT_FIRMWARE_SIZE);
    NVIC_EnableIRQ(SMARTDMA_IRQn);
    NVIC_SetPriority(SMARTDMA_IRQn, 3);
    SMARTDMA_InstallCallback(SmartDMA_pwm_fault_callback, NULL);
    smartdmaParam.smartdma_stack 	 = (uint32_t*)g_samrtdma_stack;
    smartdmaParam.p_gpio_reg  		 = (uint32_t*)g_pwm_gpio_register;
    SMARTDMA_Boot(kSMARTDMA_pwm_fault, &smartdmaParam, 0x2);    
    /* Initialize peripheral motor control and PFC driver */
    MCDRV_Init();
    BOARD_InitDebugConsole();
    
    /* FreeMASTER driver initialization */
    FMSTR_Init();
    FMSTR_Recorder_Create();
    
    /* SysTick initialization for CPU load measurement */
    BOARD_InitSysTick();
    
    //PWM counter must run just before IRQ enable
    FLEXPWM0->MCTRL = (FLEXPWM0->MCTRL & ~PWM_MCTRL_RUN_MASK) | PWM_MCTRL_RUN(0xF);
    FLEXPWM1->MCTRL = (FLEXPWM1->MCTRL & ~PWM_MCTRL_RUN_MASK) | PWM_MCTRL_RUN(0xF);
    
    /* Enable interrupts */
    EnableGlobalIRQ(ui32PrimaskReg);

    while (1)
    {
      /* FreeMASTER Polling function */
      FMSTR_Poll();
    }
}

//ADC0 interrupt for compressor sample, priority 4
void ADC0_IRQHandler(void)
{
    /* Clear the TCOMP INT flag */
    ADC0->STAT |= (uint32_t)(1U << 9);
    
    /* Transfer the UDCB voltage for Compressor control */
    g_sM1Drive.sFocPMSM.fltUDcBusFilt = gsPFC_Drive.sUCtrl.fltUDcBusFilt;

    /* StateMachine call */
    SM_StateMachineFast(&g_sM1Ctrl);

    /* Call FreeMASTER recorder */
    FMSTR_Recorder(0);
    __DSB();
    __ISB();
}

int16_t pwm0_counter;
void ADC1_IRQHandler(void)
{
    /* Clear the TCOMP INT flag */
    ADC1->STAT |= (uint32_t)(1U << 9);
    

    pwm0_counter = FLEXPWM0->SM[0].CNT;
    
    if(pwm0_counter<0)
    {
    	M2_isr_cnt++;

    	/* StateMachine call */
    	SM_StateMachineFast(&g_sM2Ctrl);
      
    }
    else
    {
      M3_isr_cnt++;

      /* StateMachine call */
      SM_StateMachineFast(&g_sM3Ctrl);

    }

    __DSB();
    __ISB();
    
}

int cntt=0;
void LPTMR0_IRQHandler(void)
{
	static int isr_cnt = 0;

	if(++isr_cnt==1000)
	{
		isr_cnt = 0;
		GPIO_PortToggle(GPIO2, 1<<25);
	}
  /* Clear the match interrupt flag. */
    LPTMR_ClearStatusFlags(LPTMR0, kLPTMR_TimerCompareFlag);

    if(4 == cntt++)
    {
      /* M1 Slow StateMachine call */
      SM_StateMachineSlow(&g_sM1Ctrl);
    
      /* M2 Slow StateMachine call */
      SM_StateMachineSlow(&g_sM2Ctrl);
      
      /* M3 Slow StateMachine call */
      SM_StateMachineSlow(&g_sM3Ctrl);

      cntt=0;      
    }
    
#if 1
    gsPFC_Drive.sUCtrl.fltUDcBus = gsPFC_Drive.sUInPeakDetection.fltUdcb;
    gsPFC_Drive.sUCtrl.fltUDcBusFilt = GDFLIB_FilterIIR1_FLT(gsPFC_Drive.sUCtrl.fltUDcBus, &gsPFC_Drive.sUCtrl.sUDcBusFilter);

    PFC_UInPeak_detect(&gsPFC_Drive);

    //PFC control
    SM_StateMachineSlow(&gsPFC_Ctrl);
    
#endif

    __DSB();
    __ISB();
}

void CTIMER4_IRQHandler(void)
{
    /* Clear the match interrupt flag. */
    CTIMER4->IR |= CTIMER_IR_MR2INT(1U);
   
    CTIMER4->TCR = CTIMER_TCR_CRST_MASK;
    CTIMER4->TCR = 0;
    CTIMER4->TCR = CTIMER_TCR_ATCEN_MASK;
    
    PFC_isr_cnt++;
    
    //GPIO4->PSOR |= 1<<4;
    
    PFC_MCDRV_ADC_GET(&g_sPFCAdcSensor);
      
    gsPFC_Drive.sICtrlPh1.fltIFdbck = 
      MLIB_ConvSc_FLTsf(MLIB_Sub_F16(gsPFC_Drive.sUInPeakDetection.f16Imos2, gsPFC_Drive.sUInPeakDetection.f16Offset2), g_fltPFCCurrentScale);
   
    gsPFC_Drive.sICtrlPh2.fltIFdbck = 
      MLIB_ConvSc_FLTsf(MLIB_Sub_F16(gsPFC_Drive.sUInPeakDetection.f16Imos1, gsPFC_Drive.sUInPeakDetection.f16Offset2), g_fltPFCCurrentScale);
    
    
    if(gsPFC_Drive.sICtrlPh1.fltIFdbck<0) gsPFC_Drive.sICtrlPh1.fltIFdbck=0;
    if(gsPFC_Drive.sICtrlPh2.fltIFdbck<0) gsPFC_Drive.sICtrlPh2.fltIFdbck=0;
    
    gsPFC_Drive.sUInPeakDetection.fltUInFiltRaw = GDFLIB_FilterIIR1_FLT(gsPFC_Drive.sUInPeakDetection.fltUIn, &gsPFC_Drive.sUInPeakDetection.sFilter);
    gsPFC_Drive.sUInPeakDetection.fltUInFilt = MLIB_Abs_FLT(gsPFC_Drive.sUInPeakDetection.fltUInFiltRaw);
    
    #if PLL
    PFC_Phase_detect(&gsPFC_Drive);
    #endif
      
    SM_StateMachineFast(&gsPFC_Ctrl);
    
    __DSB();
    __ISB();
}

static void BOARD_InitSysTick(void)
{
    /* Initialize SysTick core timer to run free */
    /* Set period to maximum value 2^24*/
    SysTick->LOAD = 0xFFFFFF;

    /*Clock source - System Clock*/
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;

    /*Start Sys Timer*/
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}
