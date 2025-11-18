/*
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "m3_sm_snsless.h"
#include "mc_periph_init.h"
#include "fsl_smartdma.h"
#include "fsl_smartdma_mcxa.h"
#include "pin_mux.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define M3_SVM_SECTOR_DEFAULT (2)        /* default SVM sector */
#define M3_BLOCK_ROT_FAULT_SH (0.03125F) /* filter window */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/


static void M3_StateFaultFast(void);

static void M3_StateInitFast(void);

static void M3_StateStopFast(void);

static void M3_StateRunFast(void);


static void M3_StateFaultSlow(void);

static void M3_StateInitSlow(void);

static void M3_StateStopSlow(void);

static void M3_StateRunSlow(void);


static void M3_TransFaultStop(void);

static void M3_TransInitFault(void);

static void M3_TransInitStop(void);

static void M3_TransStopFault(void);

static void M3_TransStopRun(void);

static void M3_TransRunFault(void);

static void M3_TransRunStop(void);


static void M3_StateRunCalibFast(void);

static void M3_StateRunReadyFast(void);

static void M3_StateRunAlignFast(void);

static void M3_StateRunStartupFast(void);

static void M3_StateRunSpinFast(void);

static void M3_StateRunFreewheelFast(void);


static void M3_StateRunCalibSlow(void);

static void M3_StateRunReadySlow(void);

static void M3_StateRunAlignSlow(void);

static void M3_StateRunStartupSlow(void);

static void M3_StateRunSpinSlow(void);

static void M3_StateRunFreewheelSlow(void);


static void M3_TransRunCalibReady(void);

static void M3_TransRunReadyAlign(void);

static void M3_TransRunAlignStartup(void);

static void M3_TransRunAlignReady(void);

static void M3_TransRunAlignSpin(void);

static void M3_TransRunStartupSpin(void);

static void M3_TransRunStartupFreewheel(void);

static void M3_TransRunSpinFreewheel(void);

static void M3_TransRunFreewheelReady(void);


static void M3_ClearFOCVariables(void);


static void M3_FaultDetection(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief Main control structure */
mcdef_pmsm_t g_sM3Drive;

/*! @brief Main application switch */
bool_t g_bM3SwitchAppOnOff;

/*! @brief M3 structure */
run_substate_t g_eM3StateRun;
run_substate_t g_eM3StateRun_causefault = 0;

/*! @brief FreeMASTER scales */
/*! DO NOT USE THEM in the code to avoid float library include */
volatile float g_fltM3voltageScale;
volatile float g_fltM3DCBvoltageScale;
volatile float g_fltM3currentScale;
volatile float g_fltM3speedScale;
volatile float g_fltM3speedAngularScale;
volatile float g_fltM3speedMechanicalScale;

/*! @brief Application state machine table - fast */
const sm_app_state_fcn_t s_M3_STATE_FAST = {M3_StateFaultFast, M3_StateInitFast, M3_StateStopFast, M3_StateRunFast};

/*! @brief Application state machine table - slow */
const sm_app_state_fcn_t s_M3_STATE_SLOW = {M3_StateFaultSlow, M3_StateInitSlow, M3_StateStopSlow, M3_StateRunSlow};

/*! @brief Application sub-state function field - fast */
static const pfc_app_state_fcn s_M3_STATE_RUN_TABLE_FAST[6] = {
    M3_StateRunCalibFast, M3_StateRunReadyFast,     M3_StateRunAlignFast,  M3_StateRunStartupFast,
    M3_StateRunSpinFast,  M3_StateRunFreewheelFast};

/*! @brief Application sub-state function field - slow */
static const pfc_app_state_fcn s_M3_STATE_RUN_TABLE_SLOW[6] = {
    M3_StateRunCalibSlow, M3_StateRunReadySlow,     M3_StateRunAlignSlow,  M3_StateRunStartupSlow,
    M3_StateRunSpinSlow,  M3_StateRunFreewheelSlow};

/*! @brief Application state-transition functions field  */
static const sm_app_trans_fcn_t s_TRANS = {M3_TransFaultStop, M3_TransInitFault, M3_TransInitStop, M3_TransStopFault,
                                           M3_TransStopRun,   M3_TransRunFault,  M3_TransRunStop};

/*! @brief  State machine structure declaration and initialization */
sm_app_ctrl_t g_sM3Ctrl = {
    /* g_sM3Ctrl.psState, User state functions  */
    &s_M3_STATE_FAST,

    /* g_sM3Ctrl.psState, User state functions  */
    &s_M3_STATE_SLOW,

    /* g_sM3Ctrl..psTrans, User state-transition functions */
    &s_TRANS,

    /* g_sM3Ctrl.uiCtrl, Default no control command */
    SM_CTRL_NONE,

    /* g_sM3Ctrl.eState, Default state after reset */
    kSM_AppInit};

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
void SmartDMA_pwm_fault_callback(void *param)
{
   g_eM3StateRun_causefault = g_eM3StateRun;
   GPIO2->PSOR |= 1<<25;
    /* Fault:   DC-bus over-current */  
   FAULT_SET(g_sM3Drive.sFaultIdPending, FAULT_I_DCBUS_OVER);
   
   g_sM3Drive.sFaultIdPending &= g_sM3Drive.sFaultIdEnable;
   /* Pass fault to Fault ID Captured */
   g_sM3Drive.sFaultIdCaptured |= g_sM3Drive.sFaultIdPending;
   
   g_sM3Ctrl.uiCtrl |= SM_CTRL_FAULT;
}

/*!
 * @brief Fault state called in fast state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */

static void M3_StateFaultFast(void)
{
    /* get all adc samples - DC-bus voltage, current, bemf and aux sample */
    M3_MCDRV_ADC_GET(&g_sM3AdcSensor);

    /* convert voltages from fractional measured values to float */
    g_sM3Drive.sFocPMSM.fltUDcBus = MLIB_ConvSc_FLTsf(g_sM3Drive.sFocPMSM.f16UDcBus, g_fltM3DCBvoltageScale);

    /* Sampled DC-Bus voltage filter */
    g_sM3Drive.sFocPMSM.fltUDcBusFilt =
        GDFLIB_FilterIIR1_FLT(g_sM3Drive.sFocPMSM.fltUDcBus, &g_sM3Drive.sFocPMSM.sUDcBusFilter);

    /* Braking resistor control with hysteresis */
    if (g_sM3Drive.sFocPMSM.fltUDcBusFilt > g_sM3Drive.sFaultThresholds.fltUDcBusTrip * (1.0F + M3_U_DCB_HYSTERESIS))
    {
    	M3_BRAKE_SET();
    }
    else if (g_sM3Drive.sFocPMSM.fltUDcBusFilt < g_sM3Drive.sFaultThresholds.fltUDcBusTrip * (1.0F - M3_U_DCB_HYSTERESIS))
    {
    	M3_BRAKE_CLEAR();
    }

    /* Disable user application switch */
    g_bM3SwitchAppOnOff = FALSE;

    /* PWM peripheral update */
    M3_MCDRV_PWM3PH_SET(&g_sM3Pwm3ph);

    /* Detects faults */
    M3_FaultDetection();
}

/*!
 * @brief State initialization routine called in fast state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */

static void M3_StateInitFast(void)
{
    /* Type the code to do when in the INIT state */
    g_sM3Drive.sFocPMSM.sIdPiParams.fltInErrK_1 = 0.0F;
    g_sM3Drive.sFocPMSM.sIdPiParams.bLimFlag    = FALSE;

    g_sM3Drive.sFocPMSM.sIqPiParams.fltInErrK_1 = 0.0F;
    g_sM3Drive.sFocPMSM.sIqPiParams.bLimFlag    = FALSE;

    /* PMSM FOC params */
    g_sM3Drive.sFocPMSM.sIdPiParams.fltPGain    = M2_D_KP_GAIN;
    g_sM3Drive.sFocPMSM.sIdPiParams.fltIGain    = M2_D_KI_GAIN;
    g_sM3Drive.sFocPMSM.sIdPiParams.fltUpperLim = M2_U_MAX;
    g_sM3Drive.sFocPMSM.sIdPiParams.fltLowerLim = -M2_U_MAX;

    g_sM3Drive.sFocPMSM.sIqPiParams.fltPGain    = M2_Q_KP_GAIN;
    g_sM3Drive.sFocPMSM.sIqPiParams.fltIGain    = M2_Q_KI_GAIN;
    g_sM3Drive.sFocPMSM.sIqPiParams.fltUpperLim = M2_U_MAX;
    g_sM3Drive.sFocPMSM.sIqPiParams.fltLowerLim = -M2_U_MAX;

    g_sM3Drive.sFocPMSM.ui16SectorSVM     = M3_SVM_SECTOR_DEFAULT;
    g_sM3Drive.sFocPMSM.fltDutyCycleLimit = M2_CLOOP_LIMIT;

    g_sM3Drive.sFocPMSM.fltUDcBus                     = 0.0F;
    g_sM3Drive.sFocPMSM.fltUDcBusFilt                 = 0.0F;
    g_sM3Drive.sFocPMSM.sUDcBusFilter.sFltCoeff.fltB0 = M2_UDCB_IIR_B0;
    g_sM3Drive.sFocPMSM.sUDcBusFilter.sFltCoeff.fltB1 = M2_UDCB_IIR_B1;
    g_sM3Drive.sFocPMSM.sUDcBusFilter.sFltCoeff.fltA1 = M2_UDCB_IIR_A1;
    /* Filter init not to enter to fault */
    g_sM3Drive.sFocPMSM.sUDcBusFilter.fltFltBfrX[0] = (M2_U_DCB_UNDERVOLTAGE / 2.0F) + (M2_U_DCB_OVERVOLTAGE / 2.0F);
    g_sM3Drive.sFocPMSM.sUDcBusFilter.fltFltBfrY[0] = (M2_U_DCB_UNDERVOLTAGE / 2.0F) + (M2_U_DCB_OVERVOLTAGE / 2.0F);

    g_sM3Drive.sAlignment.fltUdReq = M2_ALIGN_VOLTAGE;
    g_sM3Drive.sAlignment.ui16Time = M2_ALIGN_DURATION;

    /* Position and speed observer */
    g_sM3Drive.sFocPMSM.sTo.fltPGain  = M2_TO_KP_GAIN;
    g_sM3Drive.sFocPMSM.sTo.fltIGain  = M2_TO_KI_GAIN;
    g_sM3Drive.sFocPMSM.sTo.fltThGain = M2_TO_THETA_GAIN;

    g_sM3Drive.sFocPMSM.sBemfObsrv.fltIGain       = M2_I_SCALE;
    g_sM3Drive.sFocPMSM.sBemfObsrv.fltUGain       = M2_U_SCALE;
    g_sM3Drive.sFocPMSM.sBemfObsrv.fltEGain       = M2_E_SCALE;
    g_sM3Drive.sFocPMSM.sBemfObsrv.fltWIGain      = M2_WI_SCALE;
    g_sM3Drive.sFocPMSM.sBemfObsrv.sCtrl.fltPGain = M2_BEMF_DQ_KP_GAIN;
    g_sM3Drive.sFocPMSM.sBemfObsrv.sCtrl.fltIGain = M2_BEMF_DQ_KI_GAIN;

    g_sM3Drive.sFocPMSM.sSpeedElEstFilt.sFltCoeff.fltB0 = M2_TO_SPEED_IIR_B0;
    g_sM3Drive.sFocPMSM.sSpeedElEstFilt.sFltCoeff.fltB1 = M2_TO_SPEED_IIR_B1;
    g_sM3Drive.sFocPMSM.sSpeedElEstFilt.sFltCoeff.fltA1 = M2_TO_SPEED_IIR_A1;
    GDFLIB_FilterIIR1Init_FLT(&g_sM3Drive.sFocPMSM.sSpeedElEstFilt);

    /* Speed params */
    g_sM3Drive.sSpeed.sSpeedPiParams.fltPGain    = M2_SPEED_PI_PROP_GAIN;
    g_sM3Drive.sSpeed.sSpeedPiParams.fltIGain    = M2_SPEED_PI_INTEG_GAIN;
    g_sM3Drive.sSpeed.sSpeedPiParams.fltUpperLim = M2_SPEED_LOOP_HIGH_LIMIT;
    g_sM3Drive.sSpeed.sSpeedPiParams.fltLowerLim = M2_SPEED_LOOP_LOW_LIMIT;

    g_sM3Drive.sSpeed.sSpeedRampParams.fltRampUp   = M2_SPEED_RAMP_UP;
    g_sM3Drive.sSpeed.sSpeedRampParams.fltRampDown = M2_SPEED_RAMP_DOWN;

    g_sM3Drive.sSpeed.sSpeedFilter.sFltCoeff.fltB0 = M2_SPEED_IIR_B0;
    g_sM3Drive.sSpeed.sSpeedFilter.sFltCoeff.fltB1 = M2_SPEED_IIR_B1;
    g_sM3Drive.sSpeed.sSpeedFilter.sFltCoeff.fltA1 = M2_SPEED_IIR_A1;

    g_sM3Drive.sSpeed.fltSpeedCmd = 0.0F;

    /* Position params */
    //g_sM3Drive.sPosition.f16PositionPGain = M2_POS_P_PROP_GAIN;
    g_sM3Drive.sPosition.a32Position      = ACC32(0.0);
    g_sM3Drive.sPosition.a32PositionError = ACC32(0.0);
    g_sM3Drive.sPosition.a32PositionCmd   = ACC32(0.0);
    g_sM3Drive.sPosition.f16SpeedReq      = FRAC16(0.0);

    /* Scalar control params */
    g_sM3Drive.sScalarCtrl.fltVHzGain                  = M2_SCALAR_VHZ_FACTOR_GAIN;
    g_sM3Drive.sScalarCtrl.fltUqMin                    = M2_SCALAR_UQ_MIN;
    g_sM3Drive.sScalarCtrl.sFreqRampParams.fltRampUp   = M2_SCALAR_RAMP_UP;
    g_sM3Drive.sScalarCtrl.sFreqRampParams.fltRampDown = M2_SCALAR_RAMP_DOWN;
    g_sM3Drive.sScalarCtrl.sFreqIntegrator.a32Gain     = M2_SCALAR_INTEG_GAIN;
    g_sM3Drive.sScalarCtrl.fltFreqMax                  = M2_FREQ_MAX;

    /* Open loop start up */
    g_sM3Drive.sStartUp.sSpeedIntegrator.a32Gain             = M2_SCALAR_INTEG_GAIN;
    g_sM3Drive.sStartUp.f16CoeffMerging                      = M2_MERG_COEFF;
    g_sM3Drive.sStartUp.fltSpeedCatchUp                      = M2_MERG_SPEED_TRH;
    g_sM3Drive.sStartUp.fltCurrentStartup                    = M2_OL_START_I;
    g_sM3Drive.sStartUp.sSpeedRampOpenLoopParams.fltRampUp   = M2_OL_START_RAMP_INC;
    g_sM3Drive.sStartUp.sSpeedRampOpenLoopParams.fltRampDown = M2_OL_START_RAMP_INC;
    g_sM3Drive.sStartUp.fltSpeedMax                          = M2_N_MAX;
    g_sM3Drive.sStartUp.bOpenLoop                            = TRUE;

    /* MCAT cascade control variables */
    g_sM3Drive.sMCATctrl.sIDQReqMCAT.fltD = 0.0F;
    g_sM3Drive.sMCATctrl.sIDQReqMCAT.fltQ = 0.0F;
    g_sM3Drive.sMCATctrl.sUDQReqMCAT.fltD = 0.0F;
    g_sM3Drive.sMCATctrl.sUDQReqMCAT.fltQ = 0.0F;
    g_sM3Drive.sMCATctrl.ui16PospeSensor  = MCAT_SENSORLESS_CTRL;

    /* Timing control and general variables */
    g_sM3Drive.ui16CounterState           = 0U;
    g_sM3Drive.ui16TimeFullSpeedFreeWheel = M2_FREEWHEEL_DURATION;
    g_sM3Drive.ui16TimeCalibration        = M2_CALIB_DURATION;
    g_sM3Drive.ui16TimeFaultRelease       = M2_FAULT_DURATION;
    g_bM3SwitchAppOnOff                   = FALSE;
    /* Default MCAT control mode after reset */
    g_sM3Drive.eControl = kControlMode_SpeedFOC;

    /* fault set to init states */
    FAULT_CLEAR_ALL(g_sM3Drive.sFaultIdCaptured);
    FAULT_CLEAR_ALL(g_sM3Drive.sFaultIdPending);

    /* fault thresholds */
    g_sM3Drive.sFaultThresholds.fltUDcBusOver     = M2_U_DCB_OVERVOLTAGE;
    g_sM3Drive.sFaultThresholds.fltUDcBusUnder    = M2_U_DCB_UNDERVOLTAGE;
    g_sM3Drive.sFaultThresholds.fltUDcBusTrip     = M2_U_DCB_TRIP;
    g_sM3Drive.sFaultThresholds.fltSpeedOver      = M2_N_OVERSPEED;
    g_sM3Drive.sFaultThresholds.fltSpeedMin       = M2_N_MIN;
    g_sM3Drive.sFaultThresholds.fltSpeedNom       = M2_N_NOM;
    g_sM3Drive.sFaultThresholds.fltUqBemf         = M2_E_BLOCK_TRH;
    g_sM3Drive.sFaultThresholds.ui16BlockedPerNum = M2_E_BLOCK_PER;

    /* fault blocked rotor filter */
    g_sM3Drive.msBlockedRotorUqFilt.fltLambda = M3_BLOCK_ROT_FAULT_SH;

    /* Defined scaling for FreeMASTER */
    g_fltM3voltageScale         = M2_U_MAX;
    g_fltM3currentScale         = M2_I_MAX;
    g_fltM3DCBvoltageScale      = M2_U_DCB_MAX;
    g_fltM3speedScale           = M2_N_MAX;
    g_fltM3speedAngularScale    = M2_N_ANGULAR_MAX;
    g_fltM3speedMechanicalScale = (60.0F / (2.0F * FLOAT_PI));

    /* Application timing */
    g_sM3Drive.ui16FastCtrlLoopFreq = (g_sClockSetup.ui16M3PwmFreq / ((uint16_t)M3_FOC_FREQ_VS_PWM_FREQ));
    g_sM3Drive.ui16SlowCtrlLoopFreq = g_sClockSetup.ui16M1SpeedLoopFreq;
    g_sM3Drive.ui32CpuFrequency = g_sClockSetup.ui32CpuFrequency;

    /* Clear rest of variables  */
    M3_ClearFOCVariables();

    /* Init sensors/actuators pointers */
    /* For PWM driver */
    g_sM3Pwm3ph.psUABC = &(g_sM3Drive.sFocPMSM.sDutyABC);
    /* For ADC driver */
    g_sM3AdcSensor.pf16UDcBus     = &(g_sM3Drive.sFocPMSM.f16UDcBus);
    g_sM3AdcSensor.psIABC         = &(g_sM3Drive.sFocPMSM.sIABCFrac);
    g_sM3AdcSensor.pui16SVMSector = &(g_sM3Drive.sFocPMSM.ui16SectorSVM);
    g_sM3AdcSensor.pui16AuxChan   = &(g_sM3Drive.f16AdcAuxSample);

    /* For ENC driver */
    //g_sM3Enc.pf16PosElEst = &(g_sM3Drive.f16PosElEnc);
    //g_sM3Enc.pfltSpdMeEst = &(g_sM3Drive.fltSpeedEnc);

    /* INIT_DONE command */
    g_sM3Ctrl.uiCtrl |= SM_CTRL_INIT_DONE;

    /* Enable all MC faults */
    FAULT_SET(g_sM3Drive.sFaultIdEnable, FAULT_I_DCBUS_OVER);
    FAULT_SET(g_sM3Drive.sFaultIdEnable, FAULT_U_DCBUS_UNDER);
    FAULT_SET(g_sM3Drive.sFaultIdEnable, FAULT_U_DCBUS_OVER);
    FAULT_SET(g_sM3Drive.sFaultIdEnable, FAULT_LOAD_OVER);
    FAULT_SET(g_sM3Drive.sFaultIdEnable, FAULT_SPEED_OVER);
    FAULT_SET(g_sM3Drive.sFaultIdEnable, FAULT_ROTOR_BLOCKED);
}

/*!
 * @brief Stop state routine called in fast state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */

static void M3_StateStopFast(void)
{
    /* get all adc samples - DC-bus voltage, current, bemf and aux sample */
    M3_MCDRV_ADC_GET(&g_sM3AdcSensor);

    /* Set encoder direction */
    //M3_MCDRV_QD_SET_DIRECTION(&g_sM3Enc);

    /* get position and speed from quadrature encoder sensor */
    //M3_MCDRV_QD_GET(&g_sM3Enc);

    /* convert voltages from fractional measured values to float */
    g_sM3Drive.sFocPMSM.fltUDcBus = MLIB_ConvSc_FLTsf(g_sM3Drive.sFocPMSM.f16UDcBus, g_fltM3DCBvoltageScale);

    /* Sampled DC-Bus voltage filter */
    g_sM3Drive.sFocPMSM.fltUDcBusFilt =
        GDFLIB_FilterIIR1_FLT(g_sM3Drive.sFocPMSM.fltUDcBus, &g_sM3Drive.sFocPMSM.sUDcBusFilter);

    /* If the user switches on or set non-zero speed*/
    if ((g_bM3SwitchAppOnOff != FALSE) || (g_sM3Drive.sSpeed.fltSpeedCmd != 0.0F))
    {
        /* Set the switch on */
        g_bM3SwitchAppOnOff = TRUE;

        /* Start command */
        g_sM3Ctrl.uiCtrl |= SM_CTRL_START;
    }

    /* Braking resistor control with hysteresis */
    if (g_sM3Drive.sFocPMSM.fltUDcBusFilt > g_sM3Drive.sFaultThresholds.fltUDcBusTrip * (1.0F + M3_U_DCB_HYSTERESIS))
    {
    	M3_BRAKE_SET();
    }
    else if (g_sM3Drive.sFocPMSM.fltUDcBusFilt < g_sM3Drive.sFaultThresholds.fltUDcBusTrip * (1.0F - M3_U_DCB_HYSTERESIS))
    {
    	M3_BRAKE_CLEAR();
    }

    M3_FaultDetection();

    /* If a fault occurred */
    if ((bool_t)g_sM3Drive.sFaultIdPending)
    {
        /* Switches to the FAULT state */
        g_sM3Ctrl.uiCtrl |= SM_CTRL_FAULT;
    }

    /* PWM peripheral update */
    M3_MCDRV_PWM3PH_SET(&g_sM3Pwm3ph);
}

uint16_t testduty=0;
uint16_t dir=0;
frac16_t f16duty=FRAC16(0.2);
/*!
 * @brief Run state routine called in fast state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_LIB
static void M3_StateRunFast(void)
{
    /* get all adc samples - DC-bus voltage, current, bemf and aux sample */
    M3_MCDRV_ADC_GET(&g_sM3AdcSensor);

    /* get position and speed from quadrature encoder sensor */
    //M3_MCDRV_QD_GET(&g_sM3Enc);

    /* If the user switches off */
    if (!g_bM3SwitchAppOnOff)
    {
        /* Stop command */
        g_sM3Ctrl.uiCtrl |= SM_CTRL_STOP;

        g_sM3Drive.sPosition.a32PositionCmd = 0;
        g_sM3Drive.sPosition.a32Position    = 0;
    }

    /* detect fault */
    M3_FaultDetection();

    /* If a fault occurred */
    if (g_sM3Drive.sFaultIdPending != 0U)
    {
        /* Switches to the FAULT state */
        g_sM3Ctrl.uiCtrl |= SM_CTRL_FAULT;
    }

    /* Convert phase currents from fractional measured values to float */
    g_sM3Drive.sFocPMSM.sIABC.fltA = MLIB_ConvSc_FLTsf(g_sM3Drive.sFocPMSM.sIABCFrac.f16A, g_fltM3currentScale);
    g_sM3Drive.sFocPMSM.sIABC.fltB = MLIB_ConvSc_FLTsf(g_sM3Drive.sFocPMSM.sIABCFrac.f16B, g_fltM3currentScale);
    g_sM3Drive.sFocPMSM.sIABC.fltC = MLIB_ConvSc_FLTsf(g_sM3Drive.sFocPMSM.sIABCFrac.f16C, g_fltM3currentScale);

    /* Convert voltages from fractional measured values to float */
    g_sM3Drive.sFocPMSM.fltUDcBus = MLIB_ConvSc_FLTsf(g_sM3Drive.sFocPMSM.f16UDcBus, g_fltM3DCBvoltageScale);

    /* Sampled DC-Bus voltage filter */
    g_sM3Drive.sFocPMSM.fltUDcBusFilt =
        GDFLIB_FilterIIR1_FLT(g_sM3Drive.sFocPMSM.fltUDcBus, &g_sM3Drive.sFocPMSM.sUDcBusFilter);

    /* Braking resistor control with hysteresis */
    if (g_sM3Drive.sFocPMSM.fltUDcBusFilt > g_sM3Drive.sFaultThresholds.fltUDcBusTrip * (1.0F + M3_U_DCB_HYSTERESIS))
    {
    	M3_BRAKE_SET();
    }
    else if (g_sM3Drive.sFocPMSM.fltUDcBusFilt < g_sM3Drive.sFaultThresholds.fltUDcBusTrip * (1.0F - M3_U_DCB_HYSTERESIS))
    {
    	M3_BRAKE_CLEAR();
    }

    /* Run sub-state function */
    s_M3_STATE_RUN_TABLE_FAST[g_eM3StateRun]();
#if 0
    testduty++;
    if(testduty>800)
    {
      if(dir==0)
      {
        f16duty=MLIB_AddSat_F16(f16duty,FRAC16(0.01));
        if(f16duty>FRAC16(0.8))dir=1;
      }
      else
      {
        f16duty=MLIB_Sub_F16(f16duty,FRAC16(0.01));
        if(f16duty<FRAC16(0.2))dir=0;
      }
      
      
      g_sM3Pwm3ph.psUABC->f16A = f16duty;
      g_sM3Pwm3ph.psUABC->f16B = f16duty;
      g_sM3Pwm3ph.psUABC->f16C = f16duty;
      testduty=200;
    }
#endif
    /* PWM peripheral update */
    M3_MCDRV_PWM3PH_SET(&g_sM3Pwm3ph);

    /* Set current sensor for sampling - applies only to some devices. */
    //M3_MCDRV_CURR_3PH_CHAN_ASSIGN(&g_sM3AdcSensor);
}

/*!
 * @brief Fault state routine called in slow state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */

static void M3_StateFaultSlow(void)
{
    /* After fault condition ends wait defined time to clear fault state */
    if (!FAULT_ANY(g_sM3Drive.sFaultIdPending))
    {
        if (--g_sM3Drive.ui16CounterState == 0U)
        {
            /* Clear fault state */
            g_sM3Ctrl.uiCtrl |= SM_CTRL_FAULT_CLEAR;
        }
    }
    else
    {
        g_sM3Drive.ui16CounterState = g_sM3Drive.ui16TimeFaultRelease;
    }
}

/*!
 * @brief Fault state routine called in slow state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */

static void M3_StateInitSlow(void)
{
}

/*!
 * @brief Stop state routine called in slow state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */

static void M3_StateStopSlow(void)
{
}

/*!
 * @brief Run state routine called in slow state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_LIB
static void M3_StateRunSlow(void)
{
    /* Run sub-state function */
    s_M3_STATE_RUN_TABLE_SLOW[g_eM3StateRun]();
}

/*!
 * @brief Transition from Fault to Stop state
 *
 * @param void  No input parameter
 *
 * @return None
 */

static void M3_TransFaultStop(void)
{
    /* Type the code to do when going from the FAULT to the INIT state */
    /* Clear all FOC variables, init filters, etc. */
    M3_ClearFOCVariables();
    if(g_sM3Drive.sFaultIdCaptured & (1<<FAULT_I_DCBUS_OVER))
    {
       BOARD_SmartDMA0Pins();
    }         
}

/*!
 * @brief Transition from Init to Fault state
 *
 * @param void  No input parameter
 *
 * @return None
 */

static void M3_TransInitFault(void)
{
    /* Type the code to do when going from the INIT to the FAULT state */
    /* Disable PWM outputs */
    M3_MCDRV_PWM3PH_DIS(&g_sM3Pwm3ph);
    g_sM3Drive.ui16CounterState = g_sM3Drive.ui16TimeFaultRelease;

    g_sM3Drive.sSpeed.fltSpeedCmd = 0.0F;
}

/*!
 * @brief Transition from Init to Stop state
 *
 * @param void  No input parameter
 *
 * @return None
 */

static void M3_TransInitStop(void)
{
    /* Type the code to do when going from the INIT to the STOP state */
    /* Disable PWM outputs */
    M3_MCDRV_PWM3PH_DIS(&g_sM3Pwm3ph);

    /* Enable Open loop start up */
    g_sM3Drive.sStartUp.bOpenLoop = TRUE;
}

/*!
 * @brief Transition from Stop to Fault state
 *
 * @param void  No input parameter
 *
 * @return None
 */

static void M3_TransStopFault(void)
{
    /* Type the code to do when going from the STOP to the FAULT state */
    /* Load the fault release time to counter */
    g_sM3Drive.ui16CounterState = g_sM3Drive.ui16TimeFaultRelease;
}

/*!
 * @brief Transition from Stop to Run state
 *
 * @param void  No input parameter
 *
 * @return None
 */

static void M3_TransStopRun(void)
{
    /* Type the code to do when going from the STOP to the RUN state */
    /* 50% duty cycle */
    g_sM3Drive.sFocPMSM.sDutyABC.f16A = FRAC16(0.5);
    g_sM3Drive.sFocPMSM.sDutyABC.f16B = FRAC16(0.5);
    g_sM3Drive.sFocPMSM.sDutyABC.f16C = FRAC16(0.5);

    /* PWM duty cycles calculation and update */
    M3_MCDRV_PWM3PH_SET(&g_sM3Pwm3ph);

    /* Clear offset filters */
    M3_MCDRV_CURR_3PH_CALIB_INIT(&g_sM3AdcSensor);

    /* Enable PWM output */
    M3_MCDRV_PWM3PH_EN(&g_sM3Pwm3ph);

    /* pass calibration routine duration to state counter*/
    g_sM3Drive.ui16CounterState = g_sM3Drive.ui16TimeCalibration;

    /* Calibration sub-state when transition to RUN */
    g_eM3StateRun = kRunState_Calib;

    /* Acknowledge that the system can proceed into the RUN state */
    g_sM3Ctrl.uiCtrl |= SM_CTRL_RUN_ACK;
}

/*!
 * @brief Transition from Run to Fault state
 *
 * @param void  No input parameter
 *
 * @return None
 */

static void M3_TransRunFault(void)
{
    /* Type the code to do when going from the RUN to the FAULT state */
    /* Disable PWM output */
    M3_MCDRV_PWM3PH_DIS(&g_sM3Pwm3ph);
    g_sM3Drive.ui16CounterState = g_sM3Drive.ui16TimeFaultRelease;

    /* Clear over load flag */
    g_sM3Drive.sSpeed.bSpeedPiStopInteg = FALSE;

    g_sM3Drive.sSpeed.fltSpeedCmd         = 0.0F;
    g_sM3Drive.sScalarCtrl.fltFreqCmd     = 0.0F;
    g_sM3Drive.sScalarCtrl.sUDQReq.fltQ   = 0.0F;
    g_sM3Drive.sMCATctrl.sUDQReqMCAT.fltQ = 0.0F;
    g_sM3Drive.sMCATctrl.sUDQReqMCAT.fltD = 0.0F;
    g_sM3Drive.sMCATctrl.sIDQReqMCAT.fltQ = 0.0F;
    g_sM3Drive.sMCATctrl.sIDQReqMCAT.fltD = 0.0F;

    /* Clear actual speed values */
    g_sM3Drive.sScalarCtrl.fltFreqRamp = 0.0F;
    g_sM3Drive.sSpeed.fltSpeed         = 0.0F;
    g_sM3Drive.sSpeed.fltSpeedFilt     = 0.0F;
}

/*!
 * @brief Transition from Run to Stop state
 *
 * @param void  No input parameter
 *
 * @return None
 */

static void M3_TransRunStop(void)
{
    /* Type the code to do when going from the RUN to the STOP state */
    /* Disable PWM outputs */
    M3_MCDRV_PWM3PH_DIS(&g_sM3Pwm3ph);

    g_sM3Drive.sSpeed.fltSpeedCmd         = 0.0F;
    g_sM3Drive.sScalarCtrl.fltFreqCmd     = 0.0F;
    g_sM3Drive.sScalarCtrl.sUDQReq.fltQ   = 0.0F;
    g_sM3Drive.sMCATctrl.sUDQReqMCAT.fltQ = 0.0F;
    g_sM3Drive.sMCATctrl.sUDQReqMCAT.fltD = 0.0F;
    g_sM3Drive.sMCATctrl.sIDQReqMCAT.fltQ = 0.0F;
    g_sM3Drive.sMCATctrl.sIDQReqMCAT.fltD = 0.0F;

    M3_ClearFOCVariables();

    /* Acknowledge that the system can proceed into the STOP state */
    g_sM3Ctrl.uiCtrl |= SM_CTRL_STOP_ACK;
}

/*!
 * @brief Calibration process called in fast state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */

static void M3_StateRunCalibFast(void)
{
    /* Type the code to do when in the RUN CALIB sub-state
       performing ADC offset calibration */

    /* Call offset measurement */
    //M3_MCDRV_CURR_3PH_CALIB(&g_sM3AdcSensor);
    
    g_sM3AdcSensor.sCurrSec45.ui16CalibPhaA =
      GDFLIB_FilterMA_F16((frac16_t)(g_sM3AdcSensor.ui16AdcCurrA), &g_sM3AdcSensor.sCurrSec45.ui16FiltPhaA);
    g_sM3AdcSensor.sCurrSec45.ui16CalibPhaB =
      GDFLIB_FilterMA_F16((frac16_t)(g_sM3AdcSensor.ui16AdcCurrB), &g_sM3AdcSensor.sCurrSec45.ui16FiltPhaB);

}

/*!
 * @brief Ready state called in fast state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */

static void M3_StateRunReadyFast(void)
{
    /* Type the code to do when in the RUN READY sub-state */
    /* Clear actual speed values */
    g_sM3Drive.sScalarCtrl.fltFreqRamp = 0.0F;
    g_sM3Drive.sSpeed.fltSpeed         = 0.0F;
    g_sM3Drive.sSpeed.fltSpeedFilt     = 0.0F;
    g_sM3Drive.sFocPMSM.f16PosElEst    = FRAC16(0.0);
    g_sM3Drive.sFocPMSM.fltSpeedElEst  = 0.0F;

    /* MCAT control structure switch */
    switch (g_sM3Drive.eControl)
    {
        case kControlMode_Scalar:
            if (!(g_sM3Drive.sScalarCtrl.fltFreqCmd == 0.0F))
            {
                g_sM3Drive.sScalarCtrl.fltFreqRamp  = 0.0F;
                g_sM3Drive.sScalarCtrl.sUDQReq.fltQ = 0.0F;

                /* Transition to the RUN ALIGN sub-state */
                M3_TransRunReadyAlign();
            }
            break;

        case kControlMode_VoltageFOC:
            if (!(g_sM3Drive.sMCATctrl.sUDQReqMCAT.fltQ == 0.0F))
            {
                if (g_sM3Drive.sMCATctrl.sUDQReqMCAT.fltQ > 0.0F)
                {
                	g_sM3Drive.sSpeed.fltSpeedCmd = g_sM3Drive.sStartUp.fltSpeedCatchUp * 2.0F;
                }
                else
                {
                	g_sM3Drive.sSpeed.fltSpeedCmd = MLIB_Neg_FLT(g_sM3Drive.sStartUp.fltSpeedCatchUp * 2.0F);
                }

                /* Transition to the RUN ALIGN sub-state */
                M3_TransRunReadyAlign();
            }
            break;

        case kControlMode_CurrentFOC:
            if (!(g_sM3Drive.sMCATctrl.sIDQReqMCAT.fltQ == 0.0F))
            {
                if (g_sM3Drive.sMCATctrl.sIDQReqMCAT.fltQ > 0.0F)
                {
                	g_sM3Drive.sSpeed.fltSpeedCmd = g_sM3Drive.sStartUp.fltSpeedCatchUp * 2.0F;
                }
                else
                {
                	g_sM3Drive.sSpeed.fltSpeedCmd = MLIB_Neg_FLT(g_sM3Drive.sStartUp.fltSpeedCatchUp * 2.0F);
                }

                /* Transition to the RUN ALIGN sub-state */
                M3_TransRunReadyAlign();
            }
            break;

        default:
            /* Align function */
            /* Value of fltSpeedNom is increased because of float conversion */
            if ((g_sM3Drive.sMCATctrl.ui16PospeSensor == MCAT_ENC_CTRL) ||
                ((MLIB_Abs_FLT(g_sM3Drive.sSpeed.fltSpeedCmd) > g_sM3Drive.sFaultThresholds.fltSpeedMin) &&
                 (MLIB_Abs_FLT(g_sM3Drive.sSpeed.fltSpeedCmd) <= (g_sM3Drive.sFaultThresholds.fltSpeedNom + 0.1F))))
            {
                /* Transition to the RUN ALIGN sub-state */
                M3_TransRunReadyAlign();
            }
            else
            {
                g_sM3Drive.sSpeed.fltSpeedCmd = 0.0F;
            }
            break;
    }
}

/*!
 * @brief Alignment process called in fast state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */


static void M3_StateRunAlignFast(void)
{
    /* Type the code to do when in the RUN ALIGN sub-state */
    /* When alignment elapsed go to Startup */
    if (--g_sM3Drive.ui16CounterState == 0U)
    {
        if ((g_sM3Drive.sMCATctrl.ui16PospeSensor == MCAT_ENC_CTRL) && (g_sM3Drive.eControl != kControlMode_Scalar))
        {
            /* Transition to the RUN kRunState_Spin sub-state */
            M3_TransRunAlignSpin();
        }
        else
        {
            /* Transition to the RUN kRunState_Startup sub-state */
            M3_TransRunAlignStartup();
        }
    }

    /* If zero speed command go back to Ready */
    if ((g_sM3Drive.sMCATctrl.ui16PospeSensor == MCAT_SENSORLESS_CTRL) && (g_sM3Drive.sSpeed.fltSpeedCmd == 0.0F) &&
        (g_sM3Drive.sScalarCtrl.fltFreqCmd == 0.0F))
    {
    	M3_TransRunAlignReady();
    }

    /* Clear actual speed values */
    g_sM3Drive.sScalarCtrl.fltFreqRamp = 0.0F;
    g_sM3Drive.sSpeed.fltSpeed         = 0.0F;
    g_sM3Drive.sSpeed.fltSpeedFilt     = 0.0F;
    g_sM3Drive.sFocPMSM.f16PosElEst    = FRAC16(0.0);
    g_sM3Drive.sFocPMSM.fltSpeedElEst  = 0.0F;
//g_sM3Drive.sFocPMSM.sUDQReq.fltD = g_sM3Drive.sAlignment.fltUdReq;
    
    MCS_PMSMAlignment(&g_sM3Drive.sAlignment);
    g_sM3Drive.sFocPMSM.f16PosElExt = g_sM3Drive.sAlignment.f16PosAlign;
    MCS_PMSMFocCtrl(&g_sM3Drive.sFocPMSM);
}

/*!
 * @brief Start-up process called in fast state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_LIB
static void M3_StateRunStartupFast(void)
{
    /* If f16SpeedCmd = 0, go to Free-wheel state */
    if ((g_sM3Drive.sSpeed.fltSpeedCmd == 0.0F) && (g_sM3Drive.eControl == kControlMode_SpeedFOC))
    {
    	M3_TransRunStartupFreewheel();
    }

    /* Type the code to do when in the RUN STARTUP sub-state */
    /* Pass actual estimation position to OL startup structure */
    g_sM3Drive.sStartUp.f16PosEst = g_sM3Drive.sFocPMSM.f16PosElEst;

    /* Open loop startup */
    MCS_PMSMOpenLoopStartUp(&g_sM3Drive.sStartUp);

    /* Pass f16SpeedRampOpenloop to f16SpeedRamp*/
    g_sM3Drive.sSpeed.fltSpeedRamp = g_sM3Drive.sStartUp.fltSpeedRampOpenLoop;

    /* Position and speed for FOC */
    g_sM3Drive.sFocPMSM.f16PosElExt = g_sM3Drive.sStartUp.f16PosMerged;

    /* MCAT control structure switch */
    switch (g_sM3Drive.eControl)
    {
        case kControlMode_Scalar:
            /* Switch directly to SPIN state */
            M3_TransRunStartupSpin();
            break;

        case kControlMode_VoltageFOC:
            /* Pass MCAT required values in run-time */
            g_sM3Drive.sFocPMSM.sUDQReq.fltD = g_sM3Drive.sMCATctrl.sUDQReqMCAT.fltD;
            g_sM3Drive.sFocPMSM.sUDQReq.fltQ = g_sM3Drive.sMCATctrl.sUDQReqMCAT.fltQ;
            /* FOC */
            g_sM3Drive.sFocPMSM.bCurrentLoopOn = FALSE;
            MCS_PMSMFocCtrl(&g_sM3Drive.sFocPMSM);
            break;

        case kControlMode_CurrentFOC:
            /* FOC */
            g_sM3Drive.sFocPMSM.sIDQReq.fltD   = g_sM3Drive.sMCATctrl.sIDQReqMCAT.fltD;
            g_sM3Drive.sFocPMSM.sIDQReq.fltQ   = g_sM3Drive.sMCATctrl.sIDQReqMCAT.fltQ;
            g_sM3Drive.sFocPMSM.bCurrentLoopOn = TRUE;
            MCS_PMSMFocCtrl(&g_sM3Drive.sFocPMSM);
            break;

        default:
            /* Current control loop */
            g_sM3Drive.sFocPMSM.sIDQReq.fltD = 0.0F;

            /* During the open loop start up the values of required Iq current are kept in pre-defined level*/
            if (g_sM3Drive.sSpeed.fltSpeedCmd > 0.0F)
            {
            	g_sM3Drive.sFocPMSM.sIDQReq.fltQ = g_sM3Drive.sStartUp.fltCurrentStartup;
            }
            else
            {
            	g_sM3Drive.sFocPMSM.sIDQReq.fltQ = MLIB_Neg_FLT(g_sM3Drive.sStartUp.fltCurrentStartup);
            }

            /* Init Bemf observer if open-loop speed is under SpeedCatchUp/2 */
            if (MLIB_Abs_FLT(g_sM3Drive.sStartUp.fltSpeedRampOpenLoop) < (g_sM3Drive.sStartUp.fltSpeedCatchUp / 2.0F))
            {
                AMCLIB_PMSMBemfObsrvDQInit_A32fff(&g_sM3Drive.sFocPMSM.sBemfObsrv);
                AMCLIB_TrackObsrvInit_A32af(ACC32(0.0), &g_sM3Drive.sFocPMSM.sTo);
            }

            /* FOC */
            g_sM3Drive.sFocPMSM.bCurrentLoopOn = TRUE;
            MCS_PMSMFocCtrl(&g_sM3Drive.sFocPMSM);

            /* select source of actual speed value */
            if (g_sM3Drive.sMCATctrl.ui16PospeSensor == MCAT_ENC_CTRL)
            {
                /* pass encoder speed to actual speed value */
               // g_sM3Drive.sSpeed.fltSpeed = g_sM3Drive.fltSpeedEnc * ((float_t)(g_sM3Enc.ui16Pp));
            }
            else
            {
                /* pass estimated speed to actual speed value */
                g_sM3Drive.sSpeed.fltSpeed = g_sM3Drive.sFocPMSM.fltSpeedElEst;
            }
            break;

    }

    /* Switch to close loop  */
    if (!g_sM3Drive.sStartUp.bOpenLoop)
    {
        M3_TransRunStartupSpin();
    }
}

/*!
 * @brief Spin state called in fast state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_LIB
static void M3_StateRunSpinFast(void)
{
    /* Type the code to do when in the RUN SPIN sub-state */
    /* MCAT control structure switch */
    switch (g_sM3Drive.eControl)
    {
        case kControlMode_Scalar:
            /* Scalar control */
            MCS_PMSMScalarCtrl(&g_sM3Drive.sScalarCtrl);

            /* Pass required voltages to Bemf Observer to work */
            g_sM3Drive.sFocPMSM.sUDQReq.fltQ = g_sM3Drive.sScalarCtrl.sUDQReq.fltQ;
            g_sM3Drive.sFocPMSM.sUDQReq.fltD = g_sM3Drive.sScalarCtrl.sUDQReq.fltD;
            g_sM3Drive.sFocPMSM.f16PosElExt  = g_sM3Drive.sScalarCtrl.f16PosElScalar;

            /* Call voltage FOC to calculate PWM duty cycles */
            MCS_PMSMFocCtrl(&g_sM3Drive.sFocPMSM);

            /* Sub-state RUN FREEWHEEL */
            if (g_sM3Drive.sScalarCtrl.fltFreqCmd == 0.0F)
            {
            	M3_TransRunSpinFreewheel();
            }
            break;

        case kControlMode_VoltageFOC:
            /* FOC */
            g_sM3Drive.sFocPMSM.sUDQReq.fltQ   = g_sM3Drive.sMCATctrl.sUDQReqMCAT.fltQ;
            g_sM3Drive.sFocPMSM.sUDQReq.fltD   = g_sM3Drive.sMCATctrl.sUDQReqMCAT.fltD;
            g_sM3Drive.sFocPMSM.bCurrentLoopOn = FALSE;

            /* Pass encoder position to FOC is enabled */
            if (g_sM3Drive.sMCATctrl.ui16PospeSensor == MCAT_ENC_CTRL)
            {
                g_sM3Drive.sFocPMSM.f16PosElExt = g_sM3Drive.f16PosElEnc;
                g_sM3Drive.sFocPMSM.bPosExtOn   = TRUE;
            }
            else
            {
                g_sM3Drive.sFocPMSM.bPosExtOn = FALSE;
            }

            MCS_PMSMFocCtrl(&g_sM3Drive.sFocPMSM);

            /* Sub-state RUN FREEWHEEL */
            if (g_sM3Drive.sMCATctrl.sUDQReqMCAT.fltQ == 0.0F)
            {
            	M3_TransRunSpinFreewheel();
            }
            break;

        case kControlMode_CurrentFOC:
            /* Current FOC */
            g_sM3Drive.sFocPMSM.sIDQReq.fltQ = g_sM3Drive.sMCATctrl.sIDQReqMCAT.fltQ;
            g_sM3Drive.sFocPMSM.sIDQReq.fltD = g_sM3Drive.sMCATctrl.sIDQReqMCAT.fltD;

            /* Pass encoder position to FOC is enabled */
            if (g_sM3Drive.sMCATctrl.ui16PospeSensor == MCAT_ENC_CTRL)
            {
                g_sM3Drive.sFocPMSM.f16PosElExt = g_sM3Drive.f16PosElEnc;
                g_sM3Drive.sFocPMSM.bPosExtOn   = TRUE;
            }
            else
            {
                g_sM3Drive.sFocPMSM.bPosExtOn = FALSE;
            }

            g_sM3Drive.sFocPMSM.bCurrentLoopOn = TRUE;
            MCS_PMSMFocCtrl(&g_sM3Drive.sFocPMSM);

            /* Sub-state RUN FREEWHEEL */
            if (g_sM3Drive.sMCATctrl.sIDQReqMCAT.fltQ == 0.0F)
            {
            	M3_TransRunSpinFreewheel();
            }
            break;

        case kControlMode_SpeedFOC:
        case kControlMode_PositionFOC:
        default:
            if ((MLIB_Abs_FLT(g_sM3Drive.sSpeed.fltSpeedRamp) < g_sM3Drive.sFaultThresholds.fltSpeedMin) &&
                (g_sM3Drive.sMCATctrl.ui16PospeSensor == MCAT_SENSORLESS_CTRL))
            {
                /* Sub-state RUN FREEWHEEL */
                M3_TransRunSpinFreewheel();
            }

            /* Pass encoder position to FOC is enabled */
            if (g_sM3Drive.sMCATctrl.ui16PospeSensor == MCAT_ENC_CTRL)
            {
                g_sM3Drive.sFocPMSM.f16PosElExt = g_sM3Drive.f16PosElEnc;
                g_sM3Drive.sFocPMSM.bPosExtOn   = TRUE;
            }
            else
            {
                g_sM3Drive.sFocPMSM.bPosExtOn = FALSE;
            }

            /* FOC */
            g_sM3Drive.sFocPMSM.bCurrentLoopOn = TRUE;
            MCS_PMSMFocCtrl(&g_sM3Drive.sFocPMSM);

            /* select source of actual speed value */
            if (g_sM3Drive.sMCATctrl.ui16PospeSensor == MCAT_ENC_CTRL)
            {
                /* pass encoder speed to actual speed value */
               // g_sM3Drive.sSpeed.fltSpeed = g_sM3Drive.fltSpeedEnc * ((float_t)(g_sM3Enc.ui16Pp));
            }
            else
            {
                /* Pass estimated speed to actual speed value */
                g_sM3Drive.sSpeed.fltSpeed = g_sM3Drive.sFocPMSM.fltSpeedElEst;
            }
            break;
    }
}

/*!
 * @brief Free-wheel process called in fast state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */

static void M3_StateRunFreewheelFast(void)
{
    /* Type the code to do when in the RUN FREEWHEEL sub-state */

    /* Clear actual speed values */
    g_sM3Drive.sScalarCtrl.fltFreqRamp = 0.0F;
    g_sM3Drive.sSpeed.fltSpeed         = 0.0F;
    g_sM3Drive.sSpeed.fltSpeedFilt     = 0.0F;
    g_sM3Drive.sSpeed.fltSpeedRamp     = 0.0F;
}

/*!
 * @brief Calibration process called in slow state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */

static void M3_StateRunCalibSlow(void)
{
    /* Write calibrated offset values */
    //M3_MCDRV_CURR_3PH_CALIB_SET(&g_sM3AdcSensor);
      /* pass calibration data for sector 4 and 5 */
    g_sM3AdcSensor.sCurrSec45.ui16OffsetPhaA = g_sM3AdcSensor.sCurrSec45.ui16CalibPhaA;
    g_sM3AdcSensor.sCurrSec45.ui16OffsetPhaB = g_sM3AdcSensor.sCurrSec45.ui16CalibPhaB;

    if (--g_sM3Drive.ui16CounterState == 0U)
    {
      /* To switch to the RUN READY sub-state */
      M3_TransRunCalibReady();
    }
}

/*!
 * @brief Ready state called in slow state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */

static void M3_StateRunReadySlow(void)
{
}

/*!
 * @brief Alignment process called in slow state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */

static void M3_StateRunAlignSlow(void)
{
}

/*!
 * @brief Start-up process called in slow state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_LIB
static void M3_StateRunStartupSlow(void)
{
    if (g_sM3Drive.eControl == kControlMode_SpeedFOC)
    {
        /* Actual speed filter */
        g_sM3Drive.sSpeed.fltSpeedFilt =
            GDFLIB_FilterIIR1_FLT(g_sM3Drive.sSpeed.fltSpeed, &g_sM3Drive.sSpeed.sSpeedFilter);

        /* Pass required speed values lower than nominal speed */
        if ((MLIB_Abs_FLT(g_sM3Drive.sSpeed.fltSpeedCmd) > g_sM3Drive.sFaultThresholds.fltSpeedNom))
        {
            /* Set required speed to nominal speed if over speed command > speed nominal */
            if (g_sM3Drive.sSpeed.fltSpeedCmd > 0.0F)
            {
            	g_sM3Drive.sSpeed.fltSpeedCmd = g_sM3Drive.sFaultThresholds.fltSpeedNom;
            }
            else
            {
            	g_sM3Drive.sSpeed.fltSpeedCmd = MLIB_Neg_FLT(g_sM3Drive.sFaultThresholds.fltSpeedNom);
            }
        }
    }
}

/*!
 * @brief Spin state called in slow state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_LIB
static void M3_StateRunSpinSlow(void)
{
    if (g_sM3Drive.eControl == kControlMode_SpeedFOC)
    {
        /* Actual speed filter */
        g_sM3Drive.sSpeed.fltSpeedFilt =
            GDFLIB_FilterIIR1_FLT(g_sM3Drive.sSpeed.fltSpeed, &g_sM3Drive.sSpeed.sSpeedFilter);

        /* Pass required speed values lower than nominal speed */
        if ((MLIB_Abs_FLT(g_sM3Drive.sSpeed.fltSpeedCmd) > g_sM3Drive.sFaultThresholds.fltSpeedNom))
        {
            /* Set required speed to nominal speed if over speed command > speed nominal */
            if (g_sM3Drive.sSpeed.fltSpeedCmd > 0.0F)
            {
            	g_sM3Drive.sSpeed.fltSpeedCmd = g_sM3Drive.sFaultThresholds.fltSpeedNom;
            }
            else
            {
            	g_sM3Drive.sSpeed.fltSpeedCmd = MLIB_Neg_FLT(g_sM3Drive.sFaultThresholds.fltSpeedNom);
            }

        }

        if ((MLIB_Abs_FLT(g_sM3Drive.sSpeed.fltSpeedRamp) < g_sM3Drive.sFaultThresholds.fltSpeedMin) &&
            (g_sM3Drive.sMCATctrl.ui16PospeSensor == MCAT_SENSORLESS_CTRL))
        {
        	M3_TransRunSpinFreewheel();
        }

        /* call PMSM speed control */
        g_sM3Drive.sSpeed.bIqPiLimFlag = g_sM3Drive.sFocPMSM.sIqPiParams.bLimFlag;
        MCS_PMSMFocCtrlSpeed(&g_sM3Drive.sSpeed);
        g_sM3Drive.sFocPMSM.sIDQReq.fltQ = g_sM3Drive.sSpeed.fltIqReq;
    }

    if (g_sM3Drive.eControl == kControlMode_PositionFOC)
    {
        /* actual speed filter */
        g_sM3Drive.sSpeed.fltSpeedFilt =
            GDFLIB_FilterIIR1_FLT(g_sM3Drive.sSpeed.fltSpeed, &g_sM3Drive.sSpeed.sSpeedFilter);

        /* pass required speed values lower than nominal speed */
        if ((MLIB_Abs_FLT(g_sM3Drive.sSpeed.fltSpeedCmd) > g_sM3Drive.sFaultThresholds.fltSpeedNom))
        {
            /* set required speed to nominal speed if over speed command > speed nominal */
            if (g_sM3Drive.sSpeed.fltSpeedCmd > 0.0F)
            {
            	g_sM3Drive.sSpeed.fltSpeedCmd = g_sM3Drive.sFaultThresholds.fltSpeedNom;
            }
            else
            {
            	g_sM3Drive.sSpeed.fltSpeedCmd = MLIB_Neg_FLT(g_sM3Drive.sFaultThresholds.fltSpeedNom);
            }
        }

        if ((MLIB_Abs_FLT(g_sM3Drive.sSpeed.fltSpeedRamp) < g_sM3Drive.sFaultThresholds.fltSpeedMin) &&
            (g_sM3Drive.sMCATctrl.ui16PospeSensor == MCAT_SENSORLESS_CTRL))
        {
        	M3_TransRunSpinFreewheel();
        }

        /* Actual position */
        //g_sM3Drive.sPosition.a32Position = g_sM3Enc.a32PosMeReal;

        /* Call PMSM position control */
        MCS_PMSMFocCtrlPosition(&g_sM3Drive.sPosition);

        /* Speed command is equal to position controller output */
        g_sM3Drive.sSpeed.fltSpeedCmd = MLIB_ConvSc_FLTsf(g_sM3Drive.sPosition.f16SpeedReq, M3_SPEED_CONV_SCALE);

        /* Call PMSM speed control */
        g_sM3Drive.sSpeed.bIqPiLimFlag = g_sM3Drive.sFocPMSM.sIqPiParams.bLimFlag;
        MCS_PMSMFocCtrlSpeed(&g_sM3Drive.sSpeed);
        g_sM3Drive.sFocPMSM.sIDQReq.fltQ = g_sM3Drive.sSpeed.fltIqReq;
    }
}

/*!
 * @brief Free-wheel process called in slow state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */

static void M3_StateRunFreewheelSlow(void)
{
    /* Wait until free-wheel time passes */
    if (--g_sM3Drive.ui16CounterState == 0U)
    {
        /* Switch to sub state READY */
        M3_TransRunFreewheelReady();
    }
}

/*!
 * @brief Transition from Calib to Ready state
 *
 * @param void  No input parameter
 *
 * @return None
 */

static void M3_TransRunCalibReady(void)
{
    /* Type the code to do when going from the RUN CALIB to the RUN READY sub-state */

    /* Set 50% PWM duty cycle */
    g_sM3Drive.sFocPMSM.sDutyABC.f16A = FRAC16(0.5);
    g_sM3Drive.sFocPMSM.sDutyABC.f16B = FRAC16(0.5);
    g_sM3Drive.sFocPMSM.sDutyABC.f16C = FRAC16(0.5);

    /* Switch to sub state READY */
    g_eM3StateRun = kRunState_Ready;
}

/*!
 * @brief Transition from Ready to Align state
 *
 * @param void  No input parameter
 *
 * @return None
 */

static void M3_TransRunReadyAlign(void)
{
    /* Type the code to do when going from the RUN kRunState_Ready to the RUN kRunState_Align sub-state */
    /* Alignment duration set-up */
    g_sM3Drive.ui16CounterState = g_sM3Drive.sAlignment.ui16Time;
    /* Counter of half alignment duration */
    g_sM3Drive.sAlignment.ui16TimeHalf = (uint16_t)MLIB_ShR_F16((int16_t)g_sM3Drive.sAlignment.ui16Time, 1);

    /* set required alignment voltage to Ud */
    g_sM3Drive.sFocPMSM.sUDQReq.fltD = g_sM3Drive.sAlignment.fltUdReq;
    g_sM3Drive.sFocPMSM.sUDQReq.fltQ = 0.0F;

    /* enable passing required position to FOC */
    g_sM3Drive.sFocPMSM.bPosExtOn = TRUE;

    /* disable current FOC */
    g_sM3Drive.sFocPMSM.bCurrentLoopOn = FALSE;

    /* enable Open loop mode in main control structure */
    g_sM3Drive.sFocPMSM.bOpenLoop = TRUE;

    /* Enable PWM output */
    M3_MCDRV_PWM3PH_EN(&g_sM3Pwm3ph);

    /* Sub-state RUN ALIGN */
    g_eM3StateRun = kRunState_Align;
}

/*!
 * @brief Transition from Align to Startup state
 *
 * @param void  No input parameter
 *
 * @return None
 */

static void M3_TransRunAlignStartup(void)
{
    /* Type the code to do when going from the RUN kRunState_Align to the RUN kRunState_Startup sub-state */
    /* initialize encoder driver */
    //M3_MCDRV_QD_CLEAR(&g_sM3Enc);

    /* Clear application parameters */
    M3_ClearFOCVariables();

    /* Pass required speed to open loop start-up structure */
    if (g_sM3Drive.sSpeed.fltSpeedCmd > 0.0F)
    {
    	g_sM3Drive.sStartUp.fltSpeedReq = g_sM3Drive.sStartUp.fltSpeedCatchUp;
    }
    else
    {
    	g_sM3Drive.sStartUp.fltSpeedReq = MLIB_Neg_FLT(g_sM3Drive.sStartUp.fltSpeedCatchUp);
    }

    /* Enable Open loop mode in main control structure */
    g_sM3Drive.sStartUp.bOpenLoop = TRUE;
    g_sM3Drive.sFocPMSM.bOpenLoop = TRUE;

    /* Enable Open loop mode in FOC module */
    g_sM3Drive.sFocPMSM.bPosExtOn = TRUE;

    g_sM3Drive.sFocPMSM.ui16SectorSVM = M3_SVM_SECTOR_DEFAULT;
    GDFLIB_FilterIIR1Init_FLT(&g_sM3Drive.sSpeed.sSpeedFilter);

    /* Go to sub-state RUN STARTUP */
    g_eM3StateRun = kRunState_Startup;
}

/*!
 * @brief Transition from Align to Spin state
 *
 * @param void  No input parameter
 *
 * @return None
 */

static void M3_TransRunAlignSpin(void)
{
    /* Type the code to do when going from the RUN STARTUP to the RUN SPIN sub-state */
    /* initialize encoder driver */
    //M3_MCDRV_QD_CLEAR(&g_sM3Enc);

    g_sM3Drive.sFocPMSM.bPosExtOn = TRUE;  /* enable passing external electrical position from encoder to FOC */
    g_sM3Drive.sFocPMSM.bOpenLoop = FALSE; /* disable parallel runnig openloop and estimator */

    g_sM3Drive.sFocPMSM.ui16SectorSVM = M3_SVM_SECTOR_DEFAULT;
    g_sM3Drive.sFocPMSM.sIDQReq.fltD  = 0.0F;
    g_sM3Drive.sFocPMSM.sIDQReq.fltQ  = 0.0F;

    M3_ClearFOCVariables();

    /* To switch to the RUN SPIN sub-state */
    g_eM3StateRun = kRunState_Spin;
}

/*!
 * @brief Transition from Align to Ready state
 *
 * @param void  No input parameter
 *
 * @return None
 */

static void M3_TransRunAlignReady(void)
{
    /* Type the code to do when going from the RUN kRunState_Align to the RUN kRunState_Ready sub-state */

    /* Clear FOC accumulators */
    M3_ClearFOCVariables();

    /* Go to sub-state RUN READY */
    g_eM3StateRun = kRunState_Ready;
}

/*!
 * @brief Transition from Startup to Spin state
 *
 * @param void  No input parameter
 *
 * @return None
 */

static void M3_TransRunStartupSpin(void)
{
    /* Type the code to do when going from the RUN kRunState_Startup to the RUN kRunState_Spin sub-state */
    /* For FOC control switch open loop off in DQ observer */
    if (g_sM3Drive.eControl != kControlMode_Scalar)
    {
        g_sM3Drive.sFocPMSM.bPosExtOn = FALSE; /* disable passing external electrical position to FOC */
        g_sM3Drive.sFocPMSM.bOpenLoop = FALSE; /* disable parallel running open-loop and estimator */
    }

    g_sM3Drive.sSpeed.sSpeedPiParams.fltIAccK_1 = g_sM3Drive.sFocPMSM.sIDQReq.fltQ;
    g_sM3Drive.sSpeed.sSpeedRampParams.fltState = g_sM3Drive.sStartUp.fltSpeedRampOpenLoop;

    /* To switch to the RUN kRunState_Spin sub-state */
    g_eM3StateRun = kRunState_Spin;
}

/*!
 * @brief Transition from Startup to Free-wheel state
 *
 * @param void  No input parameter
 *
 * @return None
 */

static void M3_TransRunStartupFreewheel(void)
{
    M3_MCDRV_PWM3PH_DIS(&g_sM3Pwm3ph);

    /* Free-wheel duration set-up */
    g_sM3Drive.ui16CounterState = g_sM3Drive.ui16TimeFullSpeedFreeWheel;

    /* Enter FREEWHEEL sub-state */
    g_eM3StateRun = kRunState_Freewheel;
}

/*!
 * @brief Transition from Spin to Free-wheel state
 *
 * @param void  No input parameter
 *
 * @return None
 */

static void M3_TransRunSpinFreewheel(void)
{
    /* Type the code to do when going from the RUN SPIN to the RUN FREEWHEEL sub-state */
    /* Set 50% PWM duty cycle */
    g_sM3Drive.sFocPMSM.sDutyABC.f16A = FRAC16(0.5);
    g_sM3Drive.sFocPMSM.sDutyABC.f16B = FRAC16(0.5);
    g_sM3Drive.sFocPMSM.sDutyABC.f16C = FRAC16(0.5);

    g_sM3Drive.sFocPMSM.ui16SectorSVM = M3_SVM_SECTOR_DEFAULT;

    M3_MCDRV_PWM3PH_DIS(&g_sM3Pwm3ph);

    /* Generates a time gap before the alignment to assure the rotor is not rotating */
    g_sM3Drive.ui16CounterState = g_sM3Drive.ui16TimeFullSpeedFreeWheel;

    g_sM3Drive.sFocPMSM.sIDQReq.fltD = 0.0F;
    g_sM3Drive.sFocPMSM.sIDQReq.fltQ = 0.0F;

    g_sM3Drive.sFocPMSM.sUDQReq.fltD = 0.0F;
    g_sM3Drive.sFocPMSM.sUDQReq.fltQ = 0.0F;

    g_sM3Drive.sFocPMSM.sIAlBe.fltAlpha    = 0.0F;
    g_sM3Drive.sFocPMSM.sIAlBe.fltBeta     = 0.0F;
    g_sM3Drive.sFocPMSM.sUAlBeReq.fltAlpha = 0.0F;
    g_sM3Drive.sFocPMSM.sUAlBeReq.fltBeta  = 0.0F;

    /* Enter FREEWHEEL sub-state */
    g_eM3StateRun = kRunState_Freewheel;
}

/*!
 * @brief Transition from Free-wheel to Ready state
 *
 * @param void  No input parameter
 *
 * @return None
 */

static void M3_TransRunFreewheelReady(void)
{
    /* Type the code to do when going from the RUN kRunState_FreeWheel to the RUN kRunState_Ready sub-state */
    /* Clear application parameters */
    M3_ClearFOCVariables();

    M3_MCDRV_PWM3PH_EN(&g_sM3Pwm3ph);

    /* Sub-state RUN READY */
    g_eM3StateRun = kRunState_Ready;
}

/*!
 * @brief Clear FOc variables in global variable
 *
 * @param void  No input parameter
 *
 * @return None
 */

static void M3_ClearFOCVariables(void)
{
    g_sM3Drive.sAlignment.ui16TimeHalf = 0U;

    /* Clear FOC variables */
    g_sM3Drive.sFocPMSM.sIABC.fltA             = 0.0F;
    g_sM3Drive.sFocPMSM.sIABC.fltB             = 0.0F;
    g_sM3Drive.sFocPMSM.sIABC.fltC             = 0.0F;
    g_sM3Drive.sFocPMSM.sIAlBe.fltAlpha        = 0.0F;
    g_sM3Drive.sFocPMSM.sIAlBe.fltBeta         = 0.0F;
    g_sM3Drive.sFocPMSM.sIDQ.fltD              = 0.0F;
    g_sM3Drive.sFocPMSM.sIDQ.fltQ              = 0.0F;
    g_sM3Drive.sFocPMSM.sIDQReq.fltD           = 0.0F;
    g_sM3Drive.sFocPMSM.sIDQReq.fltQ           = 0.0F;
    g_sM3Drive.sFocPMSM.sIDQError.fltD         = 0.0F;
    g_sM3Drive.sFocPMSM.sIDQError.fltQ         = 0.0F;
    g_sM3Drive.sFocPMSM.sDutyABC.f16A          = FRAC16(0.5);
    g_sM3Drive.sFocPMSM.sDutyABC.f16B          = FRAC16(0.5);
    g_sM3Drive.sFocPMSM.sDutyABC.f16C          = FRAC16(0.5);
    g_sM3Drive.sFocPMSM.sUAlBeReq.fltAlpha     = 0.0F;
    g_sM3Drive.sFocPMSM.sUAlBeReq.fltBeta      = 0.0F;
    g_sM3Drive.sFocPMSM.sUDQReq.fltD           = 0.0F;
    g_sM3Drive.sFocPMSM.sUDQReq.fltQ           = 0.0F;
    g_sM3Drive.sFocPMSM.sAnglePosEl.fltSin     = 0.0F;
    g_sM3Drive.sFocPMSM.sAnglePosEl.fltCos     = 0.0F;
    g_sM3Drive.sFocPMSM.sAnglePosEl.fltSin     = 0.0F;
    g_sM3Drive.sFocPMSM.sAnglePosEl.fltCos     = 0.0F;
    g_sM3Drive.sFocPMSM.sIdPiParams.bLimFlag   = FALSE;
    g_sM3Drive.sFocPMSM.sIqPiParams.bLimFlag   = FALSE;
    g_sM3Drive.sFocPMSM.sIdPiParams.fltIAccK_1 = 0.0F;
    g_sM3Drive.sFocPMSM.sIdPiParams.fltIAccK_1 = 0.0F;
    g_sM3Drive.sFocPMSM.sIqPiParams.fltIAccK_1 = 0.0F;
    g_sM3Drive.sFocPMSM.sIqPiParams.fltIAccK_1 = 0.0F;
    GDFLIB_FilterIIR1Init_FLT(&g_sM3Drive.sFocPMSM.sSpeedElEstFilt);
    g_sM3Drive.sFocPMSM.bIdPiStopInteg = FALSE;
    g_sM3Drive.sFocPMSM.bIqPiStopInteg = FALSE;

    /* Clear Speed control state variables */
    g_sM3Drive.sSpeed.sSpeedRampParams.fltState  = 0.0F;
    g_sM3Drive.sSpeed.fltSpeed                   = 0.0F;
    g_sM3Drive.sSpeed.fltSpeedFilt               = 0.0F;
    g_sM3Drive.sSpeed.fltSpeedError              = 0.0F;
    g_sM3Drive.sSpeed.fltSpeedRamp               = 0.0F;
    g_sM3Drive.sSpeed.sSpeedPiParams.fltIAccK_1  = 0.0F;
    g_sM3Drive.sSpeed.sSpeedPiParams.bLimFlag    = FALSE;
    g_sM3Drive.sSpeed.sSpeedFilter.fltFltBfrX[0] = 0.0F;
    g_sM3Drive.sSpeed.sSpeedFilter.fltFltBfrY[0] = 0.0F;
    g_sM3Drive.sSpeed.bSpeedPiStopInteg          = FALSE;
    GDFLIB_FilterIIR1Init_FLT(&g_sM3Drive.sSpeed.sSpeedFilter);

    /* Init Blocked rotor filter */
    GDFLIB_FilterMAInit_FLT(0.0F, &g_sM3Drive.msBlockedRotorUqFilt);

    /* Clear Scalar control variables */
    g_sM3Drive.sScalarCtrl.fltFreqRamp                 = 0.0F;
    g_sM3Drive.sScalarCtrl.f16PosElScalar              = FRAC16(0.0);
    g_sM3Drive.sScalarCtrl.sUDQReq.fltD                = 0.0F;
    g_sM3Drive.sScalarCtrl.sUDQReq.fltQ                = 0.0F;
    g_sM3Drive.sScalarCtrl.sFreqIntegrator.f32IAccK_1  = 0;
    g_sM3Drive.sScalarCtrl.sFreqIntegrator.f16InValK_1 = 0;
    g_sM3Drive.sScalarCtrl.sFreqRampParams.fltState    = 0.0F;

    /* Clear Startup variables */
    g_sM3Drive.sStartUp.f16PosMerged                      = 0;
    g_sM3Drive.sStartUp.f16PosEst                         = 0;
    g_sM3Drive.sStartUp.f16PosGen                         = 0;
    g_sM3Drive.sStartUp.f16RatioMerging                   = 0;
    g_sM3Drive.sStartUp.fltSpeedRampOpenLoop              = 0.0F;
    g_sM3Drive.sStartUp.fltSpeedReq                       = 0.0F;
    g_sM3Drive.sStartUp.sSpeedIntegrator.f32IAccK_1       = 0;
    g_sM3Drive.sStartUp.sSpeedIntegrator.f16InValK_1      = 0;
    g_sM3Drive.sStartUp.sSpeedRampOpenLoopParams.fltState = 0.0F;

    /* Clear BEMF and Tracking observers state variables */
    AMCLIB_PMSMBemfObsrvDQInit_A32fff(&g_sM3Drive.sFocPMSM.sBemfObsrv);
    AMCLIB_TrackObsrvInit_A32af(ACC32(0.0), &g_sM3Drive.sFocPMSM.sTo);
}

/*!
 * @brief Fault detention routine - check various faults
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_LIB
static void M3_FaultDetection(void)
{

    /* Clearing actual faults before detecting them again  */
    /* Clear all pending faults */
    FAULT_CLEAR_ALL(g_sM3Drive.sFaultIdPending);

    /* Clear fault captured manually if required. */
    if (g_sM3Drive.bFaultClearMan)
    {
        if(g_sM3Drive.sFaultIdPending & FAULT_I_DCBUS_OVER)
        {
           SMARTDMA_Reset();
           BOARD_SmartDMA0Pins();
        }   
        /* Clear fault captured */
        FAULT_CLEAR_ALL(g_sM3Drive.sFaultIdCaptured);
        g_sM3Drive.bFaultClearMan = FALSE;
    }

    /* Fault:   DC-bus over-current */
   // if (M3_MCDRV_PWM3PH_FLT_GET(&g_sM3Pwm3ph))
    //{
   // 	FAULT_SET(g_sM3Drive.sFaultIdPending, FAULT_I_DCBUS_OVER);
   // }

    /* Fault:   DC-bus over-voltage */
    if (g_sM3Drive.sFocPMSM.fltUDcBusFilt > g_sM3Drive.sFaultThresholds.fltUDcBusOver)
    {
    	FAULT_SET(g_sM3Drive.sFaultIdPending, FAULT_U_DCBUS_OVER);
    }

    /* Fault:   DC-bus under-voltage */
    if (g_sM3Drive.sFocPMSM.fltUDcBusFilt < g_sM3Drive.sFaultThresholds.fltUDcBusUnder)
    {
    	FAULT_SET(g_sM3Drive.sFaultIdPending, FAULT_U_DCBUS_UNDER);
    }
#if 0
    /* Check only in SPEED_FOC control, RUN state, kRunState_Spin and kRunState_FreeWheel sub-states */
    if ((g_sM3Drive.eControl == kControlMode_SpeedFOC) && (g_sM3Ctrl.eState == kSM_AppRun) &&
        (g_eM3StateRun == kRunState_Spin || g_eM3StateRun == kRunState_Freewheel))
    {
        /* Fault: Overload  */
        float_t fltSpeedFiltAbs = MLIB_Abs_FLT(g_sM3Drive.sSpeed.fltSpeedFilt);
        float_t fltSpeedRampAbs = MLIB_Abs_FLT(g_sM3Drive.sSpeed.fltSpeedRamp);

        if ((fltSpeedFiltAbs < g_sM3Drive.sFaultThresholds.fltSpeedMin) &&
            (fltSpeedRampAbs > g_sM3Drive.sFaultThresholds.fltSpeedMin) &&
            (g_sM3Drive.sSpeed.bSpeedPiStopInteg == TRUE))
        {
        	FAULT_SET(g_sM3Drive.sFaultIdPending, FAULT_LOAD_OVER);
        }

        /* Fault: Over-speed  */
        if ((MLIB_Abs_FLT(g_sM3Drive.sSpeed.fltSpeedFilt) > g_sM3Drive.sFaultThresholds.fltSpeedOver) &&
            (MLIB_Abs_FLT(g_sM3Drive.sSpeed.fltSpeedCmd) > g_sM3Drive.sFaultThresholds.fltSpeedMin))
        {
        	FAULT_SET(g_sM3Drive.sFaultIdPending, FAULT_SPEED_OVER);
        }

        /* Fault: Blocked rotor detection */
        if(g_sM3Drive.sMCATctrl.ui16PospeSensor == 0U)
        {
            /* Filter of bemf Uq voltage */
            g_sM3Drive.fltBemfUqAvg =
                MLIB_Abs_FLT(GDFLIB_FilterMA_FLT(g_sM3Drive.sFocPMSM.sBemfObsrv.sEObsrv.fltQ, &g_sM3Drive.msM3BlockedRotorUqFilt));

            /* Check the bemf Uq voltage threshold only in kRunState_Spin - RUN state */
            if ((g_sM3Drive.fltBemfUqAvg < g_sM3Drive.sFaultThresholds.fltUqBemf) && (g_eM3StateRun == kRunState_Spin))
            {
                    g_sM3Drive.ui16BlockRotorCnt++;
            }
            else
            {
                    g_sM3Drive.ui16BlockRotorCnt = 0U;
            }
        }

        /* For bemf voltage detected above limit longer than defined period number set blocked rotor fault*/
        if (g_sM3Drive.ui16BlockRotorCnt > g_sM3Drive.sFaultThresholds.ui16BlockedPerNum)
        {
            FAULT_SET(g_sM3Drive.sFaultIdPending, FAULT_ROTOR_BLOCKED);
            g_sM3Drive.ui16BlockRotorCnt = 0U;
        }
    }
#endif
    /* Mask pending faults by enable structure */
    g_sM3Drive.sFaultIdPending &= g_sM3Drive.sFaultIdEnable;

    /* Pass fault to Fault ID Captured */
    g_sM3Drive.sFaultIdCaptured |= g_sM3Drive.sFaultIdPending;
}

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @brief Set application switch value to On or Off mode
 *
 * @param bValue  bool value, true - On of false - Off
 *
 * @return None
 */

void M3_SetAppSwitch(bool_t bValue)
{
    g_bM3SwitchAppOnOff = bValue;
}

/*!
 * @brief Get application switch value
 *
 * @param void  No input parameter
 *
 * @return bool_t Return bool value, true or false
 */

bool_t M3_GetAppSwitch(void)
{
    return (g_bM3SwitchAppOnOff);
}

/*!
 * @brief Get application state
 *
 * @param void  No input parameter
 *
 * @return uint16_t Return current application state
 */

uint16_t M3_GetAppState(void)
{
    return ((uint16_t)g_sM3Ctrl.eState);
}

/*!
 * @brief Set spin speed of the motor in float value
 *
 * @param fltSpeedCmd  Speed command - set speed
 *
 * @return None
 */

void M3_SetSpeed(float_t fltSpeedCmd)
{
    if (g_bM3SwitchAppOnOff)
    {
        /* Set speed */
        if (MLIB_Abs_FLT(fltSpeedCmd / g_fltM3speedAngularScale) < g_sM3Drive.sStartUp.fltSpeedCatchUp)
        {
            g_sM3Drive.sSpeed.fltSpeedCmd = 0.0F;
        }
        else if (MLIB_Abs_FLT(fltSpeedCmd / g_fltM3speedAngularScale) > M2_N_NOM)
        {
            g_sM3Drive.sSpeed.fltSpeedCmd = 0.0F;
        }
        else
        {
            g_sM3Drive.sSpeed.fltSpeedCmd = fltSpeedCmd / g_fltM3speedAngularScale;
        }
    }
    else
    {
        /* Set zero speed */
        g_sM3Drive.sSpeed.fltSpeedCmd = 0.0F;
    }
}

/*!
 * @brief Set position of the motor
 *
 * @param a32PositionCmdDemo  Position command - set position
 *
 * @return None
 */

void M3_SetPosition(acc32_t a32PositionCmdDemo)
{
    if (g_bM3SwitchAppOnOff)
    {
        /* Set position */
        g_sM3Drive.sPosition.a32PositionCmd = a32PositionCmdDemo;
    }
    else
    {
        /* Set zero position */
        g_sM3Drive.sPosition.a32PositionCmd = ACC32(0.0);
    }
}

/*!
 * @brief Get spin speed of the motor in float value
 *
 * @param void  No input parameter
 *
 * @return float_t Float value of the current speed
 */

float_t M3_GetSpeed(void)
{
    /* Return speed */
    return g_sM3Drive.sSpeed.fltSpeedCmd;
}
