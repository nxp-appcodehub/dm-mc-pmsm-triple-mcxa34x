/*
    * Copyright 2024 NXP 
    *
    * SPDX-License-Identifier: BSD-3-Clause 
*/

/*
    * FILE NAME: ../../source/m1_pmsm_appconfig.h
    * DATE: Thu May 30 2024, 16:10:51
*/

/*
{
    "parameters": {
        "parametersPP": 4,
        "parametersRs": 5,
        "parametersLd": 0.03,
        "parametersLq": 0.06,
        "parametersKe": 0.478,
        "parametersJ": 0.000016,
        "parametersIphNom": 2.5,
        "parametersUphNom": 15,
        "parametersNnom": 1500,
        "parametersImax": 5,
        "parametersUdcbMax": 443.3,
        "parametersUdcbTrip": 400,
        "parametersUdcbUnder": 180,
        "parametersUdcbOver": 420,
        "parametersNover": 1600,
        "parametersNmin": 50,
        "parametersEblock": 30,
        "parametersEblockPer": 2000,
        "parametersNmax": 1600,
        "parametersUdcbIIRf0": 100,
        "parametersCalibDuration": 0.2,
        "parametersFaultDuration": 3,
        "parametersFreewheelDuration": 1.5,
        "parametersScalarUqMin": 8,
        "parametersAlignVoltage": 20,
        "parametersAlignDuration": 4
    },
    "currentLoop": {
        "currentLoopSampleTime": 0.000125,
        "currentLoopF0": 600,
        "currentLoopKsi": 1,
        "currentLoopOutputLimit": 70
    },
    "speedLoop": {
        "speedLoopSampleTime": 0.001,
        "speedLoopF0": 90,
        "speedLoopKsi": 1,
        "speedLoopIncUp": 50,
        "speedLoopIncDown": 50,
        "speedLoopCutOffFreq": 100,
        "speedLoopUpperLimit": 3,
        "speedLoopLowerLimit": -2,
        "speedLoopSLKp": 0.005,
        "speedLoopSLKi": 0.00001,
        "speedLoopManualConstantTunning": false
    },
    "sensorless": {
        "sensorlessBemfObsrvF0": 360,
        "sensorlessBemfObsrvKsi": 1,
        "sensorlessTrackObsrvF0": 30,
        "sensorlessTrackObsrvKsi": 1,
        "sensorlessTrackObsrvIIRSpeedCutOff": 100,
        "sensorlessStartupRamp": 500,
        "sensorlessStartupCurrent": 0.5,
        "sensorlessMergingSpeed": 500,
        "sensorlessMergingCoeff": 150
    }
}
*/

/*
{
    "motorName": "M1p1",
    "motorDescription": ""
}
*/

#ifndef __M1_PMSM_APPCONFIG_H 
#define __M1_PMSM_APPCONFIG_H 

/* PARAMETERS*/
#define M1_MOTOR_PP (4)
#define M1_I_PH_NOM (2.5F)
#define M1_N_NOM (628.319F)
#define M1_I_MAX (5.0F)
#define M1_U_DCB_MAX (443.3F)
#define M1_U_DCB_TRIP (400.0F)
#define M1_U_DCB_UNDERVOLTAGE (180.0F)
#define M1_U_DCB_OVERVOLTAGE (420.0F)
#define M1_N_OVERSPEED (670.206F)
#define M1_N_MIN (20.9440F)
#define M1_E_BLOCK_TRH (30.0F)
#define M1_E_BLOCK_PER (2000)
#define M1_N_MAX (670.206F)
#define M1_CALIB_DURATION (200)
#define M1_FAULT_DURATION (3000)
#define M1_FREEWHEEL_DURATION (1500)
#define M1_SCALAR_UQ_MIN (8.0F)
#define M1_ALIGN_VOLTAGE (20.0F)
#define M1_ALIGN_DURATION (32000)
#define M1_U_MAX (255.939F)
#define M1_FREQ_MAX (106.667F)
#define M1_N_ANGULAR_MAX (2.38732F)
#define M1_UDCB_IIR_B0 (0.0377861F)
#define M1_UDCB_IIR_B1 (0.0377861F)
#define M1_UDCB_IIR_A1 (0.924428F)
#define M1_SCALAR_VHZ_FACTOR_GAIN (0.15F)
#define M1_SCALAR_INTEG_GAIN ACC32(0.0266667)
#define M1_SCALAR_RAMP_UP (0.000416667F)
#define M1_SCALAR_RAMP_DOWN (0.000416667F)
/* CURRENTLOOP*/
#define M1_D_KP_GAIN (221.195F)
#define M1_D_KI_GAIN (53.2959F)
#define M1_Q_KP_GAIN (447.389F)
#define M1_Q_KI_GAIN (106.592F)
#define M1_CLOOP_LIMIT (0.404145F)
/* SPEEDLOOP*/
#define M1_SPEED_RAMP_UP (0.0209440F)
#define M1_SPEED_RAMP_DOWN (0.0209440F)
#define M1_SPEED_LOOP_HIGH_LIMIT (3.0F)
#define M1_SPEED_LOOP_LOW_LIMIT (-2.0F)
#define M1_SPEED_PI_PROP_GAIN (0.00228882F)
#define M1_SPEED_PI_INTEG_GAIN (0.0000647150F)
#define M1_SPEED_IIR_B0 (0.0377861F)
#define M1_SPEED_IIR_B1 (0.0377861F)
#define M1_SPEED_IIR_A1 (0.924428F)
/* SENSORLESS*/
#define M1_OL_START_RAMP_INC (0.0261799F)
#define M1_OL_START_I (0.5F)
#define M1_MERG_SPEED_TRH (209.440F)
#define M1_MERG_COEFF FRAC16(0.00625)
#define M1_I_SCALE (0.979592F)
#define M1_U_SCALE (0.00408163F)
#define M1_E_SCALE (0.00408163F)
#define M1_WI_SCALE (0.000244898F)
#define M1_BEMF_DQ_KP_GAIN (130.717F)
#define M1_BEMF_DQ_KI_GAIN (19.1865F)
#define M1_TO_KP_GAIN (376.991F)
#define M1_TO_KI_GAIN (4.44132F)
#define M1_TO_THETA_GAIN (0.0000397887F)
#define M1_TO_SPEED_IIR_B0 (0.0377861F)
#define M1_TO_SPEED_IIR_B1 (0.0377861F)
#define M1_TO_SPEED_IIR_A1 (0.924428F)
/* USER INPUT START */
/* USER INPUT END */
#endif /* __M1_PMSM_APPCONFIG_H */
