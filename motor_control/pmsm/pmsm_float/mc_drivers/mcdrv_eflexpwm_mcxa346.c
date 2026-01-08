/*
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "mcdrv_eflexpwm_mcxa346.h"
#include "mc_periph_init.h"
#include "fsl_ctimer.h"
#include "pin_mux.h"
#include "fsl_gpio.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

static bool_t s_statusPass;
/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Function updates FTM value register
 *
 * @param this   Pointer to the current object
 * TODO - Deadtime compensation?
 * @return none
 */
RAM_FUNC_LIB
void MCDRV_eFlexPwm3PhSet(mcdrv_eflexpwm_t *this)
{
    PWM_Type * const pCurrentPwm = this->pui32PwmBaseAddress;

    GMCLIB_3COOR_T_F16 sUABCtemp;

    int16_t w16ModuloHalf;
    int16_t w16PwmValPhA;
    int16_t w16PwmValPhB;
    int16_t w16PwmValPhC;

    /* Clear error flag if no overvoltage is present */

    /* pointer to duty cycle structure */
    sUABCtemp = *this->psUABC;

    /* Get PWM_modulo/2 from PWM register, always correct regardless of PWM runtime setting, variable used for update of duty cycles of all 3 phases */
    w16ModuloHalf = (pCurrentPwm->SM[0].VAL1+1);

    /* Phase A - duty cycle calculation */
    w16PwmValPhA = (w16ModuloHalf * sUABCtemp.f16A) >> 15;
    pCurrentPwm->SM[0].VAL2 = (uint16_t)(-w16PwmValPhA); // rising edge value register update
    pCurrentPwm->SM[0].VAL3 = (uint16_t)w16PwmValPhA;  // falling edge value register update, no need to calculate it

    /* Phase B - duty cycle calculation */
    w16PwmValPhB = (w16ModuloHalf * sUABCtemp.f16B) >> 15;
    pCurrentPwm->SM[1].VAL2 = (uint16_t)(-w16PwmValPhB); // rising edge value register update
    pCurrentPwm->SM[1].VAL3 = (uint16_t)w16PwmValPhB; // falling edge value register update, no need to calculate it

    /* Phase C - duty cycle calculation */
    w16PwmValPhC = (w16ModuloHalf * sUABCtemp.f16C) >> 15;
    pCurrentPwm->SM[2].VAL2 = (uint16_t)(-w16PwmValPhC); // rising edge value register update
    pCurrentPwm->SM[2].VAL3 = (uint16_t)w16PwmValPhC; // falling edge value register update, no need to calculate it

    pCurrentPwm->MCTRL |= PWM_MCTRL_LDOK(7);
}

RAM_FUNC_LIB
void MCDRV_eFlexPwm3PhSet_M3(mcdrv_eflexpwm_t *this)
{
	static int flag = 0;
    PWM_Type * const pCurrentPwm = this->pui32PwmBaseAddress;

    GMCLIB_3COOR_T_F16 sUABCtemp;

    int16_t w16ModuloHalf;
    int16_t w16PwmValPhA;
    int16_t w16PwmValPhB;
    int16_t w16PwmValPhC;
    int16_t w16DeadVal = (M3_PWM_DEADTIME*(M3_PWM_FREQ/1000)/1000)* CTIMER2->MR[kCTIMER_Match_2]/1000;
    /* Clear error flag if no overvoltage is present */

    /* pointer to duty cycle structure */
    sUABCtemp = *this->psUABC;
    /* Keep Sample Window */   
    switch(*g_sM3AdcSensor.pui16SVMSector)
    {
        case 2:
        case 3:
          /* direct sensing of phase A and C, calculation of B */
          if(sUABCtemp.f16B > FRAC16(0.98))
          {
            sUABCtemp.f16A = (sUABCtemp.f16A > FRAC16(0.02))? (MLIB_Sub_F16(sUABCtemp.f16A,FRAC16(0.02))) : 0;
            sUABCtemp.f16C = (sUABCtemp.f16C > FRAC16(0.02))? (MLIB_Sub_F16(sUABCtemp.f16C,FRAC16(0.02))) : 0;
            sUABCtemp.f16B = MLIB_Sub_F16(sUABCtemp.f16B,FRAC16(0.02));
          }
   
        break;

        case 4:
        case 5:
        break;

        case 1:
        case 6:
        default:
          /* direct sensing of phase B and C, calculation of A */
          if(sUABCtemp.f16A > FRAC16(0.98))
          {
            sUABCtemp.f16B = (sUABCtemp.f16B > FRAC16(0.02))? (MLIB_Sub_F16(sUABCtemp.f16B,FRAC16(0.02))) : 0;
            sUABCtemp.f16C = (sUABCtemp.f16C > FRAC16(0.02))? (MLIB_Sub_F16(sUABCtemp.f16C,FRAC16(0.02))) : 0;
            sUABCtemp.f16A = MLIB_Sub_F16(sUABCtemp.f16A,FRAC16(0.02));         
          }
        break;   
    
    }

    /* Get PWM_modulo/2 from PWM register, always correct regardless of PWM runtime setting, variable used for update of duty cycles of all 3 phases */
    w16ModuloHalf = (pCurrentPwm->SM[0].VAL1+1);

    /* Phase A - duty cycle calculation */
    w16PwmValPhA = (w16ModuloHalf * sUABCtemp.f16A) >> 15;
    pCurrentPwm->SM[3].VAL2 = (uint16_t)(-w16PwmValPhA); // rising edge value register update
    pCurrentPwm->SM[3].VAL3 = (uint16_t)w16PwmValPhA;  // falling edge value register update, no need to calculate it

    /* Phase B - duty cycle calculation */
    w16ModuloHalf = CTIMER0->MR[kCTIMER_Match_2];
    w16PwmValPhB = (w16ModuloHalf * sUABCtemp.f16B) >> 15;
    w16PwmValPhB = w16ModuloHalf-w16PwmValPhB;
    CTIMER0->MSR[kCTIMER_Match_0] = w16PwmValPhB/2;
    CTIMER0->MSR[kCTIMER_Match_1] = w16ModuloHalf-w16PwmValPhB/2+w16DeadVal;
    CTIMER1->MSR[kCTIMER_Match_0] = w16PwmValPhB/2+w16DeadVal;
    CTIMER1->MSR[kCTIMER_Match_1] = w16ModuloHalf-w16PwmValPhB/2;

    /* Phase C - duty cycle calculation */
    w16ModuloHalf = CTIMER2->MR[kCTIMER_Match_2];
    w16PwmValPhC = (w16ModuloHalf * sUABCtemp.f16C) >> 15;
    w16PwmValPhC = w16ModuloHalf-w16PwmValPhC;
    CTIMER2->MSR[kCTIMER_Match_0] = w16PwmValPhC/2;
    CTIMER2->MSR[kCTIMER_Match_1] = w16ModuloHalf-w16PwmValPhC/2+w16DeadVal;
    CTIMER3->MSR[kCTIMER_Match_0] = w16PwmValPhC/2+w16DeadVal;
    CTIMER3->MSR[kCTIMER_Match_1] = w16ModuloHalf-w16PwmValPhC/2;

    
    pCurrentPwm->MCTRL |= PWM_MCTRL_LDOK(8);
}

/*!
 * @brief Function enables PWM outputs
 *
 * @param this   Pointer to the current object
 *
 * @return none
 */
void MCDRV_eFlexPwm3PhOutEn(mcdrv_eflexpwm_t *this)
{
    PWM_Type * const pCurrentPwm = this->pui32PwmBaseAddress;

    pCurrentPwm->SM[0].VAL2 = 0U;
    pCurrentPwm->SM[1].VAL2 = 0U;
    pCurrentPwm->SM[2].VAL2 = 0U;

    pCurrentPwm->SM[0].VAL3 = 0U;
    pCurrentPwm->SM[1].VAL3 = 0U;
    pCurrentPwm->SM[2].VAL3 = 0U;

    /* Clear fault flag (we're in safe mode so the PWM won't run if there's an error condition */
    pCurrentPwm->FSTS |= PWM_FSTS_FFLAG(1U);

    /* Start PWMs (set load OK flags and run) */
    pCurrentPwm->MCTRL |= PWM_MCTRL_CLDOK(7);
    pCurrentPwm->MCTRL |= PWM_MCTRL_LDOK(7);
    pCurrentPwm->MCTRL |= PWM_MCTRL_RUN(7);

    /* Enable A&B (Top & Bottm) PWM outputs of submodules one, two and three */
    pCurrentPwm->OUTEN |= PWM_OUTEN_PWMA_EN(0x7);
    pCurrentPwm->OUTEN |= PWM_OUTEN_PWMB_EN(0x7);
}

void MCDRV_eFlexPwm3PhOutEn_M3(mcdrv_eflexpwm_t *this)
{
    PWM_Type * const pCurrentPwm = this->pui32PwmBaseAddress;

    pCurrentPwm->SM[3].VAL2 = 0U;

    pCurrentPwm->SM[3].VAL3 = 0U;

    /* Clear fault flag (we're in safe mode so the PWM won't run if there's an error condition */
    pCurrentPwm->FSTS |= PWM_FSTS_FFLAG(1U);

    /* Start PWMs (set load OK flags and run) */
    pCurrentPwm->MCTRL |= PWM_MCTRL_CLDOK(8);
    pCurrentPwm->MCTRL |= PWM_MCTRL_LDOK(8);
    pCurrentPwm->MCTRL |= PWM_MCTRL_RUN(8);

    /* Enable A&B (Top & Bottm) PWM outputs of submodules one, two and three */
    pCurrentPwm->OUTEN |= PWM_OUTEN_PWMA_EN(0x8);
    pCurrentPwm->OUTEN |= PWM_OUTEN_PWMB_EN(0x8);
    
    CTIMER0->TCR |= CTIMER_TCR_ATCEN(1);
    CTIMER1->TCR |= CTIMER_TCR_ATCEN(1);
    CTIMER2->TCR |= CTIMER_TCR_ATCEN(1);
    CTIMER3->TCR |= CTIMER_TCR_ATCEN(1);

}

/*!
 * @brief Function disables PWM outputs
 *
 * @param this   Pointer to the current object
 *
 * @return none
 */
void MCDRV_eFlexPwm3PhOutDis(mcdrv_eflexpwm_t *this)
{
    PWM_Type * const pCurrentPwm = this->pui32PwmBaseAddress;

    /* Disable A&B (Top & Bottm) PWM outputs of submodules one, two and three */
    pCurrentPwm->OUTEN &= (~PWM_OUTEN_PWMA_EN(0x7));
    pCurrentPwm->OUTEN &= (~PWM_OUTEN_PWMB_EN(0x7));
}

void MCDRV_eFlexPwm3PhOutDis_M3(mcdrv_eflexpwm_t *this)
{
    PWM_Type * const pCurrentPwm = this->pui32PwmBaseAddress;

    /* Disable A&B (Top & Bottm) PWM outputs of submodules one, two and three */
    pCurrentPwm->OUTEN &= (~PWM_OUTEN_PWMA_EN(0x8));
    pCurrentPwm->OUTEN &= (~PWM_OUTEN_PWMB_EN(0x8));
 
    CTIMER0->TCR &= ~(CTIMER_TCR_ATCEN_MASK|CTIMER_TCR_CEN_MASK);   
    CTIMER1->TCR &= ~(CTIMER_TCR_ATCEN_MASK|CTIMER_TCR_CEN_MASK);
    CTIMER2->TCR &= ~(CTIMER_TCR_ATCEN_MASK|CTIMER_TCR_CEN_MASK);    
    CTIMER3->TCR &= ~(CTIMER_TCR_ATCEN_MASK|CTIMER_TCR_CEN_MASK);
    
    GPIO_PinWrite(GPIO3, BOARD_SMARTDMA0PINS_MC3_CT_GPIO_PIN, 0);
    GPIO_PinWrite(GPIO3, BOARD_SMARTDMA0PINS_MC3_CB_GPIO_PIN, 0);
    GPIO_PinWrite(GPIO3, BOARD_SMARTDMA0PINS_MC3_BT_GPIO_PIN, 0);
    GPIO_PinWrite(GPIO3, BOARD_SMARTDMA0PINS_MC3_BB_GPIO_PIN, 0);    
}

void MCDRV_eFlexPwm3PhBottomOutEn(mcdrv_eflexpwm_t *this)
{
    PWM_Type * const pCurrentPwm = this->pui32PwmBaseAddress;
    
    pCurrentPwm->OUTEN &= (~PWM_OUTEN_PWMA_EN(0x7));
    pCurrentPwm->OUTEN |= PWM_OUTEN_PWMB_EN(0x7);
}

/*!
 * @brief Function initialite PWM outputs structure
 *
 * @param this   Pointer to the current object
 *
 * @return none
 */
void MCDRV_eFlexPwm3PhOutInit(mcdrv_eflexpwm_t *this)
{

}

/*!
 * @brief Function return actual value of over current flag
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */
bool_t MCDRV_eFlexPwm3PhFltGet(mcdrv_eflexpwm_t *this)
{
    /* read over-current flags */
    s_statusPass = (((this->pui32PwmBaseAddress->FSTS & PWM_FSTS_FFPIN_MASK) >> 8) &
                    (1 << this->ui16FaultFixNum | 1 << this->ui16FaultAdjNum));

    /* clear faults flag */
    this->pui32PwmBaseAddress->FSTS = ((this->pui32PwmBaseAddress->FSTS & ~(uint16_t)(PWM_FSTS_FFLAG_MASK)) |
                                       (1 << this->ui16FaultFixNum | 1 << this->ui16FaultAdjNum));

    return ((s_statusPass > 0));
}

void MCDRV_eFlexPwm3PhFltTryClr(mcdrv_eflexpwm_t *this)
{
    PWM_Type * const pCurrentPwm = this->pui32PwmBaseAddress;

    /* We can clear the FFLAGs only if the respective FFPIN (raw fault input) isn't set. */
    const uint8_t u8FfpinNoErrorMask =
                    (uint8_t)(~(((pCurrentPwm->FSTS) & PWM_FSTS_FFLAG_MASK) >> PWM_FSTS_FFLAG_SHIFT));

    pCurrentPwm->FSTS |= PWM_FSTS_FFLAG(u8FfpinNoErrorMask);
}
