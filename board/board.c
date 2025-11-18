/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include "fsl_common.h"
#include "fsl_debug_console.h"
#include "board.h"
#if defined(SDK_I2C_BASED_COMPONENT_USED) && SDK_I2C_BASED_COMPONENT_USED
#include "fsl_lpi2c.h"
#endif /* SDK_I2C_BASED_COMPONENT_USED */

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/* Initialize debug console. */
void BOARD_InitDebugConsole(void)
{
    /* attach 12 MHz clock to FLEXCOMM0 (debug console) */
    CLOCK_SetClockDiv(kCLOCK_DivLPUART3, 1u);
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    RESET_PeripheralReset(BOARD_DEBUG_UART_RST);

    DbgConsole_Init(BOARD_DEBUG_UART_INSTANCE, BOARD_DEBUG_UART_BAUDRATE, BOARD_DEBUG_UART_TYPE,
                    BOARD_DEBUG_UART_CLK_FREQ);
}