/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _BOARD_H_
#define _BOARD_H_

#include "clock_config.h"
#include "fsl_gpio.h"
#include "fsl_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief The UART to use for debug messages. */
#define BOARD_DEBUG_UART_TYPE     kSerialPort_Uart
#define BOARD_DEBUG_UART_CLK_FREQ 12000000U

#ifndef BOARD_DEBUG_UART_BAUDRATE
#define BOARD_DEBUG_UART_BAUDRATE 115200U
#endif

#define BOARD_DEBUG_UART_BASEADDR   (uint32_t) LPUART3
#define BOARD_DEBUG_UART_INSTANCE   3U
#define BOARD_DEBUG_UART_CLK_ATTACH kFRO_LF_DIV_to_LPUART3
#define BOARD_DEBUG_UART_RST        kLPUART3_RST_SHIFT_RSTn
#define BOARD_DEBUG_UART_CLKSRC     kCLOCK_LPUART3
#define BOARD_UART_IRQ_HANDLER      LPUART3_IRQHandler
#define BOARD_UART_IRQ              LPUART3_IRQn

/*******************************************************************************
 * API
 ******************************************************************************/
void BOARD_InitDebugConsole(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _BOARD_H_ */
