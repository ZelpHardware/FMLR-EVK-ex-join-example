/*******************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 * $Date: 2016-07-28 13:49:49 -0500 (Thu, 28 Jul 2016) $
 * $Revision: 23808 $
 *
 ******************************************************************************/

/**
 * @file    board.h
 * @brief   Board support package API.
 */

#ifndef _BOARD_H
#define _BOARD_H

#include "gpio.h"
#include "spim.h"
#include "ioman.h"
#include "led.h"
#include "pb.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CONSOLE_UART
#define CONSOLE_UART        0   /// UART instance to use for console
#endif

#ifndef CONSOLE_BAUD
#define CONSOLE_BAUD    115200  /// Console baud rate
#endif

// Pushbutton Indices
#define SW1             0       /// Pushbutton index for SW1
#define SW2             1       /// Pushbutton index for SW2
#define SW3             2       /// Pushbutton index for SW3

#define LED_OFF         1       /// Inactive state of LEDs
#define LED_ON          0       /// Active state of LEDs

// Console UART configuration
extern const uart_cfg_t console_uart_cfg;
extern const sys_cfg_uart_t console_sys_cfg;
extern const gpio_cfg_t console_uart_rx;
extern const gpio_cfg_t console_uart_tx;
extern const gpio_cfg_t console_uart_cts;
extern const gpio_cfg_t console_uart_rts;

// EM9301 BTLE Radio
extern const gpio_cfg_t hci_rst;
extern const gpio_cfg_t hci_irq;

// MAX14690 PMIC
#define MAX14690_I2CM_INST  0
#define MAX14690_I2CM       MXC_I2CM0
extern const sys_cfg_i2cm_t max14690_sys_cfg;
extern const gpio_cfg_t max14690_int;
extern const gpio_cfg_t max14690_mpc0;
extern const gpio_cfg_t max14690_pfn2;

// NHD12832
#define NHD12832_SPI        MXC_SPIM0
#define NHD12832_SSEL       0
extern const sys_cfg_spim_t nhd12832_sys_cfg;
extern const gpio_cfg_t nhd12832_spi;
extern const gpio_cfg_t nhd12832_res;
extern const gpio_cfg_t nhd12832_dc;
extern const spim_cfg_t nhd12832_spim_cfg;

// PMOD
#define PMOD_SPI    MXC_SPIM0
#define PMOD_SSEL   3
#define PMOD_I2C    MXC_I2CM0

/**
 * @brief   Initialize the BSP and board interfaces.
 * @retval  E_NO_ERROR if everything is successful
 */
int Board_Init(void);

/**
 * @brief   Initialize or reinitialize the console. This may be necessary if the
 *          system clock rate is changed.
 * @retval  E_NO_ERROR if everything is successful
 */
int Console_Init(void);

/**
 * @brief   Attempt to prepare the console for sleep.
 * @retval  E_NO_ERROR if ready to sleep, #E_BUSY if not ready for sleep.
 */
int Console_PrepForSleep(void);

/**
 * @brief   Board level initialization of nhd12832 display
 *
 */
void Board_nhd12832_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* _BOARD_H */
