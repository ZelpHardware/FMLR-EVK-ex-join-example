// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
// Copyright (C) 2014-2016 IBM Corporation. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

#ifndef _debug_h_
#define _debug_h_

#include "clkman.h"
#include "max32625.h"
#include "mxc_config.h"
#include "mxc_assert.h"
#include "uart_regs.h"
#include "pwrman_regs.h"
#include "ioman.h"

/**
 * Enumeration type for defining the number of bits per character.
 */
typedef enum {
    UART_DATA_SIZE_5_BITS = MXC_V_UART_CTRL_DATA_SIZE_5_BITS, /**< 5 Data bits per UART character. */
    UART_DATA_SIZE_6_BITS = MXC_V_UART_CTRL_DATA_SIZE_6_BITS, /**< 6 Data bits per UART character. */
    UART_DATA_SIZE_7_BITS = MXC_V_UART_CTRL_DATA_SIZE_7_BITS, /**< 7 Data bits per UART character. */
    UART_DATA_SIZE_8_BITS = MXC_V_UART_CTRL_DATA_SIZE_8_BITS  /**< 8 Data bits per UART character. */
}
uart_data_size_t;

/**
 * Enumeration type for selecting Parity and type.
 */
typedef enum {
    UART_PARITY_DISABLE = MXC_V_UART_CTRL_PARITY_DISABLE,    /**< Parity disabled.  */
    UART_PARITY_ODD     = MXC_V_UART_CTRL_PARITY_ODD,        /**< Odd parity.       */
    UART_PARITY_EVEN    = MXC_V_UART_CTRL_PARITY_EVEN,       /**< Even parity.      */
    UART_PARITY_MARK    = MXC_V_UART_CTRL_PARITY_MARK        /**< Mark parity.      */
} uart_parity_t;

/**
 * Configuration structure type for a UART port.
 */
typedef struct {
    uint8_t extra_stop;             /**< @c 0 for one stop bit, @c 1 for two stop bits*/
    uint8_t cts;                    /**< CTS Enable/Disable, @c 1 to enable CTS, @c 0 to disable CTS.*/
    uint8_t rts;                    /**< @c 1 to enable RTS, @c 0 to disable RTS. */
    uint32_t baud;                  /**< Baud rate in Hz.                                               */
    uart_data_size_t size;          /**< Set the number of bits per character, see uart_data_size_t.   */
    uart_parity_t parity;           /**< Set the parity, see uart_parity_t for supported parity types. */
} uart_cfg_t;


/** @brief System Configuration Object */
typedef struct {
    clkman_scale_t clk_scale;   /** desired clock scale value for the peripheral */
    ioman_cfg_t io_cfg;         /** IOMAN configuration object */
} sys_cfg_t;

/** @brief UART System Configuration Object */
typedef sys_cfg_t sys_cfg_uart_t;


#ifndef CFG_DEBUG

#define debug_snprintf(s,n,f,...)	do { } while (0)
#define debug_printf(f,...)		do { } while (0)
#define debug_str(s)			do { } while (0)
#define debug_led(val)			do { } while (0)

#else

// write formatted string to buffer
int debug_snprintf (char *str, int size, const char *format, ...);

// write formatted string to USART
void debug_printf (char const *format, ...);

// debug init
void debug_init (void);

// write nul-terminated string to USART
void debug_str (const char* str);

// set LED state
void debug_led (int val);



/**
 * @brief   Initialize and enable UART module.
 * @param   uart        Pointer to the UART registers.
 * @param   cfg         Pointer to UART configuration.
 * @param   sys_cfg     Pointer to system configuration object
 * @returns #E_NO_ERROR UART initialized successfully, @ref MXC_Error_Codes "error" if
 *             unsuccessful.
 */
int UART_Init(mxc_uart_regs_t *uart, const uart_cfg_t *cfg, const sys_cfg_uart_t *sys_cfg);

/**
 * @brief      Write UART data. This function blocks until the write transaction
 *             is complete.
 * @param      uart  Pointer to the UART registers.
 * @param      data  Pointer to buffer for write data.
 * @param      len   Number of bytes to write.
 * @note       Will return once data has been put into FIFO, not necessarily
 *             transmitted.
 * @return     Number of bytes written if successful, error if unsuccessful.
 */
int UART_Write(mxc_uart_regs_t *uart, uint8_t* data, int len);

#endif

#endif
