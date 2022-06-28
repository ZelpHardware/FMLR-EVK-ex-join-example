// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
// Copyright (C) 2014-2016 IBM Corporation. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

#ifdef CFG_DEBUG

#include "debug.h"
#include <stdarg.h>
#include <string.h>

static int itoa (char* buf, unsigned int val, int base, int mindigits, int exp, int prec, char sign) {
    char num[33], *p = num, *b = buf;
    if (sign) {
	if ((int) val < 0) {
	    val = -val;
	    *b++ = '-';
	} else if (sign != '-') {
	    *b++ = sign; // space or plus
	}
    }
    if (mindigits > 32) {
	mindigits = 32;
    }
    do {
	int m = val % base;
        *p++ = (m <= 9) ? m + '0' : m - 10 + 'A';
	if (p - num == exp) *p++ = '.';
    } while ( (val /= base) || p - num < mindigits );
    do {
	*b++ = *--p;
    } while (p > num + exp - prec);
    *b = 0;
    return b - buf;
}

static int strpad (char *buf, int size, const char *str, int len, int width, int leftalign, char pad) {
    if (len > width) {
	width = len;
    }
    if (width > size) {
	width = size;
    }
    for (int i = 0, npad = width - len; i < width; i++) {
	buf[i] = (leftalign) ? ((i < len) ? str[i] : pad) : ((i < npad) ? pad : str[i - npad]);
    }
    return width;
}

static int debug_vsnprintf(char *str, int size, const char *format, va_list arg) {
    char c, *dst = str, *end = str + size - 1;
    int width, left, base, zero, space, plus, prec, sign;

    while ( (c = *format++) && dst < end ) {
	if (c != '%') {
	    *dst++ = c;
	} else {
	    // flags
	    width = prec = left = zero = sign = space = plus = 0;
	    while ( (c = *format++) ) {
		if (c == '-') left = 1;
		else if (c == ' ') space = 1;
		else if (c == '+') plus = 1;
		else if (c == '0') zero = 1;
		else break;
	    }
	    // width
	    if (c == '*') {
		width = va_arg(arg, int);
		c = *format++;
	    } else {
		while (c >= '0' && c <= '9') {
		    width = width * 10 + c - '0';
		    c = *format++;
		}
	    }
	    // precision
	    if (c == '.') {
		c = *format++;
		if (c == '*') {
		    prec = va_arg(arg, int);
		    c = *format++;
		} else {
		    while (c >= '0' && c <= '9') {
			prec = prec * 10 + c - '0';
			c = *format++;
		    }
		}
	    }
	    // conversion specifiers
	    switch (c) {
		case 'c': // character
		    c = va_arg(arg, int);
		    // fallthrough
		case '%': // percent literal
		    *dst++ = c;
		    break;
		case 's': { // nul-terminated string
		    char *s = va_arg(arg, char *);
		    int l = strlen(s);
		    if(prec && l > prec) {
			l = prec;
		    }
		    dst += strpad(dst, end - dst, s, l, width, left, ' ');
		    break;
		}
		case 'd': // signed integer as decimal
		    sign = (plus) ? '+' : (space) ? ' ' : '-';
		    // fallthrough
		case 'u': // unsigned integer as decimal
		    base = 10;
		    goto numeric;
		case 'x':
		case 'X': // unsigned integer as hex
		    base = 16;
		    goto numeric;
		case 'b': // unsigned integer as binary
		    base = 2;
		numeric: {
			char num[33], pad = ' ';
			if (zero && left == 0 && prec == 0) {
			    prec = width - 1; // have itoa() do the leading zero-padding for correct placement of sign
			    pad = '0';
			}
			int len = itoa(num, va_arg(arg, int), base, prec, 0, 0, sign);
			dst += strpad(dst, end - dst, num, len, width, left, pad);
			break;
		    }
		case 'F': { // signed integer and exponent as fixed-point decimal
		    char num[33], pad = (zero && left == 0) ? '0' : ' ';
		    int val = va_arg(arg, int);
		    int exp = va_arg(arg, int);
		    int len = itoa(num, val, 10, exp + 2, exp, (prec) ? prec : exp, (plus) ? '+' : (space) ? ' ' : '-');
		    dst += strpad(dst, end - dst, num, len, width, left, pad);
		    break;
		}
#if 0
		case 'e': { // LMIC event name
		    int ev = va_arg(arg, int);
		    const char *evn = (ev < sizeof(evnames) / sizeof(evnames[0]) && evnames[ev]) ? evnames[ev] : "UNKNOWN";
		    dst += strpad(dst, end - dst, evn, strlen(evn), width, left, ' ');
		    break;
		}
		case 'E': { // EUI64, lsbf (xx-xx-xx-xx-xx-xx-xx-xx)
		    char buf[23], *p = buf;
		    unsigned char *eui = va_arg(arg, unsigned char *);
		    for (int i = 7; i >= 0; i--) {
			p += itoa(p, eui[i], 16, 2, 0, 0, 0);
			if (i) *p++ = '-';
		    }
		    dst += strpad(dst, end - dst, buf, 23, width, left, ' ');
		    break;
		}
		case 't':   // ostime_t  (hh:mm:ss.mmm)
		case 'T': { // osxtime_t (ddd.hh:mm:ss)
		    char buf[12], *p = buf;
		    uint64_t t = ((c == 'T') ? va_arg(arg, uint64_t) : va_arg(arg, uint32_t)) * 1000 / OSTICKS_PER_SEC;
		    int ms = t % 1000;
		    t /= 1000;
		    int sec = t % 60;
		    t /= 60;
		    int min = t % 60;
		    t /= 60;
		    int hr = t % 24;
		    t /= 24;
		    int day = t;
		    if (c == 'T') {
			p += itoa(p, day, 10, 3, 0, 0, 0);
			*p++ = '.';
		    }
		    p += itoa(p, hr, 10, 2, 0, 0, 0);
		    *p++ = ':';
		    p += itoa(p, min, 10, 2, 0, 0, 0);
		    *p++ = ':';
		    p += itoa(p, sec, 10, 2, 0, 0, 0);
		    if (c == 't') {
			*p++ = '.';
			p += itoa(p, ms, 10, 3, 0, 0, 0);
		    }
		    dst += strpad(dst, end - dst, buf, 12, width, left, ' ');
		    break;
		}
#endif

		case 'h': { // buffer+length as hex dump (no field padding, but precision/maxsize truncation)
		    unsigned char *buf = va_arg(arg, unsigned char *);
		    int len = va_arg(arg, int);
		    char *top = (prec == 0 || dst + prec > end) ? end : dst + prec;
		    while (len--) {
			if ((len == 0 && top - dst >= 2) || top - dst >= 2 + space + 2) {
			    dst += itoa(dst, *buf++, 16, 2, 0, 0, 0);
			    if(space && len && dst < top) *dst++ = ' ';
			} else {
			    while (dst < top) *dst++ = '.';
			}
		    }
		    break;
		}
		default: // (also catch '\0')
		    goto stop;
	    }
	}
    }
 stop:
    *dst++ = 0;
    return dst - str - 1;
}

#define UART_RXFIFO_USABLE     (MXC_UART_FIFO_DEPTH-3)

/******************************************************************************/
uint32_t SYS_GetFreq(uint32_t clk_scale)
{
    uint32_t freq;
    unsigned int clkdiv;

    if (clk_scale == MXC_V_CLKMAN_CLK_SCALE_DISABLED) {
        freq = 0;
    } else {
        clkdiv = 1 << (clk_scale - 1);
        freq = SystemCoreClock / clkdiv;
    }

    return freq;
}

/******************************************************************************/
uint32_t SYS_UART_GetFreq(mxc_uart_regs_t *uart)
{
    return SYS_GetFreq(CLKMAN_GetClkScale(CLKMAN_CLK_UART));
}

/******************************************************************************/
int SYS_UART_Init(mxc_uart_regs_t *uart, const uart_cfg_t *uart_cfg, const sys_cfg_uart_t *sys_cfg)
{
    static int subsequent_call = 0;
    int err, idx;
    clkman_scale_t clk_scale;
    uint32_t min_baud;

    if(sys_cfg == NULL)
        return E_NULL_PTR;

    if (sys_cfg->clk_scale != CLKMAN_SCALE_AUTO) {
        CLKMAN_SetClkScale(CLKMAN_CLK_UART, sys_cfg->clk_scale);
    } else if (!subsequent_call) {
        /* This clock divider is shared amongst all UARTs. Only change it if it
         * hasn't already been configured. UART_Init() will check for validity
         * for this baudrate.
         */
        subsequent_call = 1;

        /* Setup the clock divider for the given baud rate */
        clk_scale = CLKMAN_SCALE_DISABLED;
        do {
            min_baud = ((SystemCoreClock >> clk_scale++) / (16 * (MXC_F_UART_BAUD_BAUD_DIVISOR >> MXC_F_UART_BAUD_BAUD_DIVISOR_POS)));
        } while (uart_cfg->baud < min_baud && clk_scale < CLKMAN_SCALE_AUTO);

        /* check if baud rate cannot be reached */
        if(uart_cfg->baud < min_baud)
            return E_BAD_STATE;

        CLKMAN_SetClkScale(CLKMAN_CLK_UART, clk_scale);
    }

    if ((err = IOMAN_Config(&sys_cfg->io_cfg)) != E_NO_ERROR) {
        return err;
    }

    /* Reset the peripheral */
    idx = MXC_UART_GET_IDX(uart);
    MXC_PWRMAN->peripheral_reset |= (MXC_F_PWRMAN_PERIPHERAL_RESET_UART0 << idx);
    MXC_PWRMAN->peripheral_reset &= ~((MXC_F_PWRMAN_PERIPHERAL_RESET_UART0 << idx));

    return E_NO_ERROR;
}


/* ************************************************************************* */
int UART_Init(mxc_uart_regs_t *uart, const uart_cfg_t *cfg, const sys_cfg_uart_t *sys_cfg)
{
    int err;
    int uart_num;
    uint32_t uart_clk;
    uint8_t baud_shift;
    uint16_t baud_div;
    uint32_t baud, diff_baud;
    uint32_t baud_1, diff_baud_1;

    // Check the input parameters
    uart_num = MXC_UART_GET_IDX(uart);
    MXC_ASSERT(uart_num >= 0);

    // Set system level configurations
    if(sys_cfg != NULL) {
        if ((err = SYS_UART_Init(uart, cfg, sys_cfg)) != E_NO_ERROR) {
            return err;
        }
    }

    // Initialize state pointers
    //rx_states[uart_num] = NULL;
    //tx_states[uart_num] = NULL;

    // Drain FIFOs and enable UART
    uart->ctrl = 0;
    uart->ctrl = (MXC_F_UART_CTRL_UART_EN | MXC_F_UART_CTRL_TX_FIFO_EN |
                  MXC_F_UART_CTRL_RX_FIFO_EN |
                  (UART_RXFIFO_USABLE <<  MXC_F_UART_CTRL_RTS_LEVEL_POS));

    // Configure data size, stop bit, parity, cts, and rts
    uart->ctrl |= ((cfg->size << MXC_F_UART_CTRL_DATA_SIZE_POS) |
                   (cfg->extra_stop << MXC_F_UART_CTRL_EXTRA_STOP_POS) |
                   (cfg->parity << MXC_F_UART_CTRL_PARITY_POS) |
                   (cfg->cts << MXC_F_UART_CTRL_CTS_EN_POS) |
                   (cfg->rts << MXC_F_UART_CTRL_RTS_EN_POS));

    // Configure the baud rate and divisor
    uart_clk = SYS_UART_GetFreq(uart);
    MXC_ASSERT(uart_clk > 0);

    baud_shift = 2;
    baud_div = (uart_clk / (cfg->baud * 4));

    // Can not support higher frequencies
    if(!baud_div) {
        return E_NOT_SUPPORTED;
    }

    // Decrease the divisor if baud_div is overflowing
    while(baud_div > 0xFF) {
        if(baud_shift == 0) {
            return E_NOT_SUPPORTED;
        }
        baud_shift--;
        baud_div = (uart_clk / (cfg->baud * (16 >> baud_shift)));
    }

    // Adjust baud_div so we don't overflow with the calculations below
    if(baud_div == 0xFF) {
        baud_div = 0xFE;
    }
    if(baud_div == 0) {
        baud_div = 1;
    }

    // Figure out if the truncation increased the error
    baud = (uart_clk / (baud_div * (16 >> baud_shift)));
    baud_1 = (uart_clk / ((baud_div+1) * (16 >> baud_shift)));

    if(cfg->baud > baud) {
        diff_baud = cfg->baud - baud;
    } else {
        diff_baud = baud - cfg->baud;
    }

    if(cfg->baud > baud_1) {
        diff_baud_1 = cfg->baud - baud_1;
    } else {
        diff_baud_1 = baud_1 - cfg->baud;
    }

    if(diff_baud < diff_baud_1) {
        uart->baud = ((baud_div & MXC_F_UART_BAUD_BAUD_DIVISOR) |
                      (baud_shift << MXC_F_UART_BAUD_BAUD_MODE_POS));
    } else {
        uart->baud = (((baud_div+1) & MXC_F_UART_BAUD_BAUD_DIVISOR) |
                      (baud_shift << MXC_F_UART_BAUD_BAUD_MODE_POS));
    }

    return E_NO_ERROR;
}

/* ************************************************************************* */
int UART_Write(mxc_uart_regs_t *uart, uint8_t* data, int len)
{
    int num, uart_num;
    mxc_uart_fifo_regs_t *fifo;

    uart_num = MXC_UART_GET_IDX(uart);
    MXC_ASSERT(uart_num >= 0);

    if(data == NULL) {
        return E_NULL_PTR;
    }

    // Make sure the UART has been initialized
    if(!(uart->ctrl & MXC_F_UART_CTRL_UART_EN)) {
        return E_UNINITIALIZED;
    }

    if(!(len > 0)) {
        return E_NO_ERROR;
    }

    // Lock this UART from writing
    //while(mxc_get_lock((uint32_t*)&tx_states[uart_num], 1) != E_NO_ERROR) {}

    // Get the FIFO for this UART
    fifo = MXC_UART_GET_FIFO(uart_num);

    num = 0;

    while(num < len) {

        // Wait for TXFIFO to not be full
        while((uart->tx_fifo_ctrl & MXC_F_UART_TX_FIFO_CTRL_FIFO_ENTRY) ==
                MXC_F_UART_TX_FIFO_CTRL_FIFO_ENTRY) {}

        // Write the data to the FIFO
#if(MXC_UART_REV == 0)
        uart->intfl = MXC_F_UART_INTFL_TX_DONE;
#endif
        fifo->tx = data[num++];
    }

    // Unlock this UART to write
    //mxc_free_lock((uint32_t*)&tx_states[uart_num]);

    return num;
}





#define DBG_USART MXC_UART0
#define BRD_DBG_UART	0

void debug_init (void) {
	uint32_t err;
	const uart_cfg_t uart_cfg = {
	    .parity = UART_PARITY_DISABLE,
	    .size = UART_DATA_SIZE_8_BITS,
	    .extra_stop = 0,
	    .cts = 0,
	    .rts = 0,
	    .baud = 115200,
	};

	const sys_cfg_uart_t uart_sys_cfg = {
	    .clk_scale = CLKMAN_SCALE_AUTO,
	    .io_cfg = IOMAN_UART(BRD_DBG_UART, IOMAN_MAP_A, IOMAN_MAP_UNUSED, IOMAN_MAP_UNUSED, 1, 0, 0)
	};

	err = UART_Init(DBG_USART, &uart_cfg, &uart_sys_cfg);
	MXC_ASSERT(err==0);
    debug_printf("\r\n============== DEBUG STARTED ==============\r\n");
}



void debug_str (const char* str) {
	int length=0;
	while(str[length++] != 0){}

    UART_Write(DBG_USART, str, length);
}

int debug_snprintf (char *str, int size, const char *format, ...) {
    va_list arg;
    int length;

    va_start(arg, format);
    length = debug_vsnprintf(str, size, format, arg);
    va_end(arg);
    return length;
}

void debug_printf (char const *format, ...) {
    char buf[256];
    va_list arg;

    va_start(arg, format);
    debug_vsnprintf(buf, sizeof(buf), format, arg);
    va_end(arg);
    debug_str(buf);
}



void debug_led (int val) {
#ifdef GPIO_DBG_LED
	if (val!=0) {
		GPIO_OutSet(&debug_led_pin);
	}
	else {
		GPIO_OutClr(&debug_led_pin);
	}
#endif
}


#endif
