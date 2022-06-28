/*     _____                _        __     _
 *    /__   \_ __ __ _  ___| | __ /\ \ \___| |_
 *      / /\/ '__/ _` |/ __| |/ //  \/ / _ \ __|
 *     / /  | | | (_| | (__|   '/ /\  /  __/ |_
 *     \_\  |_|  \__,_|\___|_|\_\_\ \_\\___|\__|
 *
 * Copyright (c) 2016-2018 Trackio International AG
 * All rights reserved.
 *
 */

#include "peripherals.h"

// Serial port API

static void start_rx (USART_TypeDef* usart); // fwd decl

// ------------------------------------------------
// TX

enum {
    FLAG_TXRX	= (1 << 0),
};

struct tx_t {
    unsigned int flags;
    const unsigned char* ptr;
    unsigned int n;

    osjob_t job;
    osjobcb_t cb;
};

static struct tx_t tx[4];

static int tx_cb (USART_TypeDef* usart, int status, void* arg) {
    switch (status) {
	case USART_CONTINUE:
	    if (tx[usartIndex(usart)].n) {
		tx[usartIndex(usart)].n -= 1;
		return *tx[usartIndex(usart)].ptr++;
	    }
	    break;
	case USART_DONE:
	    if (tx[usartIndex(usart)].flags & FLAG_TXRX) {
		start_rx(usart);
	    } else if (tx[usartIndex(usart)].cb) {
		os_setCallback(&tx[usartIndex(usart)].job, tx[usartIndex(usart)].cb);
	    }
	    break;
	default:
	    ASSERT(0);
    }
    return USART_DONE;
}

static void setup_tx (USART_TypeDef* usart, const void* buf, unsigned int n, osjobcb_t complete, unsigned int flags) {
    ASSERT(tx[usartIndex(usart)].n == 0);
    tx[usartIndex(usart)].ptr = buf;
    tx[usartIndex(usart)].n = n;
    tx[usartIndex(usart)].cb = complete;
    tx[usartIndex(usart)].flags = flags;
}

static void start_tx (USART_TypeDef* usart) {
    usart_send(usart, tx_cb, NULL);
}

void serial_tx (USART_TypeDef* usart, const void* buf, unsigned int n, osjobcb_t complete) {
    setup_tx(usart, buf, n, complete, 0);
    start_tx(usart);
}


// ------------------------------------------------
// RX

struct rx_t {
    unsigned char* ptr;
    unsigned int n;
    unsigned int max;
    ostime_t deadline;
    ostime_t silence;
    osjob_t job;
    osjobcb_t cb;
};

static struct rx_t rx[4];

static void rx_timeout_1 (osjob_t* job) {
    usart_abort_recv(USART1); // this will call rx_cb() if USART is active
}

static void rx_timeout_2 (osjob_t* job) {
    usart_abort_recv(USART2); // this will call rx_cb() if USART is active
}

static void rx_timeout_3 (osjob_t* job) {
    usart_abort_recv(USART4); // this will call rx_cb() if USART is active
}

static void rx_timeout_4 (osjob_t* job) {
    usart_abort_recv(LPUART1); // this will call rx_cb() if USART is active
}

static void* rx_timeout[4] = {
	&rx_timeout_1,
	&rx_timeout_2,
	&rx_timeout_3,
	&rx_timeout_4
};

static int rx_cb (USART_TypeDef* usart, int status, void* arg) {
    if (status >= 0) {
        rx[usartIndex(usart)].ptr[rx[usartIndex(usart)].n++] = status;
    }
    if (status < 0 || rx[usartIndex(usart)].n == rx[usartIndex(usart)].max) {
        rx[usartIndex(usart)].max = 0; // signals no RX in progress
        os_setCallback(&rx[usartIndex(usart)].job, rx[usartIndex(usart)].cb);
        return USART_DONE;
    } else {
        ostime_t now = os_getTime();
        ostime_t dd = (rx[usartIndex(usart)].deadline - now);
	if (rx[usartIndex(usart)].silence < dd) {
	    dd = rx[usartIndex(usart)].silence;
	}
	os_setTimedCallback(&rx[usartIndex(usart)].job, now + dd, rx_timeout[usartIndex(usart)]);
	return USART_CONTINUE;
    }
}

static void setup_rx (USART_TypeDef* usart, void* buf, unsigned int bufsz, ostime_t burst_silence, ostime_t timeout, osjobcb_t complete) {
    ASSERT(rx[usartIndex(usart)].max == 0);
    rx[usartIndex(usart)].ptr = buf;
    rx[usartIndex(usart)].n = 0;
    rx[usartIndex(usart)].max = bufsz;
    rx[usartIndex(usart)].deadline = timeout;
    rx[usartIndex(usart)].silence = burst_silence ?: timeout;
    rx[usartIndex(usart)].cb = complete;
}

static void start_rx (USART_TypeDef* usart) {
	rx[usartIndex(usart)].deadline += os_getTime();
    os_setTimedCallback(&rx[usartIndex(usart)].job, rx[usartIndex(usart)].deadline, rx_timeout[usartIndex(usart)]);
    usart_recv(usart, rx_cb, NULL);
}

void serial_rx (USART_TypeDef* usart, void* buf, unsigned int bufsz, ostime_t burst_silence, ostime_t timeout, osjobcb_t complete) {
    setup_rx(usart, buf, bufsz, burst_silence, timeout, complete);
    start_rx(usart);
}

unsigned int serial_rx_count (USART_TypeDef* usart) {
    return rx[usartIndex(usart)].n;
}


// ------------------------------------------------
// Combined TX/RX

void serial_tx_rx (USART_TypeDef* usart, const void* txbuf, unsigned int txn,
	void* rxbuf, unsigned int rxbufsz, ostime_t burst_silence, ostime_t timeout, osjobcb_t complete) {
    setup_tx(usart, txbuf, txn, NULL, FLAG_TXRX);
    setup_rx(usart, rxbuf, rxbufsz, burst_silence, timeout, complete);
    start_tx(usart);
}
