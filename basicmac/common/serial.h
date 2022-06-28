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

#ifndef _serial_h_
#define _serial_h_

#include "lmic.h"

void serial_tx (USART_TypeDef* usart, const void* buf, unsigned int n, osjobcb_t complete);
void serial_rx (USART_TypeDef* usart, void* buf, unsigned int bufsz, ostime_t burst_silence, ostime_t timeout, osjobcb_t complete);
void serial_tx_rx (USART_TypeDef* usart, const void* txbuf, unsigned int txn,
	void* rxbuf, unsigned int rxbufsz, ostime_t burst_silence, ostime_t timeout, osjobcb_t complete);
unsigned int serial_rx_count (USART_TypeDef* usart);

#endif
