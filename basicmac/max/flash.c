/*     _____                _        __     _
 *    /__   \_ __ __ _  ___| | __ /\ \ \___| |_
 *      / /\/ '__/ _` |/ __| |/ //  \/ / _ \ __|
 *     / /  | | | (_| | (__|   '/ /\  /  __/ |_
 *     \_\  |_|  \__,_|\___|_|\_\_\ \_\\___|\__|
 *
 * Copyright (c) 2016-2017 Trackio International AG
 * All rights reserved.
 *
 */

#include "hw.h"
#include "boot/bootloader.h"

void flash_wword (uint32_t* dst, uint32_t word) {
#if 0
    HAL_boottab->write_flash(dst, &word, 1, 0);
#endif
}

void flash_buffered_write (flash_bw_state* state, uint32_t* src, uint32_t nwords) {
#if 0
	if (src == NULL) { // flush
	if (state->off > 0) {
	    hal_disableIRQs();
	    HAL_boottab->write_flash(state->base, state->buf, state->off, BOOT_WF_PAGEERASE);
	    hal_enableIRQs();
	}
    } else {
	while (nwords > 0) {
	    uint32_t n = PAGE_NW - state->off;
	    if (n > nwords) {
		n = nwords;
	    }
	    nwords -= n;
	    while (n-- > 0) {
		state->buf[state->off++] = *src++;
	    }
	    if (state->off == PAGE_NW) {
		hal_disableIRQs();
		HAL_boottab->write_flash(state->base, state->buf, PAGE_NW, BOOT_WF_PAGEERASE);
		hal_enableIRQs();
		state->base += PAGE_NW;
		state->off = 0;
	    }
	}
    }
#endif
}
