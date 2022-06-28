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

#include "lmic.h"
#include "hw.h"
#include "board.h"
#include "boot/bootloader.h"

#include "perso.h"

#if defined(CFG_perso)

#if BRD_PERSO_UART == 1
#define USART USART1
#define USART_enable()		do { RCC->APB2ENR |= RCC_APB2ENR_USART1EN; } while (0)
#define USART_disable()		do { RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN; } while (0)
#elif BRD_PERSO_UART == 2
#define USART USART2
#define USART_enable()		do { RCC->APB1ENR |= RCC_APB1ENR_USART2EN; } while (0)
#define USART_disable()		do { RCC->APB1ENR &= ~RCC_APB1ENR_USART2EN; } while (0)
#elif BRD_PERSO_UART == 4
#define USART USART4
#define USART_enable()		do { RCC->APB1ENR |= RCC_APB1ENR_USART4EN; } while (0)
#define USART_disable()		do { RCC->APB1ENR &= ~RCC_APB1ENR_USART4EN; } while (0)
#endif

static void uart_config (int abr) {
    // configure USART (115200/8N1)
    USART_enable();
    USART->BRR = 278; // 115200 (APB1 clock @32MHz)
    if (abr) {
	// enable auto-baud rate detection
	USART->CR2 = USART_CR2_ABREN
	    | USART_CR2_ABRMODE_0 | USART_CR2_ABRMODE_1;
    }
    USART->CR3 |= USART_CR3_OVRDIS; // disable overrun detection
    USART->CR1 = USART_CR1_UE | USART_CR1_TE; // usart+transmitter enable
    CFG_PIN_AF(GPIO_PERSO_TX, GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_NONE);
    CFG_PIN_AF(GPIO_PERSO_RX, GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_NONE);
}

static void uart_deconfig (void) {
    // wait until we are done sending
    while ((USART->ISR & USART_ISR_TC) == 0);
    // disable peripheral function of PINs
    CFG_PIN_DEFAULT(GPIO_PERSO_RX);
    CFG_PIN_DEFAULT(GPIO_PERSO_TX);
    USART->CR1 = 0; // disable usart
    USART->CR3 = 0; // reset CR3
    // remove power/clock from peripheral
    USART_disable();
}

static void uart_send (unsigned char c) {
    while ((USART->ISR & USART_ISR_TXE) == 0);
    USART->TDR = c;
}

static int uart_recv (void) {
    while ((USART->ISR & USART_ISR_RXNE) == 0);
    return USART->RDR;
}

static void cobs_send (unsigned char* buf, int n) {
    while (n >= 0) {
	int rl;
	for (rl = 0; rl < n && buf[rl] != 0; rl++);
	uart_send(rl + 1);
	n -= (rl + 1);
	while (rl-- > 0) {
	    uart_send(*buf++);
	}
	buf += 1; // skip 0
    }
    uart_send(0);
}

static int cobs_recv (unsigned char* buf) {
    // enable receiver
    USART->CR1 |= USART_CR1_RE;

    int n;
sync:
    n = -1;
    while (1) {
	// read next chunk byte
	int rl = uart_recv();
	if (rl == 0) {
	    if (n < 0) {
		goto sync;
	    } else {
		// disable receiver
		USART->CR1 &= ~USART_CR1_RE;
		return n;
	    }
	} else if (n + rl > 252) {
	    // resync
	    while (uart_recv() != 0);
	    goto sync;
	} else if (n >= 0) {
	    buf[n] = 0;
	}
	n += 1;
	while (--rl > 0) {
	    int b = uart_recv();
	    if (b == 0) {
		goto sync;
	    }
	    buf[n++] = b;
	}
    }
}

static int make_packet (unsigned char* buf, int code, int len) {
    buf[0] = code;
    buf[3] = len;
    int i, crcoff = 4 + ((len + 3) & ~3);
    for (i = 4 + len; i < crcoff; i++) {
	buf[i] = 0xff;
    }
    *((uint32_t*) (buf + crcoff)) = crc32(buf, crcoff / 4);
    return crcoff + 4;
}

static void wtx (unsigned char* buf) {
    int n;
    n = make_packet(buf, RSP_WTX, 0);
    cobs_send(buf, n);
}

// return: len, -1 on error
static int check_packet (unsigned char* buf, int n) {
    int len;
    if ((n & 3) != 0 || n < 8) {
	return -1;
    }
    len = buf[3];
    if (n != 8 + ((len + 3) & ~3)) {
	return -1;
    }
    // Note: We're skipping verification of padding bytes,
    // as this is not security critical.
    if (crc32(buf, (n / 4) - 1) != *((uint32_t*) (buf + n - 4))) {
	return -1;
    }
    return len;
}

enum {
    CMD_NOP		= 0x00,
    CMD_RUN		= 0x01,
    CMD_RESET		= 0x02,
    CMD_FWINFO		= 0x03,
    CMD_EEPROM		= 0x04,

    CMD_SLEEP		= 0x10,

    CMD_WP_TARGET	= 0x20,
    CMD_WP_SN		= 0x21,
    CMD_WP_DEVEUI	= 0x22,
    CMD_WP_JOINEUI	= 0x23,
    CMD_WP_NWKKEY	= 0x24,
    CMD_WP_APPKEY	= 0x25,

    CMD_RP_TARGET	= 0x30,
    CMD_RP_SN		= 0x31,
    CMD_RP_DEVEUI	= 0x32,
    CMD_RP_JOINEUI	= 0x33,
    CMD_RP_NWKKEY	= 0x34,
    CMD_RP_APPKEY	= 0x35,

    CMD_GPIO		= 0x40,
    CMD_GPIO_WAIT	= 0x41,
    CMD_ADC		= 0x42,
    CMD_LED		= 0x43,
    CMD_VIBE		= 0x44,
    CMD_I2C		= 0x45,
    CMD_PIR		= 0x46,

    CMD_LORA_HOP	= 0x50,

    CMD_SX_RESET	= 0x80,
    CMD_SX_RD_REG	= 0x81,
    CMD_SX_WR_REG	= 0x82,
    CMD_SX_SW_TX	= 0x83,
};

static void write_e2 (void* dst, void* src, int nwords) {
    // setup word pointers
    volatile uint32_t* d = dst;
    uint32_t* s = src;
    int i;

    // unlock data eeprom memory and registers
    FLASH->PEKEYR = 0x89ABCDEF; // FLASH_PEKEY1
    FLASH->PEKEYR = 0x02030405; // FLASH_PEKEY2

    // copy data to eeprom and verify
    for (i = 0; i < nwords; i++) {
	if (d[i] != s[i]) {
	    d[i] = s[i];
	    while (FLASH->SR & FLASH_SR_BSY);
	    if (d[i] != s[i]) {
		hal_failed();
	    }
	}
    }

    // re-lock data eeprom memory and registers
    FLASH->PECR |= FLASH_PECR_PELOCK;
}

static volatile unsigned int ev;
static void ev_reset (void) {
    ev = 0;
}
static void ev_set (osjob_t* j) {
    ev = 1;
}
static int ev_wait (ostime_t ticks) {
    ostime_t tt = os_getTime() + ticks;
    while (ev == 0) {
       if ((tt - os_getTime()) < 0) {
	   return 1;
       }
    }
    return 0;
}
#define EV_CHECK_WAIT(sec)	do { if (ev_wait(sec2osticks(sec))) { n = RSP_EINTERNAL; goto empty; } } while (0)

#ifdef BRD_I2C
static int do_i2c (int addr, int wlen, unsigned char* wbuf, int rlen, unsigned char* rbuf) {
    unsigned char buf[rlen > wlen ? rlen : wlen];
    volatile int status;
    memcpy(buf, wbuf, wlen);
    ev_reset();
    i2c_xfer_ex(addr, buf, wlen, rlen, 0, NULL, ev_set, (int*) &status);
    if (ev_wait(sec2osticks(2))) {
	i2c_abort();
    }
    if (status == I2C_OK) {
	memcpy(rbuf, buf, rlen);
	return rlen;
    } else {
	return -1;
    }
}
#endif

static void nexthop (osjob_t* job); // fwd decl

static void txhop (osjob_t* job) {
    LMIC_updateTx(os_getTime());
    LMIC.osjob.func = nexthop;
    hal_waitUntil(LMIC.txend);
    os_radio(RADIO_TX);
}

static void nexthop (osjob_t* job) {
    ostime_t n = os_getTime();
    LMIC.opmode |= OP_NEXTCHNL;
    LMIC.txend = LMIC_nextTx(n);
    os_setTimedCallback(&LMIC.osjob, LMIC.txend,
	    ((LMIC.txend == n) || ((LMIC.opmode & OP_NEXTCHNL) == 0)) ? txhop : nexthop);
}

static void hop (void) {
    radio_init();
    //debug_init();

#if defined(RD_base_etsi)
    // XXX - it would be good if this is already done within LMiC
#if defined(CFG_as923)
    if (os_getRegDomain() == RD_JP) {
	LMIC_setupBand(BAND_DEFAULT, 13, 1); // 13 dBm, no duty cycle
    } else {
	LMIC_setupBand(BAND_DEFAULT, 16, 1); // 16 dBm, no duty cycle
    }
#else
    LMIC_setupBand(BAND_MILLI, 14, 1000); // 0.1%
    LMIC_setupBand(BAND_CENTI, 14, 100);  // 1%
    LMIC_setupBand(BAND_DECI,  27, 10);   // 10%
#endif
#endif

    os_setCallback(&LMIC.osjob, nexthop);
    //os_setCallback(&LMIC.osjob, testdelay);

    // enable receiver
    USART->CR1 |= USART_CR1_RE;

    hal_setMaxSleep(HAL_SLEEP_S0);
    while (1) {
	os_runstep();
	if ((USART->ISR & USART_ISR_RXNE) != 0) {
	    // stop
	    radio_reset();
	    os_clearCallback(&LMIC.osjob);
	    break;
	}
    }
    hal_clearMaxSleep(HAL_SLEEP_S0);
}

void perso_enter (void) {
    // TODO - check security bit!
    // configure RX GPIO as input with pull-down
    CFG_PIN(GPIO_PERSO_RX, GPIOCFG_MODE_INP | GPIOCFG_OSPEED_40MHz | GPIOCFG_PUPD_PDN);
    hal_waitUntil(os_getTime() + us2osticks(100));
    if (GET_PIN(GPIO_PERSO_RX) == 0) {
	// no PTE detected
	CFG_PIN_DEFAULT(GPIO_PERSO_RX);
    } else {
	// configure UART
	enum {
	    NOT_DONE,
	    DONE,
	    RESET
	};
	int state = NOT_DONE;
	uart_config(1);
	do {
	    unsigned char buf[252] __attribute__((aligned(4)));
	    int n = cobs_recv(buf);
	    if ((n = check_packet(buf, n)) < 0) {
		continue;
	    }
	    switch (buf[0]) {
		case CMD_NOP:
		    n = 0x7f;
		    goto empty;
		case CMD_RUN:
		    state = DONE;
		    goto ok;
		case CMD_RESET:
		    state = RESET;
		    goto ok;
		case CMD_FWINFO:
		    if (n != 0) {
			goto eparam;
		    } else {
			uint32_t* p = (uint32_t*) (buf + 4);
			p[0] = HAL_boottab->version;
			p[1] = BOOT_INITTAB->version;
			p[2] = BOOT_INITTAB->crc;
			p[3] = BOOT_INITTAB->size;
			p[4] = BOOT_INITTAB->target;
			n = make_packet(buf, RSP_OK, 20);
			break;
		    }
		case CMD_EEPROM:
		    if (n < 4) {
			goto eparam;
		    } else {
			u4_t off = os_rlsbf2(buf + (4 + 0));
			u4_t len = os_rlsbf2(buf + (4 + 2));
			if (len > 128 || (off + len) > EEPROM_SZ) {
			    goto eparam;
			}
			if (n > 4) {
			    if ((n - 4) != len || (len & 3) != 0 || (off & 3) != 0) {
				goto eparam;
			    }
			    // write eeprom
			    write_e2(((unsigned char*) EEPROM_BASE) + off, buf + 8, len >> 2);
			    goto ok;
			} else {
			    // read eeprom
			    memcpy(buf + 4, ((unsigned char*) EEPROM_BASE) + off, len);
			    n = make_packet(buf, RSP_OK, len);
			    break;
			}
		    }
		case CMD_SLEEP:
		    if (n != 2 || (buf[4] & ~1) != 0) {
			goto eparam;
		    } else {
			int secs = buf[5];
			ostime_t tt = os_getTime() + sec2osticks(secs);
			uart_deconfig();
			while (secs == 0 || (tt - os_getTime()) > 0) {
			    hal_disableIRQs();
			    hal_sleep(secs == 0 ? HAL_SLEEP_FOREVER : buf[4] ? HAL_SLEEP_APPROX : HAL_SLEEP_EXACT, tt);
			    hal_enableIRQs();
			}
			uart_config(0);
			goto ok;
		    }
		case CMD_GPIO:
		    if (n != 2 || (buf[4] >> 4) > 2) {
			goto eparam;
		    }
		    if ((buf[5] & 3) != 3) { // configure GPIO
			unsigned int cfg = 0;
			switch ((buf[5] >> 2) & 3) {
			    case 0: cfg |= GPIOCFG_MODE_ANA; break;
			    case 1: cfg |= GPIOCFG_MODE_INP; break;
			    case 2:
			    case 3: cfg |= (GPIOCFG_MODE_OUT
					    | GPIOCFG_OSPEED_40MHz
					    | GPIOCFG_OTYPE_PUPD); break;
			}
			if (buf[5] & 1) {
			    cfg |= GPIOCFG_PUPD_PDN;
			} else if (buf[5] & 2) {
			    cfg |= GPIOCFG_PUPD_PUP;
			}
			if (buf[5] & 8) {
			    gpio_set_pin(buf[4] >> 4, buf[4] & 0xf, buf[5] & 4);
			}
			gpio_cfg_pin(buf[4] >> 4, buf[4] & 0xf, cfg);
		    }
		    buf[4] = gpio_get_pin(buf[4] >> 4, buf[4] & 0xf) ? 1 : 0;
		    n = make_packet(buf, RSP_OK, 1);
		    break;
		case CMD_GPIO_WAIT:
		    if (n != 2 || (buf[4] >> 4) > 2) {
			goto eparam;
		    } else {
			ostime_t deadline = os_getTime() + sec2osticks(buf[5] >> 1);
			ostime_t twtx = os_getTime() + sec2osticks(1);
			uint32_t gpio = buf[4];
			uint32_t on = (buf[5] & 1);
			while (on != (gpio_get_pin(gpio >> 4, gpio & 0xf) ? 1 : 0)) {
			    ostime_t now = os_getTime();
			    if ((deadline - now) < 0) {
				n = RSP_ETIMEOUT;
				goto empty;
			    }
			    if ((twtx - now) < 0) {
				wtx(buf);
				twtx = os_getTime() + sec2osticks(1);
			    }
			}
			goto ok;
		    }
		case CMD_ADC:
		    if (n != 1 || (buf[4] > 18)) { // 19 chan max
			goto eparam;
		    } else {
			unsigned int v = adc_read(buf[4]);
			if (buf[4] == 17) {
			    unsigned int cal = *VREFINT_CAL_ADDR;
			    buf[6] = cal;
			    buf[7] = cal >> 8;
			    n = 4;
			} else {
			    n = 2;
			}
			buf[4] = v;
			buf[5] = v >> 8;
			n = make_packet(buf, RSP_OK, n);
			break;
		    }
		case CMD_LED:
		    if (n != 3) {
			goto eparam;
		    } else {
			u2_t dc = os_rlsbf2(buf + (4 + 1));
			unsigned int gpio;
			switch (buf[4]) {
#ifdef GPIO_LED1
			    case 1: gpio = GPIO_LED1; break;
#endif
#ifdef GPIO_LED2
			    case 2: gpio = GPIO_LED2; break;
#endif
			    default:
				goto eparam;
			}
			if (dc == 0) {
			    leds_set(gpio, 0);
			} else if (dc == 0xffff) {
			    leds_set(gpio, 1);
			} else {
			    leds_pwm(gpio, dc);
			}
			goto ok;
		    }
#ifdef BRD_VIBE_TIM
		case CMD_VIBE:
		    if (n != 2) {
			goto eparam;
		    }
		    vibe_set(os_rlsbf2(buf + 4));
		    goto ok;
#endif
#ifdef BRD_I2C
		case CMD_I2C:
		    if (n < 2) {
			goto eparam;
		    }
		    n = do_i2c(buf[4], n - 2, buf + 6, buf[5], buf + 4);
		    if (n < 0) {
			n = RSP_EINTERNAL;
			goto empty;
		    }
		    n = make_packet(buf, RSP_OK, n);
		    break;
#endif
#ifdef BRD_PIR_TIM
		case CMD_PIR:
		    if (n == 0) { // read out
			unsigned int config, status;
			pir_init();
			ev_reset();
			pir_readout(NULL, ev_set);
			EV_CHECK_WAIT(2);
			pir_get(&config, &status);
			os_wlsbf4(buf + 4, config);
			os_wlsbf4(buf + 8, status);
			n = make_packet(buf, RSP_OK, 8);
			break;
		    } else if (n == 4) { // config
			ev_reset();
			pir_config(os_rlsbf4(buf + 4), NULL, ev_set);
			EV_CHECK_WAIT(2);
			goto ok;
		    }
		    goto eparam;
#endif
		case CMD_LORA_HOP:
#if defined(RD_base_fcc)
		    // [ 4] 1B - type (0=us915)
		    // [ 5] 1B - reserved
		    // [ 6] 2B - chmask-lo
		    // [ 8] 2B - chmask-ml
		    // [10] 2B - chmask-mh
		    // [12] 2B - chmask-hi
		    // [14] 2B - rps
		    // [16] xB - payload
		    if (n > 12 && buf[4] == 0) {
			LMIC.txpow = buf[5];
			LMIC.channelMap[0] = os_rlsbf2(buf +  6);
			LMIC.channelMap[1] = os_rlsbf2(buf +  8);
			LMIC.channelMap[2] = os_rlsbf2(buf + 10);
			LMIC.channelMap[3] = os_rlsbf2(buf + 12);
			LMIC.rps = os_rlsbf2(buf + 14);
			memcpy(LMIC.frame, buf + 16, LMIC.dataLen = (n - 12));
			goto hop;
		    }
#elif defined(RD_base_etsi)
		    // [ 4] 1B - type (1=eu868)
		    // [ 5] 1B - chcnt
		    // [ 4] 2B - rps
		    // [ 8] 4B - freq
		    // [ x] xB - payload
		    if (n > 2 && buf[4] == 1) {
			int chcnt = buf[5];
			LMIC.rps = os_rlsbf2(buf + 6);
			if (n > 2 + (chcnt * 4)) {
			    int i;
			    for (i = 0; i < chcnt; i++) {
				LMIC_setupChannel(i, os_rlsbf4(buf + 8 + (i * 4)), 0, -1);
			    }
			    memcpy(LMIC.frame, buf + 8 + (chcnt * 4), LMIC.dataLen = (n - (4 + (chcnt * 4))));
			    goto hop;
			}
		    }
#endif
		    goto eparam;
hop:
		    n = make_packet(buf, RSP_OK, 0);
		    cobs_send(buf, n);
		    hop();
		    continue;
		case CMD_SX_RESET:
		    radio_reset();
		    goto ok;
		case CMD_SX_RD_REG:
		    if (n != 2 || (n = buf[4 + 1]) > 236) {
			goto eparam;
		    }
		    radio_readBuf(buf[4 + 0], buf + 4, n);
		    n = make_packet(buf, RSP_OK, n);
		    break;
		case CMD_SX_WR_REG:
		    if (n < 2) {
			goto eparam;
		    }
		    radio_writeBuf(buf[4 + 0], buf + 4 + 1, n - 1);
		    goto ok;
		case CMD_SX_SW_TX:
		    if (n != 1) {
			goto eparam;
		    } else {
			s1_t v = buf[4];
			if(v != -1 && v != 0 && v != 1) {
			    goto eparam;
			} else {
			    hal_pin_rxtx(v);
			    goto ok;
			}
		    }
		default:
		    n = RSP_NOTIMPL;
		    goto empty;
eparam:
		    n = RSP_EPARAM;
		    goto empty;
ok:
		    n = RSP_OK;
empty:
		    n = make_packet(buf, n, 0);
	    }
	    cobs_send(buf, n);
	} while (state == NOT_DONE);
	uart_deconfig();
	if (state == RESET) {
	    NVIC_SystemReset();
	}
    }
}

#endif
