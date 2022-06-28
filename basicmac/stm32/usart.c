// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

#include "peripherals.h"

enum {
    RX_ON	= (1 << 0),
    TX_ON	= (1 << 1),
};

struct usart_t {
    unsigned int on;

    unsigned int br;

    usart_rx_func rx;
    void* rxarg;

    usart_tx_func tx;
    void* txarg;
};

static struct usart_t usart_ctrl[4];

unsigned int gpio_tx(USART_TypeDef* usart) {
	#ifdef BRD_USART1
	if(usart == USART1)
		return GPIO_USART1_TX;
	#endif
	#ifdef BRD_USART2
	if(usart == USART2)
		return GPIO_USART2_TX;
	#endif

	#ifdef BRD_USART4
	if(usart == USART4)
		return GPIO_USART4_TX;
	#endif

	#ifdef BRD_LPUART1
	if(usart == LPUART1)
		return GPIO_LPUART1_TX;
	#endif

	ASSERT(0);
	return 0;
}

unsigned int gpio_rx(USART_TypeDef* usart) {
	#ifdef BRD_USART1
	if(usart == USART1)
		return GPIO_USART1_RX;
	#endif
	#ifdef BRD_USART2
	if(usart == USART2)
		return GPIO_USART2_RX;
	#endif

	#ifdef BRD_USART4
	if(usart == USART4)
		return GPIO_USART4_RX;
	#endif

	#ifdef BRD_LPUART1
	if(usart == LPUART1)
		return GPIO_LPUART1_RX;
	#endif

	return 0;
}

unsigned int usartIndex(USART_TypeDef* usart) {
  if(usart == USART1) {
    return 0;
  }
  else if(usart == USART2) {
    return 1;
  }
  else if(usart == USART4) {
    return 2;
  }
  else if(usart == LPUART1) {
    return 3;
  }
  return 0;
}

static IRQn_Type usart_getirq (USART_TypeDef* usart) {
  if(usart == USART1) {
    return USART1_IRQn;
  }
  else if(usart == USART2) {
    return USART2_IRQn;
  }
  else if(usart == USART4) {
    return USART4_5_IRQn;
  }
  else if(usart == LPUART1) {
    return LPUART1_IRQn;
  }
  return 0;
}

static void usart_enable (USART_TypeDef* usart) {
  if(usart == USART1) {
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
  }
  else if(usart == USART2) {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  }
  else if(usart == USART4) {
    RCC->APB1ENR |= RCC_APB1ENR_USART4EN;
  }
  else if(usart == LPUART1) {
    RCC->APB1ENR |= RCC_APB1ENR_LPUART1EN;
  }
}

static void usart_disable (USART_TypeDef* usart) {
  if(usart == USART1) {
    RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN;
  }
  else if(usart == USART2) {
    RCC->APB1ENR &= ~RCC_APB1ENR_USART2EN;
  }
  else if(usart == USART4) {
    RCC->APB1ENR &= ~RCC_APB1ENR_USART4EN;
  }
  else if(usart == LPUART1) {
    RCC->APB1ENR &= ~RCC_APB1ENR_LPUART1EN;
  }
}

static void usart_on (USART_TypeDef* usart, unsigned int flag) {
	hal_disableIRQs();
	if (usart_ctrl[usartIndex(usart)].on == 0) {
		// disable sleep (keep clock at full speed during transfer
		hal_setMaxSleep(HAL_SLEEP_S0);
		// enable peripheral clock
		usart_enable(usart);
		// set baudrate
		usart->BRR = usart_ctrl[usartIndex(usart)].br;
		// usart enable
		usart->CR1 = USART_CR1_UE;
		// enable interrupts in NVIC
		NVIC_EnableIRQ(usart_getirq(usart));
	}
	usart_ctrl[usartIndex(usart)].on |= flag;
	hal_enableIRQs();
}

static void usart_off (USART_TypeDef* usart, unsigned int flag) {
	hal_disableIRQs();
	usart_ctrl[usartIndex(usart)].on &= ~flag;
	if (usart_ctrl[usartIndex(usart)].on == 0) {
		// disable USART
		usart->CR1 = 0;
		// disable peripheral clock
		usart_disable(usart);
		// disable interrupts in NVIC
		NVIC_DisableIRQ(usart_getirq(usart));
		// re-enable sleep
		hal_clearMaxSleep(HAL_SLEEP_S0);
	}
	hal_enableIRQs();
}

static void rx_on (USART_TypeDef* usart, unsigned int noirq) {
	// turn on usart
	usart_on(usart, RX_ON);
	// enable receiver
	usart->CR1 |= USART_CR1_RE;
	// setup I/O line
	CFG_PIN_AF(gpio_rx(usart), GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_NONE);
	if (noirq == 0) {
		// flush data, clear ORE and enable receive interrupt
		usart->RQR |= USART_RQR_RXFRQ;
		usart->ICR |= USART_ISR_ORE;
		usart->CR1 |= USART_CR1_RXNEIE;
	}
}

static void rx_off (USART_TypeDef* usart) {
	// deconfigure I/O line
	CFG_PIN_DEFAULT(gpio_rx(usart));
	// disable receiver and interrupts
	usart->CR1 &= ~(USART_CR1_RE | USART_CR1_RXNEIE);
	// turn off usart
	usart_off(usart, RX_ON);
}

static void tx_on (USART_TypeDef* usart) {
	// turn on usart
	usart_on(usart, TX_ON);
	// enable transmitter
	usart->CR1 |= USART_CR1_TE;
	// setup I/O line
	CFG_PIN_AF(gpio_tx(usart), GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_NONE);
	// enable interrupt
	usart->CR1 |= USART_CR1_TXEIE;
}

static void tx_off (USART_TypeDef* usart) {
	// deconfigure I/O line, activate pullup
	CFG_PIN(gpio_tx(usart), GPIOCFG_MODE_INP | GPIOCFG_OSPEED_400kHz | GPIOCFG_OTYPE_OPEN | GPIOCFG_PUPD_PUP);
	// disable receiver and interrupts
	usart->CR1 &= ~(USART_CR1_TE | USART_CR1_TXEIE);
	// turn off usart
	usart_off(usart, TX_ON);
}

void usart_init (void* usart_void) {
	USART_TypeDef* usart = (USART_TypeDef*) usart_void;
	usart_ctrl[usartIndex(usart)].on = 0;
	// activate pullup on tx line
	CFG_PIN(gpio_tx(usart), GPIOCFG_MODE_INP | GPIOCFG_OSPEED_400kHz | GPIOCFG_OTYPE_OPEN | GPIOCFG_PUPD_PUP);
}

void usart_cfg (USART_TypeDef* usart, unsigned int br) {
  usart_ctrl[usartIndex(usart)].br = br;
}

void usart_recv (USART_TypeDef* usart, usart_rx_func rx, void* arg) {
  usart_ctrl[usartIndex(usart)].rx = rx;
  usart_ctrl[usartIndex(usart)].rxarg = arg;
	rx_on(usart, 0);
}

void usart_abort_recv (USART_TypeDef* usart) {
	hal_disableIRQs();
	if (usart_ctrl[usartIndex(usart)].on & RX_ON) {
		rx_off(usart);
		usart_ctrl[usartIndex(usart)].rx(usart, USART_ERROR, usart_ctrl[usartIndex(usart)].rxarg);
	}
	hal_enableIRQs();
}

void usart_send (USART_TypeDef* usart, usart_tx_func tx, void* arg) {
  usart_ctrl[usartIndex(usart)].tx = tx;
  usart_ctrl[usartIndex(usart)].txarg = arg;
	tx_on(usart);
}

static void usart_irq (USART_TypeDef* usart) {
	unsigned int isr = usart->ISR;
	unsigned int cr1 = usart->CR1;
	if (cr1 & USART_CR1_RXNEIE) {
		if (isr & USART_ISR_ORE) {
			usart->ICR |= USART_ISR_ORE;
			rx_off(usart);
			usart_ctrl[usartIndex(usart)].rx(usart, USART_ERROR, usart_ctrl[usartIndex(usart)].rxarg);
		} else if (isr & USART_ISR_RXNE) {
			if (usart_ctrl[usartIndex(usart)].rx(usart, usart->RDR, usart_ctrl[usartIndex(usart)].rxarg) != USART_CONTINUE) { // done
				rx_off(usart);
			}
		}
	}
	if ((cr1 & USART_CR1_TXEIE) && (isr & USART_ISR_TXE)) {
		int ch;
		if ((ch = usart_ctrl[usartIndex(usart)].tx(usart, USART_CONTINUE, usart_ctrl[usartIndex(usart)].txarg)) < 0) { // done
			unsigned int cr1 = usart->CR1;
			cr1 = (cr1 & ~USART_CR1_TXEIE) | USART_CR1_TCIE;
			usart->CR1 = cr1;
		} else {
			usart->TDR = ch;
		}
	}
	if ((cr1 & USART_CR1_TCIE) && (isr & USART_ISR_TC)) {
		usart->CR1 &= ~USART_CR1_TCIE;
		tx_off(usart);
		usart_ctrl[usartIndex(usart)].tx(usart, USART_DONE, usart_ctrl[usartIndex(usart)].txarg);
	}
}

void usart_irq_handler_1 (void) {
  usart_irq(USART1);
}

void usart_irq_handler_2 (void) {
  usart_irq(USART2);
}

void usart_irq_handler_4 (void) {
  usart_irq(USART4);
}

int usart_wait_silence (USART_TypeDef* usart, int silence_ticks, int timeout_ticks) {
	ostime_t deadline = os_getTime() + timeout_ticks;
	ostime_t threshold = os_getTime() + silence_ticks;
	int retval = 0;
	rx_on(usart, 1);
	while (1) {
		if (usart->ISR & USART_ISR_BUSY) {
			threshold = os_getTime() + silence_ticks;
		}
		if ((deadline - os_getTime()) < 0) {
			retval = -1;
			break;
		}
		if ((threshold - os_getTime()) < 0) {
			retval = 0;
			break;
		}
	}
	rx_off(usart);
	return retval;
}
