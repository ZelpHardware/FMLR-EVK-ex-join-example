/*
 * Copyright (c) 2016 Trackio International AG
 * All rights reserved.
 *
 * Copyright (c) 2014-2016 IBM Corporation.
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of the <organization> nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "hw.h"

#ifdef BRD_SPI

#if BRD_SPI == 1
#define SPIx                    SPI1
#define SPIx_enable()           do { RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; } while (0)
#define SPIx_disable()          do { RCC->APB2ENR &= ~RCC_APB2ENR_SPI1EN; } while (0)
#define RCC_APB2ENR_SPIxEN      RCC_APB2ENR_SPI1EN
#elif BRD_SPI == 2
#define SPIx                    SPI2
#define SPIx_enable()           do { RCC->APB1ENR |= RCC_APB1ENR_SPI2EN; } while (0)
#define SPIx_disable()          do { RCC->APB1ENR &= ~RCC_APB1ENR_SPI2EN; } while (0)
#else
#error "Unsupported value for BRD_SPI"
#endif

#define BRD_GPIO_NONE		    (1 << 31)

void spi_init (u1_t mode, u1_t br, u1_t lsb_first) {
    // enable clock for SPI interface
    SPIx_enable();

    // configure and activate the SPI (master, internal slave select, software slave mgmt)
    // (use default mode: 8-bit, 2-wire, no crc, MSBF, PCLK32/4=8MHz), CPOL, CPHA given by mode
    SPIx->CR1 = SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM | SPI_CR1_SPE | (mode & 0x03) | (br & (7<<3)) |
            (lsb_first ? SPI_CR1_LSBFIRST : 0);

    // configure I/O lines and disable clock
    spi_cs_io(BRD_GPIO_NONE, 0);
}

void spi_cs_io(u4_t cs, int on) {
    if (on) {
        // enable clock for SPI interface
        SPIx_enable();
        // configure pins for alternate function SPIx (SCK, MISO, MOSI)
        CFG_PIN_AF(GPIO_BRD_SCK, GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_PDN);
        CFG_PIN_AF(GPIO_BRD_MISO, GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_PDN);
        CFG_PIN_AF(GPIO_BRD_MOSI, GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_PDN);

        if (cs != BRD_GPIO_NONE) {
			// drive chip select low
            SET_PIN_ONOFF(cs, 1);
			CFG_PIN(cs, GPIOCFG_MODE_OUT | GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_NONE);
        }
    } else {
        if (cs != BRD_GPIO_NONE) {
			// stop driving chip select, activate pull-up
            CFG_PIN_DEFAULT(cs);
			CFG_PIN(cs, GPIOCFG_MODE_INP | (((cs) & BRD_GPIO_ACTIVE_LOW) ? GPIOCFG_PUPD_PUP : GPIOCFG_PUPD_PDN));
        }
        // put SCK, MISO, MOSI back to analog input (HiZ) mode
#if defined(BRD_sck_mosi_pulldown)
        CFG_PIN(GPIO_BRD_SCK, GPIOCFG_MODE_INP | GPIOCFG_PUPD_PDN);
        CFG_PIN(GPIO_BRD_MOSI, GPIOCFG_MODE_INP | GPIOCFG_PUPD_PDN);
#elif defined(BRD_sck_mosi_drivelow)
        SET_PIN(GPIO_BRD_SCK, 0);
        CFG_PIN(GPIO_BRD_SCK, GPIOCFG_MODE_OUT | GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_NONE);
        SET_PIN(GPIO_BRD_MOSI, 0);
        CFG_PIN(GPIO_BRD_MOSI, GPIOCFG_MODE_OUT | GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_NONE);
#else
        CFG_PIN_DEFAULT(GPIO_BRD_SCK);
        CFG_PIN_DEFAULT(GPIO_BRD_MOSI);
#endif
        CFG_PIN_DEFAULT(GPIO_BRD_MISO);
        // disable clock for SPI interface
        SPIx_disable();
    }
}

void spi_wait_rx() {
    while( (SPIx->SR & SPI_SR_RXNE ) == 0);
}

// perform SPI transaction
u1_t spi_xfer (u1_t out) {
    SPIx->DR = out;
    spi_wait_rx();
    return SPIx->DR; // in
}

void spi_tx(u1_t out) {
    while ((SPIx->SR & SPI_SR_TXE) == 0);
    SPIx->DR = out;
}

void spi_tx_buf(u1_t * buf, u2_t n) {
    while (n) {
        spi_tx(*buf);
        buf++;
        n--;
    }
    // wait for transfer to complete
    spi_wait_rx();
}
#endif
