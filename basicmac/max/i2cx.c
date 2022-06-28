// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.
#include "peripherals.h"
#include "i2c.h"
#include "i2cm.h"
#ifdef BRD_I2C

#if BRD_I2C == 0
#define I2Cx    MXC_I2CM0
#define I2C_MASTER_IDX      0
#define I2C_SPEED           I2CM_SPEED_100KHZ
//#define I2Cx_enable() do { RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; } while (0)
//#define I2Cx_disable()  do { RCC->APB1ENR &= ~RCC_APB1ENR_I2C1EN; } while (0)
//#define I2Cx_IRQn I2C1_IRQn

#elif BRD_I2C == 1
#define I2Cx		I2C1
#define I2Cx_enable()	do { RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; } while (0)
#define I2Cx_disable()	do { RCC->APB1ENR &= ~RCC_APB1ENR_I2C1EN; } while (0)
#define I2Cx_IRQn	I2C1_IRQn
#else
#error "Unsupported I2C peripheral"
#endif
mxc_i2cm_fifo_regs_t *fifo;
uint16_t i2c_addr;

static struct {
    unsigned int wlen;
    unsigned char* wptr;
    unsigned int rlen;
    unsigned char* rptr;
    osjob_t* job;
    osjobcb_t cb;
    int* pstatus;
} xfr;

static struct {
    int status;
    osjob_t job;
    i2c_cb cb;
} xfr2;

static void i2c_cont(void);
static void i2c_start (int addr);

void i2cm_callback_read(i2cm_req_t * req, int  error_code){
  debug_printf("i2cm_callback_read\r\n");
  i2c_cont();
}
void i2cm_callback_write(i2cm_req_t * req, int  error_code){
  debug_printf("i2cm_callback_write\r\n");
  i2c_cont();
}

i2cm_req_t request;

static void i2c_stop(int status) {
  NVIC_DisableIRQ(I2CM0_IRQn);
  I2CM_AbortAsync(&request);
  I2CM_Shutdown(I2Cx);
#if 0
    // generate stop condition
    I2Cx->CR2 |= I2C_CR2_STOP;
    // disable interrupts in NVIC
    NVIC_DisableIRQ(I2Cx_IRQn);
    // disable interrupts/peripheral
    I2Cx->CR1 = 0;
    // reconfigure GPIOs
    CFG_PIN_DEFAULT(GPIO_I2C_SCL);
    CFG_PIN_DEFAULT(GPIO_I2C_SDA);
    // disable peripheral clock
    I2Cx_disable();
#endif
    // schedule callback
    *(xfr.pstatus) = status;
    if (xfr.job != NULL) {
        os_setCallback(xfr.job, xfr.cb);
    } else {
        xfr.cb(NULL);
    }
    // re-enable sleep
    hal_clearMaxSleep(HAL_SLEEP_S0);
}

static void i2c_start (int addr) {
  uint32_t err;
  sys_cfg_i2cm_t i2cm_sys_cfg;

  //I2CM0A, SDA on pin P1.6
  //I2CM0A, SCL on pin P1.7

  // Set system level configurations
  i2cm_sys_cfg.clk_scale = CLKMAN_SCALE_DIV_1;
  i2cm_sys_cfg.io_cfg = (ioman_cfg_t)IOMAN_I2CM(I2C_MASTER_IDX, 1, 0);

  err = I2CM_Init(I2Cx, &i2cm_sys_cfg, I2C_SPEED);
  ASSERT(err==0);

  hal_setMaxSleep(HAL_SLEEP_S0);

  debug_printf("intfl: %x\r\n", ((mxc_i2cm_regs_t*)(I2Cx))->intfl);

  ((mxc_i2cm_regs_t*)(I2Cx))->intfl = ((mxc_i2cm_regs_t*)(I2Cx))->intfl;
  //Todo Enable interrupt in NVIC
  //NVIC_EnableIRQ(I2Cx_IRQn);
  NVIC_ClearPendingIRQ(I2CM0_IRQn);
  NVIC_EnableIRQ(I2CM0_IRQn);


#if 0
    // enable peripheral clock
    I2Cx_enable();
    // set timing
    I2Cx->TIMINGR = 0x40101A22; // from CubeMX tool; t_rise=t_fall=50ns, 100kHz
    // start I2C
    I2Cx->CR1 |= I2C_CR1_PE;
    // setup slave address
    I2Cx->CR2 = (I2Cx->CR2 & ~I2C_CR2_SADD) | (addr & I2C_CR2_SADD);
    // setup GPIOs
    CFG_PIN_AF(GPIO_I2C_SCL, GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_OPEN | GPIOCFG_PUPD_NONE);
    CFG_PIN_AF(GPIO_I2C_SDA, GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_OPEN | GPIOCFG_PUPD_NONE);
    // disable sleep (keep clock at full speed during transfer
    hal_setMaxSleep(HAL_SLEEP_S0);
    // enable interrupts in NVIC
    NVIC_EnableIRQ(I2Cx_IRQn);
#endif
}

static void i2c_cont(void) {
  int err;
  request.addr = i2c_addr;
  request.callback = NULL;
  request.cmd_data = NULL;
  request.cmd_len = 0;
  request.cmd_num = 0;
  request.data = NULL;
  request.data_len = 0;
  request.data_num = 0;
  debug_printf("i2c_cont: xfr.wlen:%x, xfr.rlen:%x\r\n", xfr.wlen, xfr.rlen);
    if (xfr.wlen) {
        // calculate length; TODO: handle >255
        int n = xfr.wlen & 0xff;
        xfr.wlen -= n;
        request.callback = i2cm_callback_write;
        request.data = xfr.wptr;
        request.data_len = n;
        err = I2CM_WriteAsync(I2Cx, &request);
        ASSERT(err==0);


//        err = I2CM_Write(I2Cx, i2c_addr, NULL, 0, xfr.wptr, n);
//        debug_printf("err: %x\r\n", err);
//        ASSERT(err==0);

//        debug_printf("I2CM_WriteAsync return: %x\r\n", err);
//        debug_printf("I2Cx->fs_clk_div:%x\r\n", I2Cx->fs_clk_div);
//        debug_printf("I2Cx->timeout:%x\r\n", I2Cx->timeout);
//        debug_printf("I2Cx->ctrl:%x\r\n", I2Cx->ctrl);
//        debug_printf("I2Cx->trans:%x\r\n", I2Cx->trans);
//        debug_printf("I2Cx->intfl:%x\r\n", I2Cx->intfl);
//        debug_printf("I2Cx->inten:%x\r\n", I2Cx->inten);
//        debug_printf("I2Cx->bb:%x\r\n", I2Cx->bb);
//        while(1){
//          debug_printf("I2Cx->intfl:%x\r\n", I2Cx->intfl);
//          debug_printf("I2Cx->trans:%x\r\n", I2Cx->trans);
//
//        }


#if 0
        // set direction & number of bytes
        I2Cx->CR2 = (I2Cx->CR2 & ~(I2C_CR2_RD_WRN | I2C_CR2_NBYTES)) | (n << 16);
        // enable interrupts
        I2Cx->CR1 = (I2Cx->CR1 & ~0xfe) | I2C_CR1_TXIE | I2C_CR1_TCIE | I2C_CR1_NACKIE | I2C_CR1_ERRIE;
        // start TX
        I2Cx->CR2 |= I2C_CR2_START;
#endif
    } else if (xfr.rlen) {
        // calculate length; TODO: handle >255
        int n = xfr.rlen & 0xff;
        xfr.rlen -= n;
        request.callback = i2cm_callback_read;
        request.data = xfr.rptr;
        request.data_len = n;
        err = I2CM_ReadAsync(I2Cx, &request);
        ASSERT(err==0);

#if 0
        // set direction & number of bytes
        I2Cx->CR2 = (I2Cx->CR2 & ~(I2C_CR2_RD_WRN | I2C_CR2_NBYTES)) | I2C_CR2_RD_WRN | (n << 16);
        // enable interrupts
        I2Cx->CR1 = (I2Cx->CR1 & ~0xfe) | I2C_CR1_RXIE | I2C_CR1_TCIE | I2C_CR1_NACKIE | I2C_CR1_ERRIE;
        // start RX
        I2Cx->CR2 |= I2C_CR2_START;
#endif
    } else {
        // done
        i2c_stop(I2C_OK);
    }
#endif
}

void i2c_irq(void) {
//  debug_printf("irq\r\n");
  I2CM_Handler(I2Cx);
  NVIC_ClearPendingIRQ(I2CM0_IRQn);
#if 0
    unsigned int isr = I2Cx->ISR;
    if (isr & I2C_ISR_NACKF) {
        // NACK detected, transfer failed!
        i2c_stop(I2C_NAK);
    } else if (isr & I2C_ISR_RXNE) {
        // next byte received
        *xfr.rptr++ = I2Cx->RXDR;
    } else if (isr & I2C_ISR_TXIS) {
        // write next byte
        I2Cx->TXDR = *xfr.wptr++;
    } else if (isr & I2C_ISR_TC) {
        // transfer complete, move on
        i2c_cont();
    } else {
        hal_failed(); // XXX
    }
#endif
}

static void i2c_timeout (osjob_t* job) {
    i2c_abort();
}

void i2c_xfer_ex(unsigned int addr, unsigned char* buf, unsigned int wlen,
        unsigned int rlen, ostime_t timeout, osjob_t* job, osjobcb_t cb,
        int* pstatus) {
    debug_printf("i2c_xfer_ex\r\n");
    // setup xfr structure
    xfr.wlen = wlen;
    xfr.rlen = rlen;
    xfr.wptr = xfr.rptr = buf;
    xfr.job = job;
    xfr.cb = cb;
    xfr.pstatus = pstatus;
    *xfr.pstatus = I2C_BUSY;
    // set timeout
    if (timeout) {
        os_setTimedCallback(job, os_getTime() + timeout, i2c_timeout);
    }
    i2c_addr = (addr >> 1);
    // prepare peripheral
    i2c_start(i2c_addr);
    // start actual transfer
    i2c_cont();
}

static void i2cfunc (osjob_t* j) {
    xfr2.cb(xfr2.status);
}

void i2c_xfer (unsigned int addr, unsigned char* buf, unsigned int wlen, unsigned int rlen, i2c_cb cb, ostime_t timeout) {
    xfr2.cb = cb;
    i2c_xfer_ex(addr, buf, wlen, rlen, timeout, &xfr2.job, i2cfunc, &xfr2.status);
}

void i2c_abort (void) {

#if 0
    hal_disableIRQs();
    i2c_stop(I2C_ABORT);
    hal_enableIRQs();
#endif
}
